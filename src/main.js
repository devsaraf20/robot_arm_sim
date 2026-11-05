/**
 * main.js — the "level blueprint" of the app.
 * It sets up the 3D scene, connects the UI buttons/sliders to the arm, handles
 * dragging the target, and runs the animation loop that redraws every frame.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ARM, JOINTS, LIMITS, HOME, solveIK, forwardKinematics, clamp, toDeg } from './ik.js';
import { createArm, JOINT_COLORS } from './arm.js';
import { createGridMaterial, createGlowMaterial } from './shaders.js';
import { createRecorder } from './recorder.js';
import { toPython, toArduino } from './exporters.js';

const $ = (id) => document.getElementById(id);
const DEG = Math.PI / 180;

/* ═══════════════════════════════════════════════════════════════════════════
 *  1. RENDERER, SCENE, CAMERA  (the viewport)
 * ═══════════════════════════════════════════════════════════════════════════ */
const canvas = $('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2)); // sharp on retina, capped for speed
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping; // same filmic curve idea as Blender's "Filmic"/AgX

const scene = new THREE.Scene();
scene.background = new THREE.Color('#0b0f17');
scene.fog = new THREE.Fog('#0b0f17', 12, 26);

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);
camera.position.set(5.5, 4.2, 6.5);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0.6, 1.3, 0);
controls.enableDamping = true; // smooth "inertia" when orbiting
controls.minDistance = 3;
controls.maxDistance = 18;
controls.maxPolarAngle = Math.PI * 0.495; // don't go below the floor

// Lights — a soft sky/ground fill plus one shadow-casting "sun" and a cool rim light.
scene.add(new THREE.HemisphereLight('#cfe3ff', '#1a1f2b', 1.1));
const sun = new THREE.DirectionalLight('#ffffff', 2.2);
sun.position.set(4, 8, 5);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: 1, far: 20 });
sun.shadow.bias = -0.0005;
scene.add(sun);
const rim = new THREE.DirectionalLight('#7dd3fc', 1.0);
rim.position.set(-6, 3, -4);
scene.add(rim);

// Floor: an invisible plane that only shows shadows + the procedural grid shader.
const shadowFloor = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), new THREE.ShadowMaterial({ opacity: 0.35 }));
shadowFloor.rotation.x = -Math.PI / 2;
shadowFloor.receiveShadow = true;
scene.add(shadowFloor);
const grid = new THREE.Mesh(new THREE.PlaneGeometry(30, 30), createGridMaterial());
grid.rotation.x = -Math.PI / 2;
grid.position.y = 0.001; // a hair above the shadow floor to avoid flicker (z-fighting)
scene.add(grid);

/* ═══════════════════════════════════════════════════════════════════════════
 *  2. THE ARM, THE TARGET, AND HELPER LINES
 * ═══════════════════════════════════════════════════════════════════════════ */
const arm = createArm();
scene.add(arm.root);

const STATUS_COLORS = { ok: '#22c55e', 'joint-limit': '#f59e0b', 'out-of-reach': '#ef4444' };

const glowMat = createGlowMaterial(STATUS_COLORS.ok);
const target = new THREE.Group();
const glow = new THREE.Mesh(new THREE.SphereGeometry(0.17, 32, 16), glowMat);
const core = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 8), new THREE.MeshBasicMaterial({ color: '#ffffff' }));
// An invisible, bigger sphere that's easier to hit with a finger.
const hitSphere = new THREE.Mesh(new THREE.SphereGeometry(0.38, 12, 8), new THREE.MeshBasicMaterial({ visible: false }));
// Draw the target ON TOP of everything (like an editor gizmo) so the gripper never hides it.
for (const m of [glow, core]) {
  m.material.depthTest = false;
  m.renderOrder = 10;
}
target.add(glow, core, hitSphere);
scene.add(target);

// Line from the target straight down to the floor + a little floor ring: depth cues.
const dropLine = makeLine(2, '#ffffff', 0.25);
const floorRing = new THREE.Mesh(
  new THREE.RingGeometry(0.12, 0.16, 32),
  new THREE.MeshBasicMaterial({ color: '#ffffff', transparent: true, opacity: 0.35, side: THREE.DoubleSide })
);
floorRing.rotation.x = -Math.PI / 2;
// Line from the gripper to the target, visible only when the arm can't reach it.
const gapLine = makeLine(2, STATUS_COLORS['out-of-reach'], 0.8);
// Recorded path of the gripper tip.
const MAX_TRAIL = 4000;
const trail = makeLine(MAX_TRAIL, '#a78bfa', 0.85);
scene.add(dropLine, floorRing, gapLine, trail);

function makeLine(maxPoints, color, opacity) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(maxPoints * 3), 3));
  geo.setDrawRange(0, 0);
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
}
function setLine(line, points) {
  const attr = line.geometry.attributes.position;
  const n = Math.min(points.length, attr.count);
  for (let i = 0; i < n; i++) attr.setXYZ(i, points[i].x, points[i].y, points[i].z);
  attr.needsUpdate = true;
  line.geometry.setDrawRange(0, n);
  line.geometry.computeBoundingSphere();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  3. APP STATE
 * ═══════════════════════════════════════════════════════════════════════════ */
const state = {
  mode: 'ik', // 'ik' = drag the target, 'sliders' = drive joints directly
  dragPlane: 'view', // 'view' | 'floor' | 'height'
  q: HOME.slice(), // angles currently SHOWN (radians)
  goal: HOME.slice(), // angles the arm is moving TOWARDS
  ik: { status: 'ok', error: 0 },
};
const recorder = createRecorder();

// Start the target where the gripper is.
target.position.copy(vec(forwardKinematics(HOME).tip));
arm.setAngles(state.q);

function vec(p) {
  return new THREE.Vector3(p.x, p.y, p.z);
}

/** Run the IK solver for the current target position. */
function solveForTarget() {
  const res = solveIK(target.position, state.goal);
  state.goal = res.q;
  state.ik = res;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  4. UI: JOINT SLIDERS
 * ═══════════════════════════════════════════════════════════════════════════ */
const sliders = JOINTS.map((j, i) => {
  const wrap = document.createElement('div');
  wrap.className = 'joint';
  wrap.style.setProperty('--jc', JOINT_COLORS[i]);
  wrap.innerHTML = `
    <div class="joint-head">
      <span class="joint-name"><i></i>${j.name}</span>
      <span class="joint-val"><b>0°</b><small>servo 0°</small></span>
    </div>
    <input type="range" min="${j.min}" max="${j.max}" step="0.5" aria-label="${j.name} angle in degrees" />`;
  $('joints').appendChild(wrap);
  const input = wrap.querySelector('input');
  const [valEl, servoEl] = wrap.querySelectorAll('.joint-val b, .joint-val small');

  input.addEventListener('input', () => {
    stopPlayback();
    setMode('sliders');
    state.goal[i] = Number(input.value) * DEG;
  });
  // Light up the matching joint on the 3D model while interacting.
  const on = () => arm.highlight(i);
  const off = () => arm.highlight(-1);
  wrap.addEventListener('pointerenter', on);
  wrap.addEventListener('pointerleave', off);
  input.addEventListener('focus', on);
  input.addEventListener('blur', off);

  let shown = null;
  return {
    update(rad) {
      const deg = rad / DEG;
      if (shown !== null && Math.abs(deg - shown) < 0.05) return; // skip needless DOM work
      shown = deg;
      if (document.activeElement !== input || state.mode !== 'sliders') input.value = deg.toFixed(1);
      input.style.setProperty('--fill', `${((deg - j.min) / (j.max - j.min)) * 100}%`);
      valEl.textContent = `${deg.toFixed(1)}°`;
      servoEl.textContent = `servo ${(deg - j.min).toFixed(0)}°`;
    },
  };
});

/* ═══════════════════════════════════════════════════════════════════════════
 *  5. UI: MODE SWITCH, DRAG PLANE, HOME
 * ═══════════════════════════════════════════════════════════════════════════ */
function setMode(mode) {
  if (state.mode === mode) return;
  state.mode = mode;
  for (const b of $('modeSwitch').querySelectorAll('button')) b.setAttribute('aria-selected', String(b.dataset.mode === mode));
  $('ikOptions').style.display = mode === 'ik' ? '' : 'none';
  if (mode === 'ik') {
    // Put the target where the gripper currently is, so nothing jumps.
    target.position.copy(vec(forwardKinematics(state.q).tip));
    state.goal = state.q.slice();
    solveForTarget();
  }
}
$('modeSwitch').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (b) { stopPlayback(); setMode(b.dataset.mode); }
});
$('dragPlane').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  state.dragPlane = b.dataset.plane;
  for (const x of $('dragPlane').querySelectorAll('button')) x.setAttribute('aria-pressed', String(x === b));
});
$('homeBtn').addEventListener('click', () => {
  stopPlayback();
  state.goal = HOME.slice();
  if (state.mode === 'ik') {
    target.position.copy(vec(forwardKinematics(HOME).tip));
    solveForTarget();
  }
});

/* ═══════════════════════════════════════════════════════════════════════════
 *  6. DRAGGING THE TARGET IN 3D
 * ═══════════════════════════════════════════════════════════════════════════
 *  The mouse lives on a flat 2D screen, but the target lives in 3D. To turn a
 *  2D drag into a 3D move we:
 *    1. shoot a RAY from the camera through the mouse pointer (raycasting),
 *    2. intersect that ray with an imaginary PLANE passing through the target,
 *    3. move the target to where the ray hits the plane.
 *  Which plane we use decides how it moves:
 *    Free       – plane faces the camera  → moves like dragging on the screen glass
 *    Horizontal – floor-parallel plane    → slides at constant height
 *    Height     – upright plane           → only goes up/down
 */
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const dragPlane = new THREE.Plane();
const hitPoint = new THREE.Vector3();
const grabOffset = new THREE.Vector3();
let dragging = null; // pointerId while dragging

function setPointer(e) {
  const rect = canvas.getBoundingClientRect();
  // Convert pixels → "normalised device coordinates": −1…1 across the canvas.
  pointer.set(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
}

function buildDragPlane() {
  const normal = new THREE.Vector3();
  camera.getWorldDirection(normal);
  if (state.dragPlane === 'floor') normal.set(0, 1, 0);
  else if (state.dragPlane === 'height') normal.setY(0).normalize(); // upright, facing camera
  dragPlane.setFromNormalAndCoplanarPoint(normal, target.position);
}

// Capture-phase listener runs BEFORE OrbitControls' own listener, so we can
// claim the gesture (and switch orbiting off) when the target is grabbed.
canvas.addEventListener(
  'pointerdown',
  (e) => {
    if (e.button !== 0 && e.pointerType === 'mouse') return;
    setPointer(e);
    if (raycaster.intersectObject(hitSphere).length === 0) return;
    stopPlayback();
    setMode('ik');
    buildDragPlane();
    if (!raycaster.ray.intersectPlane(dragPlane, hitPoint)) return;
    grabOffset.subVectors(target.position, hitPoint);
    dragging = e.pointerId;
    controls.enabled = false;
    canvas.setPointerCapture(e.pointerId);
    canvas.style.cursor = 'grabbing';
    $('hint').classList.add('hidden');
    e.stopImmediatePropagation();
  },
  { capture: true }
);

canvas.addEventListener('pointermove', (e) => {
  setPointer(e);
  if (dragging === null) {
    canvas.style.cursor = raycaster.intersectObject(hitSphere).length ? 'grab' : '';
    return;
  }
  if (e.pointerId !== dragging || !raycaster.ray.intersectPlane(dragPlane, hitPoint)) return;
  const p = hitPoint.add(grabOffset);
  if (state.dragPlane === 'height') p.set(target.position.x, p.y, target.position.z);
  // Keep the target in a sensible box: above the floor and not miles away.
  p.y = clamp(p.y, 0.05, 4.5);
  const horiz = Math.hypot(p.x, p.z);
  if (horiz > 4) p.multiply(new THREE.Vector3(4 / horiz, 1, 4 / horiz));
  target.position.copy(p);
  solveForTarget();
});

const endDrag = (e) => {
  if (e.pointerId !== dragging) return;
  dragging = null;
  controls.enabled = true;
  canvas.style.cursor = '';
};
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);

/* ═══════════════════════════════════════════════════════════════════════════
 *  7. RECORDER UI
 * ═══════════════════════════════════════════════════════════════════════════ */
function stopPlayback() {
  if (recorder.state === 'playing') {
    recorder.stop();
    if (state.mode === 'ik') {
      target.position.copy(vec(forwardKinematics(state.q).tip));
    }
    refreshRecorderUI();
  }
}

function refreshRecorderUI() {
  const s = recorder.state;
  const n = recorder.frames.length;
  $('recBtn').classList.toggle('active', s === 'recording');
  $('recBtn').querySelector('.txt').textContent = s === 'recording' ? 'Stop' : 'Record';
  $('playBtn').querySelector('.txt').textContent = s === 'playing' ? '■ Stop' : '▶ Play';
  $('playBtn').disabled = s === 'recording' || n < 2;
  $('clearBtn').disabled = s === 'recording' || n === 0;
  $('exportPy').disabled = $('exportIno').disabled = s === 'recording' || n < 2;
  $('exportHint').textContent =
    n < 2 ? 'Record a motion first.' : 'Exports the recorded joint angles, resampled at a fixed rate.';
  $('recInfo').textContent =
    s === 'recording'
      ? `Recording… ${recorder.time.toFixed(1)} s`
      : n
        ? `${recorder.duration.toFixed(1)} s · ${n} keyframes`
        : 'No recording yet';
  if (s !== 'recording') updateTrail();
}

/** Draw the recorded path of the gripper tip (computed with forward kinematics). */
function updateTrail() {
  setLine(trail, recorder.frames.map((f) => forwardKinematics(f.q).tip));
}

$('recBtn').addEventListener('click', () => {
  if (recorder.state === 'recording') recorder.stop();
  else {
    stopPlayback();
    recorder.startRecording();
  }
  refreshRecorderUI();
});
$('playBtn').addEventListener('click', () => {
  if (recorder.state === 'playing') stopPlayback();
  else recorder.play();
  refreshRecorderUI();
});
$('clearBtn').addEventListener('click', () => {
  recorder.clear();
  refreshRecorderUI();
});
$('loopChk').addEventListener('change', (e) => (recorder.loop = e.target.checked));

/* ═══════════════════════════════════════════════════════════════════════════
 *  8. EXPORT DIALOG
 * ═══════════════════════════════════════════════════════════════════════════ */
const dialog = $('exportDialog');
let exportKind = 'python';
let exportText = '';

function renderExport() {
  const stepMs = Number($('rateSel').value);
  const frames = recorder.resample(stepMs / 1000).map(toDeg);
  exportText = exportKind === 'python' ? toPython(frames, stepMs) : toArduino(frames, stepMs);
  $('exportCode').textContent = exportText;
  const flashBytes = frames.length * 6;
  $('exportStats').textContent =
    `${frames.length} frames` +
    (exportKind === 'arduino'
      ? ` · ~${(flashBytes / 1024).toFixed(1)} KB flash${flashBytes > 24000 ? ' — too big for an Uno, lower the rate' : ''}`
      : '');
}
function openExport(kind) {
  exportKind = kind;
  $('exportTitle').textContent = kind === 'python' ? 'Python script' : 'Arduino sketch';
  renderExport();
  dialog.showModal();
}
$('exportPy').addEventListener('click', () => openExport('python'));
$('exportIno').addEventListener('click', () => openExport('arduino'));
$('rateSel').addEventListener('change', renderExport);

$('copyBtn').addEventListener('click', async () => {
  try {
    await navigator.clipboard.writeText(exportText);
    $('copyBtn').textContent = 'Copied!';
  } catch {
    $('copyBtn').textContent = 'Copy failed';
  }
  setTimeout(() => ($('copyBtn').textContent = 'Copy'), 1500);
});
$('downloadBtn').addEventListener('click', () => {
  // Make a file in memory (a "Blob") and click a temporary link to download it.
  const name = exportKind === 'python' ? 'robot_arm_motion.py' : 'robot_arm_motion.ino';
  const url = URL.createObjectURL(new Blob([exportText], { type: 'text/plain' }));
  const a = Object.assign(document.createElement('a'), { href: url, download: name });
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

/* ═══════════════════════════════════════════════════════════════════════════
 *  9. LAYOUT: resize + keep the arm visible next to/above the panel
 * ═══════════════════════════════════════════════════════════════════════════ */
const panel = $('panel');
$('panelToggle').addEventListener('click', () => {
  const collapsed = panel.classList.toggle('collapsed');
  $('panelToggle').setAttribute('aria-expanded', String(!collapsed));
  resize();
});

function resize() {
  const w = window.innerWidth;
  const h = window.innerHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  // Shift the picture so the arm is centred in the area NOT covered by the panel.
  const r = panel.getBoundingClientRect();
  const mobile = w <= 720;
  const offX = mobile ? 0 : (w - r.left) / 2;
  const offY = mobile ? (h - r.top) / 2 : 0;
  camera.setViewOffset(w, h, offX, offY, w, h);
  // Pull the camera back a little on narrow/tall screens so the whole arm fits.
  camera.fov = w / h < 0.8 ? 60 : 45;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
new ResizeObserver(resize).observe(panel);
resize();

/* ═══════════════════════════════════════════════════════════════════════════
 *  10. THE ANIMATION LOOP — runs ~60 times per second
 * ═══════════════════════════════════════════════════════════════════════════ */
const clock = new THREE.Clock();
const statusEl = $('status');
const STATUS_TEXT = {
  ok: 'IK solved',
  'joint-limit': 'Joint limit',
  'out-of-reach': 'Out of reach',
  manual: 'Manual control',
  playing: 'Playing',
  recording: '● Recording',
};

function tick() {
  const dt = Math.min(clock.getDelta(), 0.05); // seconds since last frame (capped)
  const elapsed = clock.elapsedTime;

  if (recorder.state === 'playing') {
    // Playback drives the joints directly (no smoothing, so it's exact).
    const q = recorder.advance(dt);
    if (q) state.q = q.slice();
    state.goal = state.q.slice();
    $('timelineFill').style.width = `${(recorder.time / (recorder.duration || 1)) * 100}%`;
    if (recorder.state !== 'playing') refreshRecorderUI(); // finished
  } else {
    // Ease towards the goal like a real servo: cover a fixed fraction of the
    // remaining distance each frame (exponential smoothing, frame-rate independent).
    const k = 1 - Math.exp(-dt * 12);
    state.q = state.q.map((v, i) => clamp(v + (state.goal[i] - v) * k, LIMITS[i][0], LIMITS[i][1]));
    $('timelineFill').style.width = recorder.state === 'recording' ? '100%' : '0%';
  }

  if (recorder.state === 'recording') {
    recorder.capture(dt, state.q);
    $('recInfo').textContent = `Recording… ${recorder.time.toFixed(1)} s`;
  }

  arm.setAngles(state.q);
  sliders.forEach((s, i) => s.update(state.q[i]));

  // In sliders mode / playback the target simply rides on the gripper.
  const tip = vec(forwardKinematics(state.q).tip);
  const following = state.mode === 'sliders' || recorder.state === 'playing';
  if (following) target.position.copy(tip);

  // Status colour + gap line.
  const status =
    recorder.state === 'recording' ? 'recording'
      : recorder.state === 'playing' ? 'playing'
        : state.mode === 'sliders' ? 'manual'
          : state.ik.status;
  if (statusEl.dataset.status !== status) {
    statusEl.dataset.status = status;
    statusEl.textContent = STATUS_TEXT[status];
  }
  const ikColor = STATUS_COLORS[following ? 'ok' : state.ik.status];
  glowMat.uniforms.uColor.value.set(ikColor);
  glowMat.uniforms.uTime.value = elapsed;
  const gap = !following && state.ik.status !== 'ok';
  gapLine.visible = gap;
  if (gap) {
    gapLine.material.color.set(ikColor);
    setLine(gapLine, [tip, target.position]);
  }

  // Depth cues under the target.
  const p = target.position;
  setLine(dropLine, [p, new THREE.Vector3(p.x, 0.002, p.z)]);
  floorRing.position.set(p.x, 0.003, p.z);
  $('coords').textContent = `x ${p.x.toFixed(2)} · y ${p.y.toFixed(2)} · z ${p.z.toFixed(2)}`;

  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

solveForTarget();
refreshRecorderUI();
requestAnimationFrame(tick);

// Handy for debugging in the browser console: try `sim.state` or `sim.ARM`.
window.sim = { state, recorder, ARM, target, camera, solveForTarget };
