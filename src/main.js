/**
 * main.js — the "level blueprint" of the app.
 * It sets up the 3D scene, connects the UI to the arm, handles dragging the target,
 * talks to the real arm (Web Serial) and Unreal (OSC bridge), applies the safety
 * rules, and runs the animation loop that redraws every frame.
 *
 * WHO DRIVES THE ARM? Several things can want to move it. Every frame we pick ONE,
 * in this priority order (highest first):
 *   1. Emergency stop   – nothing moves
 *   2. Homing           – slow move to HOME
 *   3. Real arm (read)  – the 3D model mirrors the physical arm
 *   4. Recording playback
 *   5. You / Unreal     – dragging the target, the sliders, or OSC messages
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import {
  ARM, JOINTS, LIMITS, HOME, IK_JOINTS, solveIK, forwardKinematics, clamp, toDeg, simToServo, servoToSim,
} from './ik.js';
import { createArm, JOINT_COLORS } from './arm.js';
import { createGridMaterial, createGlowMaterial } from './shaders.js';
import { createRecorder } from './recorder.js';
import { toPython, toArduino } from './exporters.js';
import { createSafety, stepTowards, SPEED_RANGE } from './safety.js';
import { createSerialLink, serialSupported } from './serial.js';
import { cmd } from './protocol.js';
import { createUnrealLink, unrealToSim, simToUnreal } from './unreal-link.js';

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
 *  2. THE ARM, ITS GHOST, THE TARGET, AND HELPER LINES
 * ═══════════════════════════════════════════════════════════════════════════ */
const arm = createArm();
scene.add(arm.root);

// The "ghost" shows where the REAL arm says it is (digital twin), on top of the sim.
const ghost = createArm({ ghost: true });
ghost.root.visible = false;
scene.add(ghost.root);

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
  q: HOME.slice(), // angles currently SHOWN (radians): base, shoulder, elbow, gripper
  goal: HOME.slice(), // angles the arm is moving TOWARDS
  ik: { status: 'ok', error: 0 },
  twin: {
    mode: 'send', // 'send' = sim drives the real arm, 'read' = real arm drives the sim
    measured: null, // last pose reported by the real arm (radians)
    measuredAt: 0,
    firmware: null, // { firmware, version, joints, feedback } from the HELLO line
    deviceState: '',
  },
};
const recorder = createRecorder();

// Start the target where the gripper is.
target.position.copy(vec(forwardKinematics(HOME).tip));
arm.setAngles(state.q);

function vec(p) {
  return new THREE.Vector3(p.x, p.y, p.z);
}

/** Keep a target inside a sensible box: above the floor and not miles away. */
function clampTarget(p) {
  p.y = clamp(p.y, 0.05, 4.5);
  const horiz = Math.hypot(p.x, p.z);
  if (horiz > 4) { p.x *= 4 / horiz; p.z *= 4 / horiz; }
  return p;
}

/** Run the IK solver for the current target position (keeps the gripper as it is). */
function solveForTarget() {
  const res = solveIK(target.position, state.goal);
  state.goal = [...res.q, ...state.goal.slice(IK_JOINTS)];
  state.ik = res;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  4. SAFETY (see safety.js) — e-stop, homing, speed limit
 * ═══════════════════════════════════════════════════════════════════════════ */
const safety = createSafety({
  onChange(s, reason) {
    if (s === 'estop') {
      state.goal = state.q.slice(); // freeze exactly where we are
      stopPlayback();
      if (recorder.state === 'recording') recorder.stop();
      serial.send(cmd.estop());
      unreal.send('/arm/status', ['estop']);
      $('estopReason').textContent = `All motion halted (${reason}). Check the arm, then Reset.`;
    }
    if (s === 'normal' && reason === 'reset') {
      // If the real arm can tell us where it is now (it may have sagged while limp),
      // start from THERE so nothing jumps when motors re-engage.
      if (state.twin.measured && performance.now() - state.twin.measuredAt < 500) {
        state.q = state.twin.measured.slice();
      }
      state.goal = state.q.slice();
      if (state.mode === 'ik') syncTargetToTip();
    }
    $('estopBanner').hidden = s !== 'estop';
    $('estopBtn').classList.toggle('active', s === 'estop');
    refreshLocks();
    refreshRecorderUI();
  },
});

/** Can a person / Unreal / playback move the arm right now? */
const reading = () => serial.connected && state.twin.mode === 'read';
const canDrive = () => safety.acceptsInput && !reading();

function emergencyStop(reason) {
  safety.estop(reason);
}
function startHoming() {
  if (!safety.startHoming()) return;
  stopPlayback();
  if (state.twin.mode === 'read') setTwinMode('send');
  serial.send(cmd.home());
}

$('estopBtn').addEventListener('click', () => emergencyStop('stop button'));
$('resetBtn').addEventListener('click', () => {
  serial.send(cmd.clear()); // only a human clicking here can clear an e-stop
  safety.reset();
});
window.addEventListener('keydown', (e) => {
  if (e.key === 'Escape') emergencyStop('Esc key');
});
$('homeBtn').addEventListener('click', startHoming);

const speedSlider = $('speedSlider');
speedSlider.min = SPEED_RANGE.min;
speedSlider.max = SPEED_RANGE.max;
speedSlider.value = SPEED_RANGE.default;
function applySpeed() {
  safety.maxSpeed = Number(speedSlider.value);
  $('speedVal').textContent = `${safety.maxSpeed}°/s`;
  speedSlider.style.setProperty('--fill', `${((safety.maxSpeed - SPEED_RANGE.min) / (SPEED_RANGE.max - SPEED_RANGE.min)) * 100}%`);
}
speedSlider.addEventListener('input', applySpeed);
speedSlider.addEventListener('change', () => serial.send(cmd.speed(safety.maxSpeed)));
applySpeed();

/* ═══════════════════════════════════════════════════════════════════════════
 *  5. UI: JOINT SLIDERS
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
  const isGripper = i >= IK_JOINTS;

  input.addEventListener('input', () => {
    if (!canDrive()) return;
    stopPlayback();
    // The gripper isn't part of IK, so it can be used in either mode.
    if (!isGripper) setMode('sliders');
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
    input,
    update(rad) {
      const deg = rad / DEG;
      if (shown !== null && Math.abs(deg - shown) < 0.05) return; // skip needless DOM work
      shown = deg;
      const userHolding = document.activeElement === input && canDrive() && (isGripper || state.mode === 'sliders');
      if (!userHolding) input.value = deg.toFixed(1);
      input.style.setProperty('--fill', `${((deg - j.min) / (j.max - j.min)) * 100}%`);
      valEl.textContent = `${deg.toFixed(1)}°`;
      servoEl.textContent = `servo ${(deg - j.min).toFixed(0)}°`;
    },
  };
});

/** Grey out controls that can't be used right now (e-stop, homing, read mode). */
function refreshLocks() {
  const locked = !canDrive();
  for (const s of sliders) s.input.disabled = locked;
  for (const b of $('modeSwitch').querySelectorAll('button')) b.disabled = locked;
  $('homeBtn').disabled = safety.estopped || safety.homing;
  $('twinMode').querySelectorAll('button').forEach((b) => (b.disabled = !serial.connected || !safety.acceptsInput));
  $('calBtn').disabled = !serial.connected || !state.twin.firmware?.feedback || !safety.acceptsInput || reading();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  6. UI: MODE SWITCH, DRAG PLANE
 * ═══════════════════════════════════════════════════════════════════════════ */
function syncTargetToTip() {
  target.position.copy(vec(forwardKinematics(state.q).tip));
}
function setMode(mode) {
  if (state.mode === mode) return;
  state.mode = mode;
  for (const b of $('modeSwitch').querySelectorAll('button')) b.setAttribute('aria-selected', String(b.dataset.mode === mode));
  $('ikOptions').style.display = mode === 'ik' ? '' : 'none';
  if (mode === 'ik') {
    // Put the target where the gripper currently is, so nothing jumps.
    syncTargetToTip();
    state.goal = state.q.slice();
    solveForTarget();
  }
}
$('modeSwitch').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (b && canDrive()) { stopPlayback(); setMode(b.dataset.mode); }
});
$('dragPlane').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  state.dragPlane = b.dataset.plane;
  for (const x of $('dragPlane').querySelectorAll('button')) x.setAttribute('aria-pressed', String(x === b));
});

/* ═══════════════════════════════════════════════════════════════════════════
 *  7. DRAGGING THE TARGET IN 3D
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
    if (!canDrive()) return; // locked: let the camera orbit instead
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
    canvas.style.cursor = canDrive() && raycaster.intersectObject(hitSphere).length ? 'grab' : '';
    return;
  }
  if (!canDrive()) return endDrag(e);
  if (e.pointerId !== dragging || !raycaster.ray.intersectPlane(dragPlane, hitPoint)) return;
  const p = hitPoint.add(grabOffset);
  if (state.dragPlane === 'height') p.set(target.position.x, p.y, target.position.z);
  target.position.copy(clampTarget(p));
  solveForTarget();
});

function endDrag(e) {
  if (e.pointerId !== dragging) return;
  dragging = null;
  controls.enabled = true;
  canvas.style.cursor = '';
}
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);

/* ═══════════════════════════════════════════════════════════════════════════
 *  8. DIGITAL TWIN — live USB link to the real arm (serial.js, protocol.js)
 * ═══════════════════════════════════════════════════════════════════════════ */
const serialLogEl = $('serialLog');
const logLines = [];
function log(dir, text) {
  logLines.push(`${dir} ${text}`);
  if (logLines.length > 60) logLines.shift();
  serialLogEl.textContent = logLines.join('\n');
  serialLogEl.scrollTop = serialLogEl.scrollHeight;
}

const serial = createSerialLink({
  onLine: (dir, line) => { if (!line.startsWith('P ')) log(dir, line); }, // skip the 25 Hz position spam
  onStatus(status, detail) {
    if (status === 'connected') {
      $('serialStatus').textContent = 'Connected. Waiting for firmware…';
    } else if (status === 'disconnected' || status === 'error') {
      state.twin.firmware = null;
      state.twin.measured = null;
      if (state.twin.mode === 'read') setTwinMode('send');
      $('serialStatus').textContent = status === 'error' ? `Connection error: ${detail}` : `Disconnected${detail ? ` (${detail})` : ''}.`;
    }
    $('serialBtn').textContent = serial.connected ? 'Disconnect' : 'Connect via USB';
    $('serialBtn').classList.toggle('primary', !serial.connected);
    refreshLinkDots();
    refreshLocks();
  },
  onMessage(msg) {
    const tw = state.twin;
    switch (msg.type) {
      case 'hello':
        tw.firmware = msg;
        $('serialStatus').textContent =
          `Connected · ${msg.firmware} v${msg.version} · ${msg.joints} servos · ` +
          (msg.feedback ? 'position feedback ✓' : 'no feedback (send only)');
        serial.send(cmd.speed(safety.maxSpeed));
        serial.send(tw.mode === 'read' ? cmd.read() : cmd.write());
        if (safety.estopped) serial.send(cmd.estop()); // never let the arm forget an active e-stop
        refreshLocks();
        refreshLinkDots();
        break;
      case 'position': {
        const servo = JOINTS.map((j, i) => msg.angles[i] ?? simToServo(state.q)[i]);
        tw.measured = servoToSim(servo);
        tw.measuredAt = performance.now();
        break;
      }
      case 'state':
        tw.deviceState = msg.state;
        // The physical stop button on the arm stops the browser too.
        if (msg.state.startsWith('ESTOP') && !safety.estopped) emergencyStop('arm button / firmware');
        // The firmware homes itself after a reset; mirror that in the sim.
        if (msg.state === 'HOMING' && safety.acceptsInput) safety.startHoming();
        break;
      case 'error':
        if (/read mode/i.test(msg.text)) setTwinMode('send');
        $('serialStatus').textContent = `Arm says: ${msg.text}`;
        break;
    }
  },
});

if (!serialSupported()) {
  $('serialUnsupported').hidden = false;
  $('serialBtn').disabled = true;
}

$('serialBtn').addEventListener('click', async () => {
  if (serial.connected) return serial.disconnect();
  try {
    await serial.connect();
  } catch (err) {
    // The user closing the device picker throws "NotFoundError": not worth shouting about.
    $('serialStatus').textContent = err.name === 'NotFoundError' ? 'No device selected.' : err.message;
  }
});

function setTwinMode(mode) {
  state.twin.mode = mode;
  for (const b of $('twinMode').querySelectorAll('button')) b.setAttribute('aria-pressed', String(b.dataset.twin === mode));
  if (mode === 'read') {
    stopPlayback();
    serial.send(cmd.read());
  } else {
    // Hand control back to the sim from wherever the real arm was left.
    state.goal = state.q.slice();
    if (state.mode === 'ik') { syncTargetToTip(); solveForTarget(); }
    serial.send(cmd.write());
  }
  refreshLocks();
}
$('twinMode').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (b && !b.disabled) setTwinMode(b.dataset.twin);
});
$('calBtn').addEventListener('click', () => {
  if (confirm('Auto-calibration moves every joint slowly between its limits. Clear the area around the arm. Continue?')) {
    serial.send(cmd.calibrate());
  }
});

/* ═══════════════════════════════════════════════════════════════════════════
 *  9. UNREAL ENGINE — OSC via the local bridge (unreal-link.js)
 * ═══════════════════════════════════════════════════════════════════════════ */
const unreal = createUnrealLink({
  onStatus(status) {
    const text = {
      connecting: 'Connecting…',
      connected: 'Connected to bridge. Waiting for Unreal…',
      retrying: 'Bridge not reachable. Is `npm run bridge` running? Retrying…',
      disconnected: 'Not connected.',
      error: 'Could not connect.',
    }[status];
    $('oscStatus').textContent = text ?? status;
    $('oscBtn').textContent = status === 'disconnected' ? 'Connect' : 'Disconnect';
    refreshLinkDots();
  },
  onMessage({ address, args }) {
    const nums = args.filter((a) => typeof a === 'number' && Number.isFinite(a));
    $('oscStatus').textContent = `Receiving · last: ${address} ${nums.map((n) => n.toFixed(1)).join(' ')}`;
    // E-stop is ALWAYS honoured. Unreal can trigger it but never clear it.
    if (address === '/arm/estop') return emergencyStop('Unreal');
    if (address === '/arm/home') return startHoming();
    if (!canDrive()) return;
    if (address === '/arm/target' && nums.length >= 3) {
      stopPlayback();
      setMode('ik');
      target.position.copy(clampTarget(vec(unrealToSim(nums))));
      solveForTarget();
    } else if (address === '/arm/joints' && nums.length >= 3) {
      stopPlayback();
      setMode('sliders');
      nums.slice(0, JOINTS.length).forEach((d, i) => (state.goal[i] = clamp(d * DEG, LIMITS[i][0], LIMITS[i][1])));
    } else if (address === '/arm/gripper' && nums.length >= 1) {
      state.goal[3] = clamp(nums[0], 0, 1) * LIMITS[3][1];
    }
  },
});
$('oscBtn').addEventListener('click', () => {
  if ($('oscBtn').textContent === 'Connect') unreal.connect($('oscUrl').value.trim());
  else unreal.disconnect();
});

function refreshLinkDots() {
  const s = $('serialDot');
  s.className = 'link-dot' + (state.twin.firmware ? ' on' : serial.connected ? ' busy' : '');
  s.title = state.twin.firmware ? 'Real arm: connected' : serial.connected ? 'Real arm: waiting for firmware' : 'Real arm: not connected';
  const o = $('oscDot');
  o.className = 'link-dot' + (unreal.connected ? ' on' : '');
  o.title = unreal.connected ? 'Unreal bridge: connected' : 'Unreal bridge: not connected';
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  10. RECORDER UI
 * ═══════════════════════════════════════════════════════════════════════════ */
function stopPlayback() {
  if (recorder.state === 'playing') {
    recorder.stop();
    state.goal = state.q.slice();
    if (state.mode === 'ik') syncTargetToTip();
    refreshRecorderUI();
  }
}

function refreshRecorderUI() {
  const s = recorder.state;
  const n = recorder.frames.length;
  $('recBtn').classList.toggle('active', s === 'recording');
  $('recBtn').querySelector('.txt').textContent = s === 'recording' ? 'Stop' : 'Record';
  $('recBtn').disabled = safety.estopped;
  $('playBtn').querySelector('.txt').textContent = s === 'playing' ? '■ Stop' : '▶ Play';
  $('playBtn').disabled = s === 'recording' || n < 2 || (s !== 'playing' && !canDrive());
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
  else if (canDrive()) recorder.play();
  refreshRecorderUI();
});
$('clearBtn').addEventListener('click', () => {
  recorder.clear();
  refreshRecorderUI();
});
$('loopChk').addEventListener('change', (e) => (recorder.loop = e.target.checked));

/* ═══════════════════════════════════════════════════════════════════════════
 *  11. EXPORT DIALOG
 * ═══════════════════════════════════════════════════════════════════════════ */
const dialog = $('exportDialog');
let exportKind = 'python';
let exportText = '';

function renderExport() {
  const stepMs = Number($('rateSel').value);
  const frames = recorder.resample(stepMs / 1000).map(toDeg);
  exportText = exportKind === 'python' ? toPython(frames, stepMs) : toArduino(frames, stepMs);
  $('exportCode').textContent = exportText;
  const flashBytes = frames.length * JOINTS.length * 2;
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
 *  12. LAYOUT: resize + keep the arm visible next to/above the panel
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
 *  13. THE ANIMATION LOOP — runs ~60 times per second
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
  estop: 'E-STOP',
  homing: 'Homing…',
  reading: 'Mirroring real arm',
  unreal: 'Driven by Unreal',
};
let lastOscSend = 0;
let lastOscStatus = '';

function tick() {
  const dt = Math.min(clock.getDelta(), 0.05); // seconds since last frame (capped)
  const elapsed = clock.elapsedTime;
  const tw = state.twin;

  // ── 1. Decide the goal pose (see the priority list at the top of this file) ──
  let speed = safety.effectiveSpeed;
  let ease = 12;
  if (safety.estopped) {
    state.goal = state.q.slice();
  } else if (safety.homing) {
    state.goal = HOME.slice();
  } else if (reading()) {
    // Mirror the real arm. It's already moving at real speed, so allow fast following
    // with light smoothing to hide sensor noise.
    if (tw.measured) state.goal = tw.measured.slice();
    speed = SPEED_RANGE.max * 2;
    ease = 20;
  } else if (recorder.state === 'playing') {
    const q = recorder.advance(dt);
    if (q) state.goal = q.slice();
    $('timelineFill').style.width = `${(recorder.time / (recorder.duration || 1)) * 100}%`;
    if (recorder.state !== 'playing') refreshRecorderUI(); // finished
  }
  if (recorder.state !== 'playing') {
    $('timelineFill').style.width = recorder.state === 'recording' ? '100%' : '0%';
  }

  // ── 2. Move towards it, never faster than the speed limit ──
  if (!safety.estopped) state.q = stepTowards(state.q, state.goal, dt, speed, ease);
  safety.updateHoming(state.q);

  if (recorder.state === 'recording') {
    recorder.capture(dt, state.q);
    $('recInfo').textContent = `Recording… ${recorder.time.toFixed(1)} s`;
  }

  arm.setAngles(state.q);
  sliders.forEach((s, i) => s.update(state.q[i]));

  // ── 3. Digital twin: stream to the real arm / show its ghost ──
  if (serial.connected && tw.mode === 'send' && !safety.estopped && tw.firmware && tw.deviceState !== 'HOMING') {
    serial.sendTarget(simToServo(state.q));
  }
  const ghostOn = $('ghostChk').checked && tw.mode === 'send' && tw.measured && performance.now() - tw.measuredAt < 1000;
  ghost.root.visible = !!ghostOn;
  if (ghostOn) ghost.setAngles(tw.measured);

  // ── 4. Target, status, helper lines ──
  const tip = vec(forwardKinematics(state.q).tip);
  const following = state.mode === 'sliders' || recorder.state === 'playing' || !canDrive();
  if (following) target.position.copy(tip);

  const status =
    safety.estopped ? 'estop'
      : safety.homing ? 'homing'
        : reading() ? 'reading'
          : recorder.state === 'recording' ? 'recording'
            : recorder.state === 'playing' ? 'playing'
              : unreal.active() ? 'unreal'
                : state.mode === 'sliders' ? 'manual'
                  : state.ik.status;
  if (statusEl.dataset.status !== status) {
    statusEl.dataset.status = status;
    statusEl.textContent = STATUS_TEXT[status];
    refreshLocks();
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

  // ── 5. Tell Unreal what the arm is doing (30×/s) so it can mirror it ──
  const now = performance.now();
  if (unreal.connected && now - lastOscSend > 33) {
    lastOscSend = now;
    unreal.send('/arm/state', toDeg(state.q).map((v) => Math.round(v * 10) / 10));
    unreal.send('/arm/tip', simToUnreal(tip).map((v) => Math.round(v * 10) / 10));
    if (status !== lastOscStatus) {
      lastOscStatus = status;
      unreal.send('/arm/status', [status]);
    }
  }

  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
}

solveForTarget();
refreshRecorderUI();
refreshLocks();
refreshLinkDots();
requestAnimationFrame(tick);

// Handy for debugging in the browser console: try `sim.state` or `sim.ARM`.
window.sim = { state, recorder, safety, ARM, target, camera, solveForTarget };
