/**
 * ik.js — the math of the robot arm. NO Three.js in here on purpose: this file is
 * pure numbers so it can be unit-tested in Node and copied into any other project.
 *
 * ──────────────────────────────────────────────────────────────────────────────
 *  THE ARM
 * ──────────────────────────────────────────────────────────────────────────────
 *
 *                    elbow ●───────────● tip (end effector)
 *                         /     L2
 *                    L1  /
 *                       /
 *          shoulder ●  ← sits BASE_HEIGHT above the floor
 *                   █
 *        ───────────█─────────── floor (y = 0)
 *                 base (spins around the vertical Y axis)
 *
 *  Three joints (all angles in RADIANS inside this file):
 *    q[0]  base yaw      – spin around the vertical (Y) axis. 0 = arm points along +X.
 *    q[1]  shoulder      – pitch of the upper arm, measured UP from horizontal.
 *                          0 = upper arm flat & forward, 90° = pointing straight up.
 *    q[2]  elbow         – bend of the forearm RELATIVE to the upper arm.
 *                          0 = arm straight, negative = forearm folds downward.
 *
 *  Coordinate system is Three.js's: Y is up, X right, Z towards the viewer.
 *  Rotating by +angle about Y turns +X towards −Z, which is why you will see
 *  "-z" in the yaw formulas below.
 */

const DEG = Math.PI / 180;

/** Physical dimensions (arbitrary units — think "decimetres"). */
export const ARM = {
  BASE_HEIGHT: 0.9, // floor → shoulder pivot
  L1: 1.6, // shoulder → elbow
  L2: 1.4, // elbow → tip
};

/**
 * Joint limits in DEGREES. The three arm joints each span exactly 180° so they map
 * 1:1 onto a standard hobby servo (0…180°). Everything that talks to hardware
 * (exporters, Web Serial link) uses the same rule:   servo = angle − min
 *
 * The 4th entry is the GRIPPER: not part of the IK chain (it only opens/closes the
 * fingers), but it is a 4th servo on a real desktop arm, so it is recorded, exported
 * and streamed like the others. 0° = closed, 90° = fully open.
 */
export const JOINTS = [
  { name: 'Base', key: 'base', min: -90, max: 90, home: 0 },
  { name: 'Shoulder', key: 'shoulder', min: 0, max: 180, home: 70 },
  { name: 'Elbow', key: 'elbow', min: -150, max: 30, home: -100 },
  { name: 'Gripper', key: 'gripper', min: 0, max: 90, home: 45 },
];

/** How many joints take part in the IK chain (base, shoulder, elbow). */
export const IK_JOINTS = 3;

export const LIMITS = JOINTS.map((j) => [j.min * DEG, j.max * DEG]);
export const HOME = JOINTS.map((j) => j.home * DEG);

export const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
export const clampToLimits = (q) => q.map((v, i) => clamp(v, LIMITS[i][0], LIMITS[i][1]));
const withinLimits = (q, eps = 1e-6) =>
  q.every((v, i) => v >= LIMITS[i][0] - eps && v <= LIMITS[i][1] + eps);

/** Wrap any angle into (−π, π] so "350°" and "−10°" are treated as the same pose. */
export const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));

/* ════════════════════════════════════════════════════════════════════════════
 *  FORWARD KINEMATICS  (angles → position)
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  Idea: walk along the arm, adding each bone as a little arrow.
 *
 *  Step 1 — work in the arm's own vertical plane (a 2D sheet of paper that the
 *  base spins around). Call the horizontal axis r ("reach") and the vertical s.
 *
 *     elbow = L1 · (cos q1, sin q1)                          ← upper-arm arrow
 *     tip   = elbow + L2 · (cos(q1+q2), sin(q1+q2))          ← forearm arrow
 *
 *  The forearm's absolute angle is q1 + q2 because the elbow angle is measured
 *  relative to the upper arm (exactly like a child bone in Blender).
 *
 *  Step 2 — lift that 2D sheet into 3D: raise it by BASE_HEIGHT, then spin it by
 *  the base yaw q0 around Y:   x = r·cos q0,   z = −r·sin q0,   y = s + BASE_HEIGHT
 */
export function forwardKinematics(q, arm = ARM) {
  const [q0, q1, q2] = q;
  const elbowR = arm.L1 * Math.cos(q1);
  const elbowS = arm.L1 * Math.sin(q1);
  const tipR = elbowR + arm.L2 * Math.cos(q1 + q2);
  const tipS = elbowS + arm.L2 * Math.sin(q1 + q2);
  const toWorld = (r, s) => ({
    x: r * Math.cos(q0),
    y: s + arm.BASE_HEIGHT,
    z: -r * Math.sin(q0),
  });
  return {
    shoulder: { x: 0, y: arm.BASE_HEIGHT, z: 0 },
    elbow: toWorld(elbowR, elbowS),
    tip: toWorld(tipR, tipS),
  };
}

const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);

/* ════════════════════════════════════════════════════════════════════════════
 *  INVERSE KINEMATICS  (position → angles)
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  We solve it ANALYTICALLY (closed-form geometry) — fast, exact, no iterations —
 *  and only fall back to a numerical solver when joint limits get in the way.
 *
 *  ── Step A: base yaw ───────────────────────────────────────────────────────
 *  Look down from above. The arm must face the target, so the yaw is just the
 *  compass direction of the target:   q0 = atan2(−z, x)
 *  (atan2 is the "angle of the vector (x, y)" function that handles all four
 *  quadrants correctly, unlike plain atan(y/x).)
 *
 *  There is a second option: face AWAY from the target (q0 + 180°) and reach
 *  backwards over the shoulder. Then the reach r becomes negative. We try both,
 *  because base limits (−90…90°) mean targets behind the robot can ONLY be
 *  reached the over-the-shoulder way.
 *
 *  ── Step B: the 2-link planar problem ──────────────────────────────────────
 *  Now we are back on the 2D sheet:  r = ±√(x² + z²),   s = y − BASE_HEIGHT.
 *  We know both bone lengths and the distance d from shoulder to target:
 *
 *        d² = r² + s²
 *
 *  The shoulder, elbow and target form a TRIANGLE with sides L1, L2, d.
 *  The Law of Cosines relates a triangle's sides to one of its angles:
 *
 *        d² = L1² + L2² + 2·L1·L2·cos(q2)
 *    ⇒   cos(q2) = (d² − L1² − L2²) / (2·L1·L2)
 *
 *  (It is "+2…" rather than the textbook "−2…" because q2 is the bend AWAY from
 *  straight, i.e. 180° minus the triangle's interior angle.)
 *
 *  • If |cos q2| > 1 the triangle cannot exist → target is out of reach. We clamp
 *    it to ±1, which gives the arm fully stretched (or fully folded) pointing at
 *    the target — the closest it can get.
 *  • acos gives ONE angle, but ±q2 both work: "elbow up" and "elbow down".
 *
 *  ── Step C: shoulder ───────────────────────────────────────────────────────
 *  The shoulder angle = (direction to the target) − (the offset the bent elbow
 *  adds). Both are atan2's:
 *
 *        q1 = atan2(s, r) − atan2(L2·sin q2,  L1 + L2·cos q2)
 *
 *  The second term is the angle of the tip as seen from the shoulder if the
 *  upper arm were lying flat — subtracting it "rotates the whole bent arm" so
 *  the tip lands on the target.
 *
 *  ── Step D: pick the best candidate ────────────────────────────────────────
 *  2 yaw options × 2 elbow options = up to 4 solutions. We keep the ones inside
 *  the joint limits and choose the one closest to the arm's CURRENT pose, so the
 *  arm doesn't suddenly flip when another solution becomes available.
 *
 *  ── Step E: fallback when limits block every exact solution ───────────────
 *  See `refineWithinLimits` below (damped least squares).
 *
 *  Returns { q: [base, shoulder, elbow], error, status }: only the IK joints. The
 *  caller keeps whatever gripper value it already had.
 */
export function solveIK(target, current = HOME, arm = ARM) {
  const { L1, L2, BASE_HEIGHT } = arm;
  const horiz = Math.hypot(target.x, target.z);
  const s = target.y - BASE_HEIGHT;
  // Facing straight up/down the yaw is undefined (atan2(0,0)): keep the current yaw.
  const yaw = horiz < 1e-6 ? current[0] : Math.atan2(-target.z, target.x);

  const candidates = [];
  for (const flip of [false, true]) {
    const q0 = wrapAngle(flip ? yaw + Math.PI : yaw);
    const r = flip ? -horiz : horiz;
    const cosQ2 = clamp((r * r + s * s - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1);
    const bend = Math.acos(cosQ2);
    for (const q2 of [-bend, bend]) {
      const q1 = Math.atan2(s, r) - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2));
      candidates.push([q0, wrapAngle(q1), q2]);
    }
  }

  // How "far" a pose is from the current one (sum of squared joint moves).
  const effort = (q) => q.reduce((sum, v, i) => sum + wrapAngle(v - current[i]) ** 2, 0); // first 3 joints only

  let best = null;
  for (const q of candidates) {
    if (!withinLimits(q)) continue;
    const err = dist(forwardKinematics(q, arm).tip, target);
    // Prefer accuracy first (1 mm tolerance), then the least joint motion.
    if (!best || err < best.err - 1e-3 || (Math.abs(err - best.err) <= 1e-3 && effort(q) < best.effort)) {
      best = { q: clampToLimits(q), err, effort: effort(q) };
    }
  }

  const reachable = Math.hypot(horiz, s) <= L1 + L2 + 1e-9 && Math.hypot(horiz, s) >= Math.abs(L1 - L2) - 1e-9;

  if (best && best.err < 1e-4) {
    return { q: best.q, error: best.err, status: 'ok' };
  }

  // No exact in-limit solution: start the numerical solver from every candidate
  // (clamped into limits) plus the current pose, and keep the best result.
  let refined = null;
  for (const seed of [...candidates, current.slice(0, IK_JOINTS)]) {
    const q = refineWithinLimits(clampToLimits(seed), target, arm);
    const err = dist(forwardKinematics(q, arm).tip, target);
    if (!refined || err < refined.err - 1e-4 || (Math.abs(err - refined.err) <= 1e-4 && effort(q) < effort(refined.q))) {
      refined = { q, err };
    }
  }
  return {
    q: refined.q,
    error: refined.err,
    status: !reachable ? 'out-of-reach' : refined.err > 1e-3 ? 'joint-limit' : 'ok',
  };
}

/* ════════════════════════════════════════════════════════════════════════════
 *  NUMERICAL FALLBACK — Damped Least Squares with joint clamping
 * ════════════════════════════════════════════════════════════════════════════
 *
 *  When the exact answer would break a joint limit, we instead ask: "which LEGAL
 *  pose puts the tip as close as possible to the target?"  We find it by taking
 *  small steps downhill:
 *
 *  1. The JACOBIAN J is a 3×3 table answering "if I nudge joint i a tiny bit,
 *     how does the tip move in x, y, z?". We measure it by literally nudging each
 *     joint by h and looking at the change (finite differences).
 *
 *  2. We want a joint change Δq such that J·Δq ≈ e  (e = target − tip).
 *     Damped least squares solves  (JᵀJ + λ²I)·Δq = Jᵀe.
 *     The λ² "damping" keeps steps sane near singularities (e.g. arm fully
 *     stretched) where plain inversion would explode.
 *
 *  3. After each step we clamp every joint back inside its limits. A joint that
 *     hits its stop is "frozen" and the others keep working to close the gap.
 */
export function refineWithinLimits(q, target, arm = ARM, iterations = 60) {
  q = q.slice();
  const h = 1e-4;
  const lambda2 = 0.01;
  for (let it = 0; it < iterations; it++) {
    const tip = forwardKinematics(q, arm).tip;
    const e = [target.x - tip.x, target.y - tip.y, target.z - tip.z];
    if (Math.hypot(...e) < 1e-6) break;

    // J[row = x/y/z][col = joint]
    const J = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let j = 0; j < 3; j++) {
      const qh = q.slice();
      qh[j] += h;
      const t2 = forwardKinematics(qh, arm).tip;
      J[0][j] = (t2.x - tip.x) / h;
      J[1][j] = (t2.y - tip.y) / h;
      J[2][j] = (t2.z - tip.z) / h;
    }

    // Frozen joints: at a limit AND the gradient wants to push further past it.
    const g = [0, 1, 2].map((j) => J[0][j] * e[0] + J[1][j] * e[1] + J[2][j] * e[2]); // Jᵀe
    const free = [0, 1, 2].map((j) => {
      const [lo, hi] = LIMITS[j];
      return !((q[j] <= lo + 1e-9 && g[j] < 0) || (q[j] >= hi - 1e-9 && g[j] > 0));
    });

    // A = JᵀJ + λ²I, restricted to free joints (frozen rows become identity, rhs 0).
    const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    const b = [0, 0, 0];
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        if (!free[r] || !free[c]) { A[r][c] = r === c ? 1 : 0; continue; }
        A[r][c] = J[0][r] * J[0][c] + J[1][r] * J[1][c] + J[2][r] * J[2][c] + (r === c ? lambda2 : 0);
      }
      b[r] = free[r] ? g[r] : 0;
    }
    const dq = solve3(A, b);
    if (!dq) break;

    // Safety: cap the step size, then only ACCEPT a step if it really brings the tip
    // closer (halving it a few times if not). This "backtracking" keeps the solver
    // from overshooting when the target is far away and the linear guess is poor.
    const maxStep = 0.25; // radians
    const big = Math.max(...dq.map(Math.abs));
    let scale = big > maxStep ? maxStep / big : 1;
    const errNow = Math.hypot(...e);
    let accepted = false;
    for (let tries = 0; tries < 6; tries++, scale *= 0.5) {
      const qNew = clampToLimits(q.map((v, i) => v + dq[i] * scale));
      if (dist(forwardKinematics(qNew, arm).tip, target) < errNow) {
        q = qNew;
        accepted = true;
        break;
      }
    }
    if (!accepted) break; // we are at the closest legal pose we can find
  }
  return q;
}

/** Solve a 3×3 linear system A·x = b with Cramer's rule (fine for 3×3). */
function solve3(A, b) {
  const det = (m) =>
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  const D = det(A);
  if (Math.abs(D) < 1e-12) return null;
  return [0, 1, 2].map((col) => det(A.map((row, r) => row.map((v, c) => (c === col ? b[r] : v)))) / D);
}

export const toDeg = (q) => q.map((v) => v / DEG);
export const toRad = (d) => d.map((v) => v * DEG);

/* ── Hardware angle conversion ──────────────────────────────────────────────
 *  The browser and the Arduino agree on "logical servo degrees" (0…180) for every
 *  joint: servo = simulatorAngle − jointMin. Hardware quirks (a servo mounted
 *  backwards, a few degrees of trim) are handled in the firmware, not here.     */
export const simToServo = (qRad) => qRad.map((v, i) => v / DEG - JOINTS[i].min);
export const servoToSim = (servoDeg) => servoDeg.map((v, i) => clamp((v + JOINTS[i].min) * DEG, LIMITS[i][0], LIMITS[i][1]));
