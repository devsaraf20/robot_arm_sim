/**
 * safety.js — the rules that stop the arm (virtual or real) from hurting itself or you.
 *
 * Four layers, from gentlest to hardest:
 *   1. JOINT LIMITS   – every target is clamped into each joint's legal range (ik.js LIMITS).
 *   2. MAX SPEED      – no joint may turn faster than `maxSpeed` degrees per second, however
 *                       fast the target moves. This is a "slew-rate limiter".
 *   3. HOMING         – a slow, predictable move back to the known HOME pose.
 *   4. EMERGENCY STOP – freezes everything immediately and stays latched until a HUMAN
 *                       presses Reset in the browser. Nothing remote (Unreal, OSC,
 *                       scripts) is allowed to clear it.
 *
 * The Arduino firmware enforces layers 1, 2 and 4 again on its own, so a browser bug
 * or a crashed tab can't make the real arm move dangerously. Belt and braces.
 *
 * Nothing in here touches the DOM or Three.js, so it is unit-tested in tests/.
 */
import { LIMITS, HOME, clamp } from './ik.js';

const DEG = Math.PI / 180;

export const SPEED_RANGE = { min: 10, max: 360, default: 90 }; // degrees / second
export const HOMING_SPEED = 30; // degrees / second — deliberately slow

/**
 * Move each joint of `q` towards `goal`, but never more than `maxStepRad` in one go.
 * Called once per frame with  maxStepRad = maxSpeed × dt.
 *
 * We first apply a soft "ease" (exponential smoothing, like a servo settling) and
 * THEN cap the step. The ease makes motion look natural, the cap makes it safe.
 */
export function stepTowards(q, goal, dt, maxSpeedDegPerSec, ease = 12) {
  const maxStep = maxSpeedDegPerSec * DEG * dt;
  const k = ease > 0 ? 1 - Math.exp(-dt * ease) : 1;
  return q.map((v, i) => {
    const wanted = (goal[i] - v) * k;
    const step = clamp(wanted, -maxStep, maxStep);
    return clamp(v + step, LIMITS[i][0], LIMITS[i][1]);
  });
}

/** Largest difference between two poses, in degrees. */
export const maxDiffDeg = (a, b) => Math.max(...a.map((v, i) => Math.abs(v - b[i]) / DEG));

/**
 * The safety state machine. States:
 *   'normal'  – anything may drive the arm
 *   'homing'  – driving back to HOME at HOMING_SPEED; user input is ignored until done
 *   'estop'   – frozen; only reset() leaves this state
 */
export function createSafety({ onChange = () => {} } = {}) {
  let state = 'normal';
  let maxSpeed = SPEED_RANGE.default;

  const set = (s, reason) => {
    if (state === s) return;
    state = s;
    onChange(s, reason);
  };

  return {
    get state() { return state; },
    get estopped() { return state === 'estop'; },
    get homing() { return state === 'homing'; },
    /** Can user/remote input move the arm right now? */
    get acceptsInput() { return state === 'normal'; },

    get maxSpeed() { return maxSpeed; },
    set maxSpeed(v) { maxSpeed = clamp(v, SPEED_RANGE.min, SPEED_RANGE.max); },

    /** Speed limit to use this frame. */
    get effectiveSpeed() { return state === 'homing' ? Math.min(HOMING_SPEED, maxSpeed) : maxSpeed; },

    estop(reason = 'button') { set('estop', reason); },
    reset() { if (state === 'estop') set('normal', 'reset'); },

    startHoming() {
      if (state === 'estop') return false; // must reset first
      set('homing', 'start');
      return true;
    },
    /** Call every frame while homing; finishes when the arm is within 0.5° of HOME. */
    updateHoming(q) {
      if (state === 'homing' && maxDiffDeg(q, HOME) < 0.5) set('normal', 'homed');
    },
  };
}
