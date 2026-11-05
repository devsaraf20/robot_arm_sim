/**
 * recorder.js — records joint angles over time and plays them back.
 *
 * A recording is just a list of KEYFRAMES, like an animation timeline:
 *    [{ t: 0.00, q: [base, shoulder, elbow, gripper] }, { t: 0.033, q: [...] }, ...]
 * t is seconds since recording started, q is joint angles in radians
 * (base, shoulder, elbow, gripper).
 *
 * Playback finds the two keyframes around the current time and blends between
 * them (linear interpolation — "lerp"), exactly like the default interpolation
 * between keys on Blender's timeline.
 */
import { HOME } from './ik.js';

const SAMPLE_INTERVAL = 1 / 30; // record at most 30 keyframes per second
const STORAGE_KEY = 'robot-arm-sim:recording';

export function createRecorder() {
  let frames = loadSaved();
  let state = 'idle'; // 'idle' | 'recording' | 'playing'
  let clock = 0;
  let loop = false;

  return {
    get state() { return state; },
    get frames() { return frames; },
    get duration() { return frames.length ? frames[frames.length - 1].t : 0; },
    get time() { return clock; },
    get loop() { return loop; },
    set loop(v) { loop = v; },

    startRecording() {
      frames = [];
      clock = 0;
      state = 'recording';
    },

    /** Call every animation frame while recording, with the current angles. */
    capture(dt, q) {
      if (state !== 'recording') return;
      clock += dt;
      const last = frames[frames.length - 1];
      if (!last || clock - last.t >= SAMPLE_INTERVAL) frames.push({ t: clock, q: q.slice() });
    },

    stop() {
      if (state === 'recording') {
        // Re-base time so the first keyframe is at t = 0.
        const t0 = frames.length ? frames[0].t : 0;
        frames = frames.map((f) => ({ t: f.t - t0, q: f.q }));
        save(frames);
      }
      state = 'idle';
    },

    play() {
      if (frames.length < 2) return false;
      clock = 0;
      state = 'playing';
      return true;
    },

    /** Advance playback by dt seconds. Returns the interpolated pose, or null when done. */
    advance(dt) {
      if (state !== 'playing') return null;
      clock += dt;
      if (clock > this.duration) {
        if (loop) clock %= this.duration || 1;
        else { state = 'idle'; return frames[frames.length - 1].q; }
      }
      return sample(frames, clock);
    },

    clear() {
      frames = [];
      state = 'idle';
      clock = 0;
      save(frames);
    },

    /** Resample to a fixed time step — what the Python/Arduino exporters need. */
    resample(stepSeconds) {
      if (frames.length < 2) return frames.map((f) => f.q);
      const out = [];
      for (let t = 0; t <= this.duration + 1e-9; t += stepSeconds) out.push(sample(frames, t));
      return out;
    },
  };
}

/** Find the pose at time t by blending the two neighbouring keyframes. */
export function sample(frames, t) {
  if (t <= frames[0].t) return frames[0].q.slice();
  // Binary search: jump to the right keyframe quickly even in long recordings.
  let lo = 0;
  let hi = frames.length - 1;
  if (t >= frames[hi].t) return frames[hi].q.slice();
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (frames[mid].t <= t) lo = mid; else hi = mid;
  }
  const a = frames[lo];
  const b = frames[hi];
  const k = (t - a.t) / (b.t - a.t || 1); // 0 → at a, 1 → at b
  return a.q.map((v, i) => v + (b.q[i] - v) * k);
}

// localStorage = a small per-browser save slot, so a recording survives a reload.
// Wrapped in try/catch because private browsing can block it.
function save(frames) {
  try { localStorage.setItem(STORAGE_KEY, JSON.stringify(frames)); } catch { /* ignore */ }
}
function loadSaved() {
  try {
    const data = JSON.parse(localStorage.getItem(STORAGE_KEY) || '[]');
    if (!Array.isArray(data)) return [];
    // Recordings from the older 3-joint version have no gripper: fill in the home value.
    return data.map((f) => ({ t: f.t, q: f.q.length < HOME.length ? [...f.q, ...HOME.slice(f.q.length)] : f.q }));
  } catch {
    return [];
  }
}
