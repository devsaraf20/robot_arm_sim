/**
 * unreal-link.js — the browser side of the Unreal Engine connection.
 *
 * Unreal speaks OSC over UDP, which browsers cannot receive, so we connect to the
 * local bridge (bridge/osc-bridge.mjs) over a WebSocket. The bridge hands us each
 * OSC message as JSON: { "address": "/arm/target", "args": [x, y, z] }.
 *
 * Messages we understand (from Unreal):
 *   /arm/target  x y z            gripper target in UNREAL space: cm, relative to the
 *                                 arm's base, X forward, Y right, Z up
 *   /arm/joints  base shoulder elbow [gripper]   simulator angles in degrees
 *   /arm/gripper open             0 = closed … 1 = fully open
 *   /arm/home                     run the homing routine
 *   /arm/estop                    EMERGENCY STOP (Unreal can trigger it, never clear it)
 *
 * Messages we send back (~30×/s):
 *   /arm/state   base shoulder elbow gripper   (degrees, what the arm is doing now)
 *   /arm/tip     x y z                         (cm, Unreal space, relative to the base)
 *   /arm/status  "ok" | "joint-limit" | "out-of-reach" | "estop" | "homing" | ...
 */

/* Coordinate conversion — the classic gotcha when moving between engines.
 *
 *   Unreal:   X forward, Y right, Z up, centimetres, LEFT-handed
 *   Three.js: X right,   Y up,    Z towards you, RIGHT-handed. Our arm faces +X.
 *
 * Standing behind the arm looking along +X with +Y up, Three's +Z points to your
 * RIGHT. So:  sim.x = ue.x,  sim.y = ue.z,  sim.z = ue.y.  Swapping two axes is
 * exactly what converts left-handed to right-handed coordinates.
 * One simulator unit is ~10 cm (see ik.js), hence the 0.1 scale.            */
export const UNITS_PER_CM = 0.1;
export const unrealToSim = ([x, y, z], scale = UNITS_PER_CM) => ({ x: x * scale, y: z * scale, z: y * scale });
export const simToUnreal = ({ x, y, z }, scale = UNITS_PER_CM) => [x / scale, z / scale, y / scale];

export function createUnrealLink({ onMessage = () => {}, onStatus = () => {} } = {}) {
  let ws = null;
  let url = '';
  let wanted = false; // keep reconnecting while the user wants a connection
  let retryTimer = null;
  let lastMessageAt = 0;

  function open() {
    clearTimeout(retryTimer);
    onStatus('connecting');
    try {
      ws = new WebSocket(url);
    } catch (err) {
      onStatus('error', err.message);
      return;
    }
    ws.onopen = () => onStatus('connected');
    ws.onmessage = (e) => {
      let msg;
      try { msg = JSON.parse(e.data); } catch { return; }
      if (typeof msg?.address !== 'string' || !Array.isArray(msg.args)) return;
      lastMessageAt = performance.now();
      onMessage(msg);
    };
    ws.onclose = () => {
      ws = null;
      if (wanted) {
        onStatus('retrying');
        retryTimer = setTimeout(open, 2000); // bridge not running yet? try again
      } else onStatus('disconnected');
    };
  }

  return {
    connect(newUrl) {
      url = newUrl;
      wanted = true;
      open();
    },
    disconnect() {
      wanted = false;
      clearTimeout(retryTimer);
      ws?.close();
      onStatus('disconnected');
    },
    send(address, args) {
      if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ address, args }));
    },
    get connected() { return ws?.readyState === WebSocket.OPEN; },
    /** True if Unreal sent something in the last `ms` milliseconds. */
    active(ms = 500) { return performance.now() - lastMessageAt < ms; },
  };
}
