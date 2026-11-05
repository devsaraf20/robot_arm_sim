/**
 * protocol.js — the little "language" the browser and the Arduino speak over USB.
 *
 * It is plain text, one command per line, so you can also type it by hand in the
 * Arduino IDE's Serial Monitor (115200 baud, "Newline" line ending) to debug.
 * All angles are LOGICAL SERVO DEGREES 0…180 (see simToServo in ik.js), in the
 * order base, shoulder, elbow, gripper.
 *
 *  Browser → Arduino                       Arduino → Browser
 *  ─────────────────                       ─────────────────
 *  ?              hello / status request   HELLO robot-arm-twin <version> <joints> <feedback 0|1>
 *  W              WRITE mode (sim → arm)   P a b c d    position report (~25×/s)
 *  R              READ mode  (arm → sim)   S <state>    READY | MODE W | MODE R | HOMING |
 *  T a b c d      move to these angles                  HOMED | ESTOP | CAL ...
 *  V deg/s        set max speed            # text       log message (shown in the console)
 *  H              run homing routine       ! text       error message
 *  E              EMERGENCY STOP
 *  C              clear e-stop (re-arm)
 *  K              auto-calibrate feedback
 *  L j angle      learn a calibration point for joint j (potentiometer arms)
 */

export const BAUD_RATE = 115200;

const fmt = (v) => (Math.round(v * 10) / 10).toFixed(1);

export const cmd = {
  hello: () => '?',
  write: () => 'W',
  read: () => 'R',
  target: (servoDeg) => `T ${servoDeg.map(fmt).join(' ')}`,
  speed: (degPerSec) => `V ${Math.round(degPerSec)}`,
  home: () => 'H',
  estop: () => 'E',
  clear: () => 'C',
  calibrate: () => 'K',
  learn: (joint, angle) => `L ${joint} ${fmt(angle)}`,
};

/**
 * Turn one line from the Arduino into an object. Unknown lines become { type: 'log' }
 * so that a misbehaving board can never crash the app.
 */
export function parseLine(raw) {
  const line = raw.trim();
  if (!line) return null;
  const [head, ...rest] = line.split(/\s+/);
  switch (head) {
    case 'P': {
      const angles = rest.map(Number);
      if (angles.length < 3 || angles.some((v) => !Number.isFinite(v))) return { type: 'log', text: line };
      return { type: 'position', angles };
    }
    case 'S':
      return { type: 'state', state: rest.join(' ') };
    case 'HELLO':
      return {
        type: 'hello',
        firmware: rest[0] ?? 'unknown',
        version: rest[1] ?? '?',
        joints: Number(rest[2] ?? 0),
        feedback: rest[3] === '1',
      };
    case '!':
      return { type: 'error', text: rest.join(' ') };
    case '#':
      return { type: 'log', text: rest.join(' ') };
    default:
      return { type: 'log', text: line };
  }
}

/**
 * Serial data arrives in arbitrary chunks ("P 90.0 4" … "5.0 30.0\nP 9"…).
 * This collects chunks and hands back only complete lines.
 */
export function createLineSplitter() {
  let buffer = '';
  return (chunk) => {
    buffer += chunk;
    const lines = buffer.split(/\r?\n/);
    buffer = lines.pop(); // the last piece has no newline yet — keep it for next time
    if (buffer.length > 1000) buffer = ''; // garbage protection
    return lines;
  };
}
