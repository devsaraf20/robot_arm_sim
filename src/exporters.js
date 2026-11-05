/**
 * exporters.js — turns a recording into code for a REAL robot arm.
 *
 * Both exporters receive the same input:
 *   frames  – joint angles in DEGREES, one row per time step:
 *             [[base, shoulder, elbow, gripper], ...]
 *   stepMs  – time between rows in milliseconds
 *
 * Simulator angles are converted to servo angles with:
 *     servo = (angle − jointMin)            → 0…180 for the arm joints, 0…90 for the gripper
 *     servo = 180 − servo   if INVERT        → for servos mounted "the other way round"
 *     servo = servo + TRIM                   → small calibration offset per servo
 * The generated code does this conversion itself (so you can edit INVERT/TRIM
 * there) and the raw simulator angles stay readable in the file.
 *
 * Pin numbers match the digital-twin firmware and docs/HARDWARE.md, so the same
 * wiring works for both.
 *
 * We build the code as text using JavaScript template strings (`...${value}...`).
 */
import { JOINTS } from './ik.js';

const r1 = (v) => (Math.round(v * 10) / 10).toFixed(1);
const N = JOINTS.length;
const names = JOINTS.map((j) => j.key);

export function toPython(frames, stepMs) {
  const rows = frames.map((q) => `    (${q.map(r1).join(', ')}),`).join('\n');
  return `#!/usr/bin/env python3
"""
Robot arm motion exported from the Robot Arm Simulator.

  ${frames.length} frames, ${stepMs} ms apart  (${((frames.length * stepMs) / 1000).toFixed(2)} s total)

Usage:
  python3 robot_arm_motion.py                          # dry run: print servo angles
  python3 robot_arm_motion.py --plot                   # plot the motion (needs matplotlib)
  python3 robot_arm_motion.py --backend servokit       # Raspberry Pi + PCA9685 servo HAT
  python3 robot_arm_motion.py --backend serial --port /dev/ttyUSB0
        # streams to an Arduino running firmware/robot_arm_twin (the digital-twin
        # firmware). The firmware applies its own joint limits and speed limit.

Joint conventions (simulator, degrees):
${JOINTS.map((j) => `  ${j.key.padEnd(9)} ${String(j.min).padStart(4)}..${String(j.max).padEnd(4)} servo = angle ${j.min < 0 ? '+ ' + -j.min : '- ' + j.min}`).join('\n')}
"""
import argparse
import time

FRAME_DT = ${(stepMs / 1000).toFixed(3)}  # seconds between frames

# Simulator angles in degrees: (${names.join(', ')})
FRAMES = [
${rows}
]

# ---- Calibration: edit these to match YOUR arm --------------------------------
JOINT_MIN = (${JOINTS.map((j) => j.min).join(', ')})  # simulator angle that maps to servo 0
INVERT = (${Array(N).fill('False').join(', ')})   # True if a servo turns the opposite way
TRIM = (${Array(N).fill('0.0').join(', ')})           # degrees added after mapping (fine-tuning)
SERVO_CHANNELS = (${[...Array(N).keys()].join(', ')})              # PCA9685 channels: ${names.join(', ')}


def to_servo(frame, calibrate=True):
    """Convert simulator angles to servo angles (0..180)."""
    out = []
    for angle, lo, inv, trim in zip(frame, JOINT_MIN, INVERT, TRIM):
        s = angle - lo
        if calibrate:
            if inv:
                s = 180.0 - s
            s += trim
        out.append(max(0.0, min(180.0, s)))
    return out


def run(send, speed=1.0, loop=False):
    while True:
        start = time.monotonic()
        for i, frame in enumerate(FRAMES):
            send(frame)
            # Sleep until this frame's scheduled time (keeps timing accurate).
            wait = start + (i + 1) * FRAME_DT / speed - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        if not loop:
            break


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", choices=["dry-run", "servokit", "serial"], default="dry-run")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="serial port for --backend serial")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    ap.add_argument("--max-speed", type=int, default=90, help="firmware speed limit, deg/s (serial)")
    ap.add_argument("--loop", action="store_true", help="repeat forever (Ctrl+C to stop)")
    ap.add_argument("--plot", action="store_true", help="plot joint angles and exit")
    args = ap.parse_args()

    if args.plot:
        import matplotlib.pyplot as plt
        t = [i * FRAME_DT for i in range(len(FRAMES))]
        for j, name in enumerate((${names.map((n) => `"${n}"`).join(', ')})):
            plt.plot(t, [f[j] for f in FRAMES], label=name)
        plt.xlabel("time (s)")
        plt.ylabel("angle (deg)")
        plt.legend()
        plt.show()
        return

    if args.backend == "servokit":
        from adafruit_servokit import ServoKit  # pip install adafruit-circuitpython-servokit
        kit = ServoKit(channels=16)

        def send(frame):
            for ch, a in zip(SERVO_CHANNELS, to_servo(frame)):
                kit.servo[ch].angle = a
    elif args.backend == "serial":
        import serial  # pip install pyserial
        port = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2)  # most Arduinos reset when the port opens
        # The twin firmware does INVERT/TRIM itself, so send uncalibrated angles.
        port.write(b"W\\n")
        port.write(("V %d\\n" % args.max_speed).encode())

        def send(frame):
            port.write(("T " + " ".join("%.1f" % a for a in to_servo(frame, calibrate=False)) + "\\n").encode())
    else:
        def send(frame):
            print("  ".join("%s %6.1f" % (n, a) for n, a in zip((${names.map((n) => `"${n}"`).join(', ')}), to_servo(frame))))

    try:
        run(send, speed=args.speed, loop=args.loop)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
`;
}

export function toArduino(frames, stepMs) {
  // Store angles ×10 as integers (0.1° resolution) in flash memory (PROGMEM),
  // because an Arduino Uno only has 2 KB of RAM but 32 KB of flash.
  const rows = frames
    .map((q) => `  {${q.map((v) => String(Math.round(v * 10)).padStart(5)).join(',')}},`)
    .join('\n');
  return `/*
 * Robot arm motion exported from the Robot Arm Simulator.
 * ${frames.length} frames, ${stepMs} ms apart (${((frames.length * stepMs) / 1000).toFixed(2)} s).
 * Standalone playback: no computer needed once uploaded.
 * (For LIVE control from the browser, upload firmware/robot_arm_twin instead.)
 *
 * Wiring (same as docs/HARDWARE.md): base -> D9, shoulder -> D10, elbow -> D11,
 * gripper -> D6. Power the servos from a separate 5-6 V supply, NOT the
 * Arduino's 5 V pin, and connect the supply's GND to the Arduino's GND.
 * Optional stop button between D2 and GND: press to freeze, press again to resume.
 */
#include <Servo.h>
#include <avr/pgmspace.h>

#define LOOP_PLAYBACK 1
#define JOINTS ${N}

const uint16_t FRAME_MS = ${stepMs};
const uint8_t SERVO_PINS[JOINTS] = {9, 10, 11, 6};   // ${names.join(', ')}
const uint8_t STOP_PIN = 2;

// ---- Calibration: edit these to match YOUR arm ----
const int   JOINT_MIN[JOINTS] = {${JOINTS.map((j) => j.min).join(', ')}};  // sim angle -> servo 0
const bool  INVERT[JOINTS]    = {${Array(N).fill('false').join(', ')}};
const float TRIM[JOINTS]      = {${Array(N).fill('0').join(', ')}};                 // degrees
const int   PULSE_MIN_US = 500;                        // pulse width at 0 deg
const int   PULSE_MAX_US = 2500;                       // pulse width at 180 deg

// Simulator angles x10 (e.g. -1000 = -100.0 deg): {${names.join(', ')}}
const int16_t FRAMES[][JOINTS] PROGMEM = {
${rows}
};
const uint16_t FRAME_COUNT = sizeof(FRAMES) / sizeof(FRAMES[0]);

Servo servos[JOINTS];

// Write a SERVO angle (0..180, fractional) using microseconds for smooth motion.
void writeServo(uint8_t i, float servoDeg) {
  servoDeg = constrain(servoDeg + TRIM[i], 0, 180);
  int us = PULSE_MIN_US + (long)(servoDeg * 100) * (PULSE_MAX_US - PULSE_MIN_US) / 18000L;
  servos[i].writeMicroseconds(us);
}

// Convert a SIMULATOR angle to a servo angle and write it.
void writeJoint(uint8_t i, float simDeg) {
  float s = simDeg - JOINT_MIN[i];
  if (INVERT[i]) s = 180 - s;
  writeServo(i, s);
}

void playFrame(uint16_t f) {
  for (uint8_t i = 0; i < JOINTS; i++) {
    int16_t raw = pgm_read_word(&FRAMES[f][i]);
    writeJoint(i, raw / 10.0);
  }
}

void setup() {
  pinMode(STOP_PIN, INPUT_PULLUP);
  playFrame(0);  // set the start pose BEFORE attaching, so the first pulse is correct
  for (uint8_t i = 0; i < JOINTS; i++) servos[i].attach(SERVO_PINS[i]);
  delay(1000);   // give the arm time to reach the start pose
}

void loop() {
  static uint16_t frame = 0;
  static unsigned long next = millis();
  static bool paused = false;

  // Stop button: toggles pause (debounced).
  static bool lastBtn = HIGH;
  bool btn = digitalRead(STOP_PIN);
  if (btn == LOW && lastBtn == HIGH) { paused = !paused; delay(30); next = millis(); }
  lastBtn = btn;
  if (paused) return;

  if ((long)(millis() - next) >= 0) {
    playFrame(frame);
    next += FRAME_MS;
    if (++frame >= FRAME_COUNT) {
      if (!LOOP_PLAYBACK) { while (true) {} }
      frame = 0;
      delay(1000);
      next = millis();
    }
  }
}
`;
}
