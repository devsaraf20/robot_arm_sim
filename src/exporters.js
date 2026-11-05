/**
 * exporters.js — turns a recording into code for a REAL robot arm.
 *
 * Both exporters receive the same input:
 *   frames  – joint angles in DEGREES, one row per time step, e.g. [[0, 70, -100], ...]
 *   stepMs  – time between rows in milliseconds
 *
 * Simulator angles are converted to servo angles with:
 *     servo = (angle − jointMin)            → always 0…180 because every joint range is 180° wide
 *     servo = 180 − servo   if INVERT        → for servos mounted "the other way round"
 *     servo = servo + TRIM                   → small calibration offset per servo
 * The generated code does this conversion itself (so you can edit INVERT/TRIM
 * there) and the raw simulator angles stay readable in the file.
 *
 * We build the code as text using JavaScript template strings (`...${value}...`).
 */
import { JOINTS } from './ik.js';

const r1 = (v) => Math.round(v * 10) / 10; // one decimal place

export function toPython(frames, stepMs) {
  const rows = frames.map((q) => `    (${q.map((v) => r1(v).toFixed(1)).join(', ')}),`).join('\n');
  return `#!/usr/bin/env python3
"""
Robot arm motion exported from the 3-Joint Robot Arm Simulator.

  ${frames.length} frames, ${stepMs} ms apart  (${((frames.length * stepMs) / 1000).toFixed(2)} s total)

Usage:
  python3 robot_arm_motion.py                          # dry run: print angles
  python3 robot_arm_motion.py --plot                   # plot the motion (needs matplotlib)
  python3 robot_arm_motion.py --backend servokit       # Raspberry Pi + PCA9685 servo HAT
  python3 robot_arm_motion.py --backend serial --port /dev/ttyUSB0
        # streams "base shoulder elbow" lines to the exported Arduino sketch
        # (set STREAM_MODE 1 in the sketch)

Joint conventions (simulator, degrees):
  base      ${JOINTS[0].min}..${JOINTS[0].max}   0 = facing forward, + = turn left (counter-clockwise from above)
  shoulder  ${JOINTS[1].min}..${JOINTS[1].max}   0 = upper arm horizontal forward, 90 = straight up
  elbow     ${JOINTS[2].min}..${JOINTS[2].max}  0 = straight, negative = folds downward
"""
import argparse
import time

FRAME_DT = ${(stepMs / 1000).toFixed(3)}  # seconds between frames

# Simulator angles in degrees: (base, shoulder, elbow)
FRAMES = [
${rows}
]

# ---- Calibration: edit these to match YOUR arm --------------------------------
JOINT_MIN = (${JOINTS.map((j) => j.min).join(', ')})  # simulator angle that maps to servo 0
INVERT = (False, False, False)       # True if a servo turns the opposite way
TRIM = (0.0, 0.0, 0.0)               # degrees added after mapping (fine-tuning)
SERVO_CHANNELS = (0, 1, 2)           # PCA9685 channels for base, shoulder, elbow


def to_servo(frame):
    """Convert simulator angles to servo angles (0..180)."""
    out = []
    for angle, lo, inv, trim in zip(frame, JOINT_MIN, INVERT, TRIM):
        s = angle - lo
        if inv:
            s = 180.0 - s
        out.append(max(0.0, min(180.0, s + trim)))
    return out


def run(send, speed=1.0, loop=False):
    while True:
        start = time.monotonic()
        for i, frame in enumerate(FRAMES):
            send(to_servo(frame))
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
    ap.add_argument("--loop", action="store_true", help="repeat forever (Ctrl+C to stop)")
    ap.add_argument("--plot", action="store_true", help="plot joint angles and exit")
    args = ap.parse_args()

    if args.plot:
        import matplotlib.pyplot as plt
        t = [i * FRAME_DT for i in range(len(FRAMES))]
        for j, name in enumerate(("base", "shoulder", "elbow")):
            plt.plot(t, [f[j] for f in FRAMES], label=name)
        plt.xlabel("time (s)")
        plt.ylabel("angle (deg)")
        plt.legend()
        plt.show()
        return

    if args.backend == "servokit":
        from adafruit_servokit import ServoKit  # pip install adafruit-circuitpython-servokit
        kit = ServoKit(channels=16)

        def send(s):
            for ch, a in zip(SERVO_CHANNELS, s):
                kit.servo[ch].angle = a
    elif args.backend == "serial":
        import serial  # pip install pyserial
        port = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2)  # most Arduinos reset when the port opens

        def send(s):
            port.write(("%.1f %.1f %.1f\\n" % tuple(s)).encode())
    else:
        def send(s):
            print("base %6.1f  shoulder %6.1f  elbow %6.1f" % tuple(s))

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
 * Robot arm motion exported from the 3-Joint Robot Arm Simulator.
 * ${frames.length} frames, ${stepMs} ms apart (${((frames.length * stepMs) / 1000).toFixed(2)} s).
 *
 * Wiring (default): base servo -> pin 9, shoulder -> pin 10, elbow -> pin 11.
 * Power the servos from a separate 5-6 V supply, NOT the Arduino's 5 V pin,
 * and connect the supply's GND to the Arduino's GND.
 *
 * STREAM_MODE 0: plays the recorded motion below (on a loop).
 * STREAM_MODE 1: instead listens on Serial for lines "base shoulder elbow"
 *                (servo degrees) - use with the exported Python script:
 *                python3 robot_arm_motion.py --backend serial --port <PORT>
 */
#include <Servo.h>
#include <avr/pgmspace.h>

#define STREAM_MODE 0
#define LOOP_PLAYBACK 1

const uint16_t FRAME_MS = ${stepMs};
const uint8_t SERVO_PINS[3] = {9, 10, 11};      // base, shoulder, elbow

// ---- Calibration: edit these to match YOUR arm ----
const int  JOINT_MIN[3] = {${JOINTS.map((j) => j.min).join(', ')}};  // sim angle -> servo 0
const bool INVERT[3]    = {false, false, false};
const float TRIM[3]     = {0, 0, 0};                 // degrees
const int  PULSE_MIN_US = 500;                        // pulse width at 0 deg
const int  PULSE_MAX_US = 2500;                       // pulse width at 180 deg

// Simulator angles x10 (e.g. -1000 = -100.0 deg): {base, shoulder, elbow}
const int16_t FRAMES[][3] PROGMEM = {
${rows}
};
const uint16_t FRAME_COUNT = sizeof(FRAMES) / sizeof(FRAMES[0]);

Servo servos[3];

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
  for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = pgm_read_word(&FRAMES[f][i]);
    writeJoint(i, raw / 10.0);
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) servos[i].attach(SERVO_PINS[i]);
  playFrame(0);
  delay(1000);  // give the arm time to reach the start pose
}

void loop() {
#if STREAM_MODE
  // Expect lines like "90.0 70.0 50.0\\n" (servo degrees).
  if (Serial.available()) {
    float a = Serial.parseFloat();
    float b = Serial.parseFloat();
    float c = Serial.parseFloat();
    if (Serial.read() == '\\n') {
      writeServo(0, a);
      writeServo(1, b);
      writeServo(2, c);
    }
  }
#else
  static uint16_t frame = 0;
  static unsigned long next = millis();
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
#endif
}
`;
}
