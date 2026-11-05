# Building the real arm: parts, wiring, setup

This guide builds a cheap **4-servo desktop arm** (base, shoulder, elbow, gripper) that works with the simulator as a live digital twin:

- **Sim → Arm**: drag the target in the browser and the real arm follows.
- **Arm → Sim**: move the real arm by hand and the 3D model follows. This needs servos with position feedback (see section 2).

Budget: roughly **US$40–60** for the basic version (prices are rough guides and vary by seller).

---

## 1. Parts list

| # | Part | Qty | ~Price | Notes |
|---|---|---|---|---|
| 1 | **4-DOF desktop arm frame** for micro servos (acrylic kit or 3D-printed) | 1 | $10–25 | Prefer a **direct-drive** design, with a servo at each joint like the simulator. MeArm-style *parallel-linkage* arms also work; see section 6. |
| 2 | **MG90S metal-gear micro servo** | 4 | $3–4 each | Metal gears survive mistakes; SG90s (plastic) strip easily. For a larger aluminium frame use MG996R and a 6 V / 5 A supply. |
| 3 | **Arduino Nano** (or Uno) | 1 | $5–25 | Clones are fine. Needs a data-capable USB cable. |
| 4 | **5 V 3 A DC power supply** (barrel plug) | 1 | $8 | Powers the servos. 4 micro servos can pull 2–3 A when stalled. |
| 5 | DC barrel-jack → screw-terminal adapter | 1 | $1 | |
| 6 | **Electrolytic capacitor 470–1000 µF, ≥10 V** | 1 | $0.50 | Across the servo supply; stops brown-outs that reset the Arduino. |
| 7 | **Latching emergency-stop switch** (mushroom, NC, rated ≥ 3 A DC) | 1 | $5–10 | Cuts servo power. This is the *real* e-stop. |
| 8 | Momentary push button (or arcade button) | 1 | $1 | Software stop on D2. Mount it where your hand naturally rests. |
| 9 | Half-size breadboard **or** Nano screw-terminal / sensor shield | 1 | $3–5 | A sensor shield with 3-pin servo headers makes wiring tidy. |
| 10 | Jumper wires (M-M and M-F), servo extension leads | — | $3 | |
| 11 | **Optional: position feedback**, choose one: | | | Needed only for **Arm → Sim** (read) mode. |
| 11a | Analog-feedback micro servos (servos with a 4th "feedback" wire) | 4 | ~$12 each | Easiest. Replace item 2. |
| 11b | DIY: open each MG90S and solder a thin wire to the potentiometer's middle (wiper) pin | — | free | Fiddly but cheap. Works the same as 11a. |
| 11c | **Leader arm**: a second, hand-held arm made of 10 kΩ linear potentiometers | 4 pots | $4 | Puppeteer the real arm or the sim. Calibrate with `L` (section 5). |

Tools: small Phillips screwdriver, wire strippers, multimeter (recommended), soldering iron for 11b.

---

## 2. Wiring

```
                        5 V 3 A SUPPLY
                     (+)              (−)
                      │                │
            ┌─────────┴──────┐         │
            │  E-STOP switch │         │         ← latching, normally-closed:
            │  (cuts power)  │         │           press = servos lose power
            └─────────┬──────┘         │
                      │                │
          servo +5V rail ──────┬───────┼──── servo GND rail
                               │  ┌────┴───┐
                            (+)│  │1000 µF │(−)   ← capacitor across the rails,
                               └──┤        ├─┘     mind the polarity stripe (−)
                                  └────────┘
                                       │
   ARDUINO NANO / UNO                  │
   ┌───────────────────┐               │
   │ GND ──────────────┼───────────────┘   ← COMMON GROUND (essential!)
   │ D9  ──────────────┼── Base servo     signal (orange/yellow wire)
   │ D10 ──────────────┼── Shoulder servo signal
   │ D11 ──────────────┼── Elbow servo    signal
   │ D6  ──────────────┼── Gripper servo  signal
   │ D2  ──────────────┼──[ stop button ]── GND
   │ A0..A3 ───────────┼── feedback wires: base, shoulder, elbow, gripper (optional)
   │ USB ──────────────┼── computer (Web Serial)
   └───────────────────┘

   Each servo: brown/black = GND rail, red = +5V rail, orange/yellow = signal pin.
```

| Arduino pin | Connects to |
|---|---|
| D9 | Base servo signal |
| D10 | Shoulder servo signal |
| D11 | Elbow servo signal |
| D6 | Gripper servo signal |
| D2 | Stop button → GND (internal pull-up, no resistor needed) |
| A0 / A1 / A2 / A3 | Feedback: base / shoulder / elbow / gripper (optional) |
| GND | Servo supply GND (**must** be connected) |
| 5V | **Nothing.** Don't power servos from the Arduino. |

**Rules that prevent 90% of problems**

1. **Never** power the servos from the Arduino's 5V pin. Servos draw spikes that reset the board, and can damage it or your USB port.
2. **Always** connect the servo supply's GND to the Arduino's GND. Without a shared ground the signal has no reference and servos twitch randomly.
3. Put the capacitor across the servo supply, close to the servos.
4. The **e-stop switch** goes in the servo **+5 V** line. The Arduino stays powered over USB, so the browser still sees what happened.

---

## 3. Firmware

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Open `firmware/robot_arm_twin/robot_arm_twin.ino`.
3. Edit the **CONFIGURATION** block at the top:

   | Setting | Set to |
   |---|---|
   | `HAS_FEEDBACK` | `1` if you wired feedback (11a/b/c), else `0` |
   | `PARALLEL_LINKAGE` | `1` for MeArm-style arms (section 6), else `0` |
   | `SAFE_MIN` / `SAFE_MAX` | Start wide, then tighten after testing (section 4, step 5) |
   | `INVERT` | `true` for any servo that turns the wrong way |
   | `TRIM` | A few degrees to fine-tune alignment |
   | `PULSE_MIN_US` / `PULSE_MAX_US` | 500/2500 suits MG90S. If a servo buzzes at the ends, try 1000/2000. |

4. Select your board (Tools → Board → Arduino Nano; for clones also try Tools → Processor → *ATmega328P (Old Bootloader)*) and the port, then **Upload**.
5. Open the Serial Monitor at **115200 baud** with **Newline** line ending. You should see:

   ```
   HELLO robot-arm-twin 1.0 4 0
   S READY
   S HOMING
   S HOMED
   P 90.0 70.0 50.0 45.0
   ...
   ```

The firmware enforces its own limits, a speed cap (`FW_MAX_SPEED`, 180°/s), the e-stop latch, and a **watchdog**: if the browser goes quiet for 1 s, the arm holds still.

---

## 4. Assembly and centring (do this before fixing the servo horns)

Servo horns sit on splines, so the "zero" position is set by how you press the horn on. Do it with the servo powered and parked at a known angle:

1. Wire everything **with the horns off**, power up, and open the Serial Monitor.
2. Send:

   ```
   V 60
   T 90 90 90 45
   ```

   All arm servos go to 90° and the gripper to 45°.
3. Now press the horns on so the arm is in this pose:
   - **Base**: arm facing straight forward.
   - **Shoulder**: upper arm pointing **straight up**.
   - **Elbow**: forearm pointing forward, **30° above horizontal** (bent 60° from the upper arm's line).
   - **Gripper**: half open.
4. Screw the horns down. In the browser, connect (section 5) and move each slider gently:
   - A joint goes the wrong way: set `INVERT` for it and re-upload.
   - It's a few degrees off: adjust `TRIM`.
5. Move each joint slowly towards its extremes and note where the arm would hit the table, its own frame, or a cable. Set `SAFE_MIN` / `SAFE_MAX` (logical degrees, 0–180 as shown next to each slider as "servo …°") a few degrees inside those points, and re-upload. **This is your most important safety setting.**
6. Measure your arm and update `ARM` in `src/ik.js` (1 unit = 10 cm): `BASE_HEIGHT` = table to shoulder axis, `L1` = shoulder axis to elbow axis, `L2` = elbow axis to the middle of the gripper. Then the simulated and real tips line up.

**Park the arm near the home pose before switching off.** On power-up the firmware moves slowly to HOME. Without feedback it can't know where the arm is, so a parked arm avoids a sudden first jump. With feedback it starts from wherever the arm actually is.

---

## 5. Connecting the digital twin

1. Use **Chrome or Edge on a desktop computer**. Safari and Firefox don't support Web Serial. Open the simulator (`npm run dev` → `http://localhost:5173`, or your Vercel https URL).
2. Close the Arduino IDE's Serial Monitor: only one program can use the port.
3. Panel → **Digital twin · real arm** → **Connect via USB** → pick the Arduino.
4. The status shows the firmware version and whether feedback is available. The **ARM** badge turns green.
5. **Sim → Arm**: anything that moves the 3D arm (dragging, sliders, playback, Unreal) is streamed to the real arm at up to 30 Hz. The see-through blue **ghost** shows where the real arm *reports* it is, so you can spot lag, stalls or a joint hitting a limit.
6. **Arm → Sim** (feedback required): the servos go limp. Move the real arm by hand and the 3D model follows. Press **Record** to capture the motion, switch back to **Sim → Arm**, and press **Play**. That's teaching by demonstration.

**Calibrating feedback**

- **Feedback servos / DIY wire (11a, 11b)**: click **Calibrate** in the panel, or send `K` in the Serial Monitor. Each joint moves slowly between two points inside its safe range and the firmware records the sensor readings, which are saved in EEPROM. Clear the area first.
- **Leader arm with pots (11c)**: for each joint `j` (0 = base … 3 = gripper), hold it at a known angle and send `L j angle` in the Serial Monitor, once with an angle below 90 and once above. Example: `L 1 20` with the shoulder near horizontal, then `L 1 160`.

**Safety controls**

| Control | Effect |
|---|---|
| Big red **STOP** button, or **Esc** | Browser freezes, firmware stops sending pulses (servos go limp). Latched until **Reset**. |
| Stop button on D2 | Same, from the hardware side. The browser e-stops too. |
| E-stop switch in the power line | Cuts servo power completely. Use it when anything looks wrong. |
| **Max joint speed** slider | Limits the sim and the real arm (the firmware also caps it). |
| **Run homing** | Slowly returns to HOME (30°/s). |
| Reset | Only from the browser, by a person. Unreal/OSC can trigger a stop but never clear one. |

> Limp servos drop under gravity. That's fine for a light micro-servo arm (it just slumps), and it's why software e-stop releases the motors rather than holding them. For heavier arms, add mechanical support or counterweights.

---

## 6. Parallel-linkage (MeArm-style) arms

In MeArm-style arms, both arm servos sit at the base and the elbow servo moves the forearm through a rod. Its angle then sets the forearm's angle **relative to the ground**, not relative to the upper arm. Set `PARALLEL_LINKAGE 1` and the firmware converts: `elbow servo = elbow + shoulder − 90`. Then fine-tune with `TRIM[2]` until the browser and real forearm angles match.

---

## 7. First power-on checklist

- [ ] Servo supply GND connected to Arduino GND
- [ ] Nothing powered from the Arduino 5V pin
- [ ] Capacitor polarity correct (stripe = −)
- [ ] E-stop switch reachable and tested (press = servos lose power)
- [ ] Arm parked near the home pose, area clear of fingers, cables, cups
- [ ] Speed slider low (30–60°/s) for the first tests
- [ ] `SAFE_MIN` / `SAFE_MAX` tightened after the first gentle sweep

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Arduino resets / disconnects when servos move | Servos powered from the Arduino, weak supply, or missing capacitor |
| Servos twitch or jitter | Missing common ground; long thin servo wires; supply too weak |
| "No device selected" / nothing in the picker | Charge-only USB cable; clone needs a CH340 driver; Serial Monitor still open |
| Status stuck at "Waiting for firmware…" | Wrong sketch uploaded, or wrong baud rate (must be 115200) |
| Arm → Sim says "read mode needs feedback" | `HAS_FEEDBACK` is 0 in the firmware |
| Ghost lags behind the sim | Expected when the firmware speed limit is lower than the browser's. Raise both slowly. |
| One joint mirrored | `INVERT` for that joint |
| A servo buzzes at one end | It's at its mechanical stop: tighten `SAFE_MIN`/`SAFE_MAX` or `PULSE_*_US` |
