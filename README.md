# Robot Arm Twin

A browser-based robot arm simulator built with Three.js. It doubles as a **digital twin** of a cheap 4-servo desktop arm, and Unreal Engine can drive it over OSC.

- **Drag-to-reach IK**: drag the glowing target in 3D and the arm follows. The inverse kinematics is written from scratch: a closed-form law-of-cosines solver, plus a damped-least-squares fallback that respects joint limits. The math is explained in `src/ik.js`.
- **Sliders mode**: drive the base, shoulder, elbow and gripper directly (forward kinematics).
- **Digital twin over Web Serial**:
  - **Sim → Arm** streams the pose to a real arm, and a see-through ghost shows where the real arm reports it is.
  - **Arm → Sim** mirrors the real arm, using feedback servos or potentiometers, so you can move it by hand and record the motion.
- **Unreal Engine control (OSC)**: any actor or character hand socket can drive the arm, and the arm's state streams back. See [`unreal/UNREAL.md`](unreal/UNREAL.md).
- **Safety**: joint limits, a max-speed limiter, a latched emergency stop (button, Esc key, the arm's own button, or Unreal), and a slow homing routine. The firmware enforces limits, speed and e-stop again, and adds a watchdog.
- **Path recorder**: record, replay and loop motions, with the tip's path drawn in 3D.
- **Export**: download the recorded angles as a **Python script** (dry run, plot, PCA9685, or serial) or a standalone **Arduino playback sketch**.
- Works on phones (touch drag, pinch zoom, bottom sheet). Ready to deploy on Vercel.

**Learn how it works:** [LEARN.md](LEARN.md) explains forward and inverse kinematics simply and walks through every file, written for people coming from Blender and Unreal.
**Build the real arm:** [docs/HARDWARE.md](docs/HARDWARE.md) has the parts list (~US$40–60), wiring diagram, firmware setup and calibration.

## Run locally

Requires Node 20.19 or newer.

```bash
npm install
```

```bash
npm run dev
```

```bash
npm test
```

Unreal / OSC bridge (only needed for Unreal):

```bash
npm run bridge
```

## Deploy to Vercel

Import the repo in Vercel. It detects Vite automatically (`vercel.json` pins the settings): the build command is `npm run build` and the output directory is `dist`. Web Serial works on the deployed https site in Chrome and Edge on desktop. The OSC bridge runs on your own computer.

## Project layout

| Path | What |
|---|---|
| `src/` | Simulator: IK, 3D model, shaders, safety, serial link, Unreal link, UI |
| `firmware/robot_arm_twin/` | Arduino firmware for the live digital twin |
| `bridge/` | OSC (UDP) ⇄ WebSocket bridge for Unreal |
| `unreal/` | Unreal guide + `ArmOscDriverComponent` C++ example |
| `tools/osc_test_sender.py` | Fake Unreal for testing (Python standard library only) |
| `docs/HARDWARE.md` | Parts, wiring, calibration |

## Controls

| Action | Mouse | Touch |
|---|---|---|
| Move target | drag the glowing ball | drag the ball |
| Orbit | drag empty space | one-finger drag |
| Zoom | scroll | pinch |
| Pan | right-drag | two-finger drag |
| Emergency stop | STOP button or **Esc** | STOP button |

## Joint conventions

| Joint | Range | 0° means | Servo angle |
|---|---|---|---|
| Base | −90…90° | facing +X (forward) | `angle + 90` |
| Shoulder | 0…180° | upper arm horizontal | `angle` |
| Elbow | −150…30° | arm straight | `angle + 150` |
| Gripper | 0…90° | closed | `angle` |

Hardware quirks (reversed servos, trim, parallel linkages) are configured in the firmware.

## Serial protocol (115200 baud, one command per line)

`?` hello · `W`/`R` write/read mode · `T a b c d` target (servo degrees) · `V n` max speed · `H` home · `E` e-stop · `C` clear · `K` auto-calibrate · `L j a` learn calibration point. The arm replies `HELLO …`, `P a b c d` (position, 25 Hz), `S <state>`, `# log`, `! error`. Details are in `src/protocol.js`.
