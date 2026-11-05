# Robot Arm Sim

A browser-based 3-joint robot arm simulator built with Three.js.

- **Drag-to-reach IK**: drag the glowing target in 3D and the arm follows. The inverse kinematics is written from scratch (closed-form law-of-cosines solver plus a damped-least-squares fallback that respects joint limits). The math is explained in `src/ik.js`.
- **Sliders mode**: drive the base, shoulder and elbow directly (forward kinematics).
- **Path recorder**: record a motion, replay it (optionally on a loop) and see the tip's path drawn in 3D. Recordings are kept across reloads.
- **Export for a real arm**: download the recorded joint angles as a **Python script** (dry run, plot, Raspberry Pi PCA9685, or serial streaming) or an **Arduino servo sketch** (standalone playback from flash, or a serial receiver).
- Works on phones and tablets (touch drag, pinch zoom, collapsible bottom sheet). Ready to deploy on Vercel.

New to web code? Read **[LEARN.md](LEARN.md)**: it explains forward and inverse kinematics simply and walks through every file.

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

## Deploy to Vercel

Import the repo in Vercel. It detects Vite automatically (`vercel.json` pins the settings): the build command is `npm run build` and the output directory is `dist`.

## Controls

| Action | Mouse | Touch |
|---|---|---|
| Move target | drag the glowing ball | drag the ball |
| Orbit | drag empty space | one-finger drag |
| Zoom | scroll | pinch |
| Pan | right-drag | two-finger drag |

"Drag moves the target…" picks the drag plane: **Free** (parallel to the screen), **Horizontal** (constant height) or **Height** (up/down only).

## Joint conventions

| Joint | Range | 0° means | Servo |
|---|---|---|---|
| Base | −90…90° | facing +X | `angle + 90` |
| Shoulder | 0…180° | upper arm horizontal | `angle` |
| Elbow | −150…30° | arm straight | `angle + 150` |

Calibrate `INVERT` and `TRIM` per servo inside the exported files.
