# LEARN.md — How this robot arm simulator works

Written for someone who knows **Blender** and **Unreal** but is new to web code and shaders.
Wherever possible, each idea is compared to something you have already used.

---

## 0. The 30-second map

```
robot_arm_sim/
├── index.html          ← the page layout: panel, buttons, the 3D canvas  (like a UMG widget layout)
├── public/favicon.svg  ← the little tab icon
├── src/
│   ├── main.js         ← the "Level Blueprint": wires everything together, runs every frame
│   ├── ik.js           ← pure math: forward + inverse kinematics, joint limits
│   ├── arm.js          ← builds the 3D robot out of boxes and cylinders (the "rig") + its ghost
│   ├── shaders.js      ← two hand-written GPU shaders (grid floor, glowing target)
│   ├── recorder.js     ← records joint angles over time and plays them back (a timeline)
│   ├── exporters.js    ← turns a recording into a Python script and an Arduino sketch
│   ├── safety.js       ← speed limit, emergency stop, homing (the rules everyone must obey)
│   ├── protocol.js     ← the text "language" the browser and the Arduino speak
│   ├── serial.js       ← live USB connection to the real arm (Web Serial)
│   ├── unreal-link.js  ← receives Unreal's OSC messages (via the bridge) + coordinate conversion
│   └── styles.css      ← how the UI looks (colours, spacing, phone layout)
├── firmware/robot_arm_twin/robot_arm_twin.ino  ← the program that runs ON the Arduino
├── bridge/             ← small Node.js program: Unreal (OSC/UDP) ⇄ browser (WebSocket)
├── unreal/             ← Unreal Engine guide + C++ component
├── tools/osc_test_sender.py  ← pretends to be Unreal, for testing
├── docs/HARDWARE.md    ← parts list, wiring, calibration for the real arm
├── tests/              ← automatic checks (IK math, safety, protocol, OSC)
├── package.json        ← the project's "shopping list" of libraries + commands
├── vite.config.js      ← settings for Vite, the dev server / bundler
└── vercel.json         ← tells Vercel how to build and host the site
```

Run it:

```bash
npm install
```

```bash
npm run dev
```

Then open the address it prints (usually http://localhost:5173). `npm test` runs the math tests;
`npm run build` makes the optimised version that Vercel hosts.

---

## 1. Web code in one page (for 3D people)

A web page is made of three languages, each with one job:

| Language | Job | Closest thing you know |
|---|---|---|
| **HTML** (`index.html`) | *What* is on the page: a panel, buttons, sliders, a canvas | Unreal UMG hierarchy / Blender's Outliner for UI |
| **CSS** (`styles.css`) | *How it looks*: colours, sizes, layout, phone version | Material/style settings for that UI |
| **JavaScript** (`src/*.js`) | *What it does*: reacts to clicks, runs the 3D scene | Blueprints / Python scripts in Blender |

- **Three.js** is a JavaScript library that gives us a 3D engine inside the browser — scenes,
  cameras, lights, meshes, materials. It is a tiny Unreal/Eevee you control entirely with code.
- **Vite** is a helper program. While developing, it serves the files and reloads the page the
  instant you save (like hitting "Compile" in Blueprints automatically). For publishing,
  `vite build` bundles everything into a few optimised files in `dist/` (like "Package Project").
- **Vercel** is a hosting service. Connect the GitHub repo and it runs `npm run build` and puts
  `dist/` online — `vercel.json` just spells that out.
- **`import` / `export`**: each `.js` file is a *module*. A file `export`s the things it wants to
  share, and other files `import` them. It is like making a Blueprint Function Library and calling
  its functions from other Blueprints.

Three JavaScript patterns you will see everywhere:

```js
const x = 5;                     // a variable that won't be reassigned ("let" if it will)
const add = (a, b) => a + b;     // a small function ("arrow function")
button.addEventListener('click', () => { ... });  // "On Clicked" event → run this code
```

---

## 2. Forward kinematics (FK) — "angles in, position out"

**FK is what Blender does when you rotate bones in Pose Mode.** You set each joint's angle and the
hand ends up *somewhere*. The question FK answers: *where?*

Our arm has three joints that position the hand (plus a gripper that only opens and closes, so
it doesn't take part in the kinematics):

| Joint | Turns around | Means |
|---|---|---|
| **Base** | the vertical axis | which compass direction the arm faces |
| **Shoulder** | a horizontal hinge | how high the upper arm points (0° = flat forward, 90° = straight up) |
| **Elbow** | a horizontal hinge | how much the forearm bends relative to the upper arm (0° = straight) |

The trick: **walk along the arm, adding one bone at a time.**

1. Start at the shoulder.
2. The upper arm is an arrow of length `L1` pointing at the shoulder angle. Its end is the elbow.
3. The forearm is an arrow of length `L2`. Because the elbow angle is measured *relative to the
   upper arm* (a child bone!), its real direction is **shoulder angle + elbow angle**.
4. Its end is the tip.

In 2D (the arm's side view), with `cos` = "how much sideways" and `sin` = "how much up":

```
elbow = ( L1·cos(shoulder),  L1·sin(shoulder) )
tip   = elbow + ( L2·cos(shoulder+elbow),  L2·sin(shoulder+elbow) )
```

Then spin that whole side view around the vertical axis by the base angle — done.
That is the entire `forwardKinematics()` function in `src/ik.js`.

> In the 3D model (`arm.js`) we never do this math by hand: we *parent* the groups (base → shoulder →
> elbow → tip) and Three.js combines the rotations for us, exactly like Blender's bone hierarchy.
> The hand-written version in `ik.js` exists so the solver can "imagine" poses without moving the model.

**Sliders mode = FK.** You move the angles, the arm goes wherever they put it.

---

## 3. Inverse kinematics (IK) — "position in, angles out"

**IK is what Blender's IK constraint or Unreal's Two Bone IK node does.** You drag a target and
the joints work out how to bend so the hand touches it. It is the *reverse* question, and it's
harder, because there can be **zero, one, or several** answers.

We solve it in three easy steps, with nothing fancier than school geometry.

### Step 1 — Face the target (base)

Look down from above. The base must turn so the arm points at the target. The angle of a point
`(x, z)` on a map is given by `atan2` — think "what compass heading is this point at?".

### Step 2 — Bend the elbow (the triangle trick)

Now look from the side. Shoulder, elbow and target form a **triangle**:

```
               elbow
              /     \
         L1  /       \  L2
            /         \
   shoulder ─────────── target
                 d
```

We know all three side lengths: the two bones (`L1`, `L2`) and the distance `d` to the target.
A triangle with known sides has fixed angles — the **Law of Cosines** gives the elbow angle:

```
cos(elbow) = (d² − L1² − L2²) / (2·L1·L2)
```

- If the target is **too far**, no triangle exists (the formula gives a number outside −1…1).
  We clamp it, so the arm just stretches straight towards the target — the closest it can get.
  The status pill turns red: **Out of reach**.
- A triangle can be flipped: **elbow up** or **elbow down** both reach the same point.

### Step 3 — Aim the shoulder

Point the shoulder at the target, then rotate back a little to make up for the bend in the elbow.
That's two `atan2`s subtracted from each other.

### Step 4 — Choose an answer and respect joint limits

Two base choices (face the target, or face away and reach back over the shoulder) × two elbow
choices = up to **four** answers. We throw away any that break a joint limit (a real servo only turns
180°), then pick the one **closest to the current pose**, so the arm doesn't suddenly flip.

If every exact answer breaks a limit, a small **numerical solver** takes over
(`refineWithinLimits`). Picture someone trying to reach a shelf: they nudge each joint a bit, check
whether the hand got closer, keep the nudges that help, and stop when no nudge helps. A joint that
hits its stop stays there while the others keep trying. The status pill turns amber: **Joint limit**.

> Math nerd version (in the code comments): it's *Damped Least Squares* on a numerically measured
> *Jacobian* — a 3×3 table of "if I turn joint *i* a tiny bit, how does the tip move?"

---

## 4. The code, file by file

### `src/ik.js` — the brain (pure math)

- `ARM` holds the bone lengths. Change them and the model and solver both update.
- `JOINTS` holds each joint's name, limits (in degrees) and home pose: base, shoulder, elbow and
  gripper. The three arm joints each span exactly 180° so they map onto a hobby servo's 0–180°.
  `IK_JOINTS = 3` says only the first three take part in IK.
- `simToServo` / `servoToSim` convert between simulator angles and "servo degrees"
  (`servo = angle − min`). The exporters, the USB link and the firmware all use this one rule.
- `forwardKinematics(q)` — section 2 above.
- `solveIK(target, current)` — section 3 above. Returns `{ q, error, status }`.
- `refineWithinLimits(...)` — the "nudge and check" fallback.
- Angles are stored in **radians** (a full circle = 2π ≈ 6.28) because that's what `Math.sin`
  etc. use. The UI shows degrees. `toDeg` / `toRad` convert.

There is no Three.js in this file on purpose: it can be tested on its own (`npm test`) and copied
into any other project — even translated to Python for your real robot.

### `src/arm.js` — the rig

Builds the robot from primitives, like blocking out a model in Blender:

- **`THREE.Group`** = an Empty. It has a position/rotation and children but no mesh.
- **`THREE.Mesh(geometry, material)`** = a mesh object. `BoxGeometry` and `CylinderGeometry` are
  Add → Mesh → Cube / Cylinder.
- **`MeshStandardMaterial`** = a PBR material (Principled BSDF): `color`, `metalness`, `roughness`,
  plus `emissive` for the glowing joint colours.
- **Origins/pivots**: a box is created centred on its origin. To rotate an arm segment around its
  *end*, we put it inside a Group (the pivot) and slide the box half its length along X — the same
  as moving the origin to the end of an object in Blender.
- The hierarchy `baseYaw → shoulder → elbow → tip` is the bone chain. `setAngles(q)` just sets
  three rotations; parenting does the rest.
- `castShadow` / `receiveShadow` = the shadow toggles in an object's render settings.
- The gripper fingers slide apart as the 4th angle (`q[3]`) grows. They are moved by code, like a
  driver in Blender.
- `createArm({ ghost: true })` builds the same rig with one see-through, unlit material
  (`MeshBasicMaterial`, `opacity: 0.22`, `depthWrite: false`): an onion-skin of the real arm.

### `src/shaders.js` — writing materials as code

In Unreal's Material Editor or Blender's Shader Editor you connect nodes; underneath, the engine turns
that graph into a **shader**: a small program that runs on the GPU in a language called **GLSL**.
Here we write it directly. Every shader is two programs:

| Part | Runs | Decides | Node-editor equivalent |
|---|---|---|---|
| **Vertex shader** | once per vertex | *where* each corner lands on screen | World Position Offset / Displacement |
| **Fragment shader** | once per pixel | *what colour* that pixel is | everything plugged into Base Color / Emission / Opacity |

Two kinds of inputs:

- **uniforms** — values from JavaScript, the same for every pixel (= *Parameters* in Unreal, a
  *Value* node in Blender). E.g. `uColor`, `uTime`.
- **varyings** — values the vertex shader hands to the fragment shader, smoothly blended across
  each triangle (like how vertex colours blend). E.g. `vWorldPos`.

**Grid floor** (`createGridMaterial`)

- The floor is one flat square; the lines are *calculated*, not painted in a texture.
- `fract(position / spacing)` repeats 0→1 every grid cell — like a Math node set to *Fraction*
  feeding a ramp. Pixels near 0 or 1 are on a line.
- `fwidth()` asks "how much does this value change from one pixel to the next?". Dividing by it
  keeps lines exactly ~1 pixel wide whether they're near the camera or far away — free anti-aliasing.
- The red/blue lines are the X and Z axes, like Blender's viewport. A soft ring marks the arm's
  maximum reach.
- `smoothstep` fades the floor out with distance so there's no hard edge (a Map Range node with
  smooth interpolation).

**Glowing target** (`createGlowMaterial`)

- A **Fresnel** effect: `dot(normal, viewDirection)` is 1 where the surface faces you and 0 at the
  silhouette. `1 − that` glows at the edges, like a soap bubble — the same as the *Fresnel* /
  *Layer Weight* node.
- `sin(uTime)` makes it gently pulse; JavaScript updates `uTime` every frame.
- In `main.js` the target has `depthTest = false`, so it draws on top of everything like an editor
  gizmo and never hides inside the gripper.

### `src/main.js` — the Level Blueprint

Read it top to bottom; the sections are numbered.

1. **Renderer, scene, camera.** `WebGLRenderer` is the render engine. `Scene` is the world outliner.
   `PerspectiveCamera(45, …)` is a camera with a 45° field of view. `OrbitControls` gives you
   orbit/zoom/pan like the viewport navigation. Lights: a `HemisphereLight` (sky/ground fill, like
   a world colour), a `DirectionalLight` "sun" that casts shadows, and a blue rim light. ACES tone
   mapping is the filmic look curve.
2. **The arm, its ghost, the target and helper lines.** The target is a glowing sphere plus a *bigger
   invisible sphere* that's easy to hit with a finger. Thin lines give depth cues: one drops to the
   floor, one shows the gap when the arm can't reach, one draws the recorded path. The **ghost** is
   a second, see-through copy of the arm that shows where the *real* arm says it is.
3. **App state.** One small object holds everything that changes: the mode, the angles shown
   right now (`q`) and the angles the arm is heading to (`goal`), plus what the real arm last reported.
4. **Safety.** Wires the STOP button, the Esc key, Reset, homing and the speed slider to `safety.js`.
   `canDrive()` answers "may anyone move the arm right now?" and every input checks it first.
5. **Joint sliders** are created by code from the `JOINTS` list, so adding a joint adds a slider.
   Each shows the simulator angle and the matching servo angle. Moving an arm slider switches to
   sliders mode; the gripper slider works in both modes.
6. **Mode switch / drag plane** — simple click handlers.
7. **Dragging in 3D — raycasting.** The mouse is 2D, the target is 3D. We:
   1. shoot a **ray** from the camera through the pointer (like a Line Trace by Channel in Unreal),
   2. hit an invisible **plane** through the target,
   3. move the target to where the ray hits.

   The plane you choose decides the motion: *Free* faces the camera, *Horizontal* is parallel to the
   floor, *Height* only moves up/down. We listen with `capture: true` so we hear the click *before*
   the orbit controls do — that's how grabbing the ball doesn't also spin the camera.
8. **Digital twin** — the USB connection to the real arm (section 5 below explains it).
9. **Unreal Engine** — reacting to OSC messages from Unreal (section 5 below).
10. **Recorder UI** — buttons call the recorder and refresh labels.
11. **Export dialog** — generates the code text, shows it, copies it, or downloads it as a file
    (a `Blob` is a file made in memory).
12. **Layout.** On a phone the panel covers the bottom half, on desktop the right side.
    `camera.setViewOffset` shifts the rendered picture so the arm sits in the *visible* part —
    like a camera's *Shift X/Y* lens setting in Blender.
13. **The animation loop.** `requestAnimationFrame(tick)` asks the browser to call `tick` before
    the next screen refresh — this is **Event Tick**. Each tick:
    - picks **who drives the arm** this frame, in priority order: e-stop → homing → real arm (read
      mode) → recording playback → you / Unreal,
    - moves the joints towards `goal` with `stepTowards` — a gentle ease *and* a hard speed cap,
    - records the pose if recording,
    - streams the pose to the real arm and shows the ghost,
    - updates the model, sliders, status pill, shader time, and helper lines,
    - sends the arm's state to Unreal,
    - renders the frame.

    `dt` (delta time, seconds since the last frame) keeps motion speed the same on 30 Hz and
    120 Hz screens — exactly like multiplying by *Get World Delta Seconds*.

### `src/recorder.js` — a tiny timeline

- A recording is a list of **keyframes** `{ t: seconds, q: [base, shoulder, elbow, gripper] }`,
  captured up to 30 times per second.
- **Playback** finds the two keyframes around the current time and blends them — *linear
  interpolation*, the "Linear" interpolation mode in Blender's Graph Editor. A *binary search*
  finds the right pair quickly (like opening a dictionary in the middle instead of reading from A).
- `resample(step)` produces evenly spaced frames, which real hardware wants.
- The recording is saved in **`localStorage`** (a small save slot inside your browser), so it
  survives a page reload.

### `src/exporters.js` — talking to real hardware

Both exporters take the same thing: rows of joint angles in degrees, `stepMs` apart. They build
the program as text (JavaScript template strings, ``` `like ${this}` ```).

**Simulator angle → servo angle.** Every arm joint's range is 180° wide (the gripper uses 0–90°), so:

```
servo = simulatorAngle − jointMinimum        (always within 0…180)
servo = 180 − servo   if the servo is mounted backwards (INVERT)
servo = servo + TRIM                          (small calibration offset)
```

The generated files keep the raw simulator angles and do this conversion themselves, so you can fix
`INVERT` and `TRIM` right there without re-exporting.

**Python script** (`robot_arm_motion.py`)

- `python3 robot_arm_motion.py` — dry run, prints the servo angles.
- `--plot` — graphs the motion (needs `matplotlib`).
- `--backend servokit` — Raspberry Pi + PCA9685 servo board (`pip install adafruit-circuitpython-servokit`).
- `--backend serial --port /dev/ttyUSB0` — streams angles over USB to an Arduino running the
  digital-twin firmware (`pip install pyserial`). The firmware still applies its own safety limits.
- Timing uses a schedule (`start + i·dt`) instead of plain sleeps, so small delays don't add up.

**Arduino sketch** (`robot_arm_motion.ino`)

- Angles are stored as whole numbers ×10 (−1000 = −100.0°) in **PROGMEM** = flash memory. An Arduino
  Uno has only 2 KB of working memory but 32 KB of flash, so big tables must live there. The export
  dialog warns when a recording is too big — lower the sample rate.
- `writeMicroseconds` sends a precise pulse (500–2500 µs ≈ 0–180°) for smoother motion than whole
  degrees. Many servos prefer 1000–2000 µs; adjust `PULSE_MIN_US`/`PULSE_MAX_US`.
- `loop()` uses `millis()` timing instead of `delay()` so frames stay evenly spaced.
- It's a *standalone* player: once uploaded, the arm replays the motion without a computer. A button
  on D2 pauses it. Pins match `docs/HARDWARE.md`. For live control use the twin firmware instead.

### `src/styles.css` — the look

- **Custom properties** (`--accent: #38bdf8`) at the top are design tokens — change one, the whole
  UI updates. Like a Material Parameter Collection.
- **Flexbox / Grid** (`display: flex`, `display: grid`) are auto-layout, like UMG's Horizontal/Vertical Box.
- **`@media (max-width: 720px)`** = "if the screen is narrow (a phone), use these rules instead".
  That's what turns the side panel into a bottom sheet you can collapse.
- `env(safe-area-inset-…)` keeps things clear of the iPhone notch and home bar.
- `touch-action: none` on the canvas tells the phone "don't scroll or zoom the page here — the app
  handles touches itself".

### `index.html` — the layout

Plain HTML elements: a `<canvas>` (where Three.js draws), a header with the status pill, an
`<aside>` panel with `<section>`s, and a `<dialog>` for exporting. Elements have `id`s so
JavaScript can find them (`document.getElementById('recBtn')` — like getting a widget reference by
name). The last line loads `src/main.js`, which starts everything.

### `tests/` — automatic checks

`ik.test.mjs` picks 500 random legal poses, computes where the tip would be (FK), asks IK to find
angles for that point, and checks the tip lands within 1 mm. It also checks the out-of-reach,
behind-the-robot and joint-limit cases. `twin.test.mjs` checks the speed limit, the e-stop latch,
the serial protocol parser, the OSC encoder/decoder and the Unreal coordinate conversion.
Run `npm test` after changing any of them.

---

## 5. The digital twin — connecting to a real arm and to Unreal

A **digital twin** is a virtual copy that stays in sync with a physical thing. Here, data flows both
ways: the browser can drive the real arm, and the real arm can drive the browser.

```
 Unreal ──OSC──▶ bridge ──WebSocket──▶ BROWSER (IK, safety, recorder) ──USB serial──▶ Arduino ──▶ servos
 Unreal ◀─────── bridge ◀───────────── BROWSER ◀───── position reports ───────────── Arduino ◀── sensors
```

### Talking over USB: `serial.js` + `protocol.js`

- **Web Serial** is a browser feature that lets a page talk to a USB device after you pick it from a
  list (for privacy, a page can never grab a device by itself). It works in Chrome and Edge on
  desktop, on `https://` or `localhost` pages.
- The data arrives through **streams**, which are pipes: bytes come in → a `TextDecoderStream`
  turns them into text → we cut the text into lines. Data comes in random-sized chunks, a bit like
  packets arriving out of order in multiplayer, so `createLineSplitter` keeps the unfinished end
  of a chunk until the rest arrives.
- The **protocol** is deliberately plain text, one command per line: `T 90.0 70.0 50.0 45.0` means
  "go to these servo angles", `E` is emergency stop, and the arm replies `P …` with its position
  about 25 times a second. Plain text means you can debug by typing into the Arduino Serial Monitor.
- `sendTarget` is **throttled** to 30 messages per second and only sends when something changed,
  plus a "keepalive" every 250 ms. An Arduino Uno's input buffer is only 64 bytes, so flooding it
  makes motion stutter, like a network-saturated multiplayer game.

**The two directions:**

- **Sim → Arm**: every frame, `main.js` converts the shown angles with `simToServo` and streams them.
  The **ghost** shows the arm's *reported* position, so lag or a stalled joint is visible at a glance.
- **Arm → Sim** (read mode): the firmware lets the servos go limp and reports what its sensors
  measure. The sim's `goal` becomes that pose. Recording still works, so you can **teach by
  demonstration**: move the real arm by hand, record, then play it back. It's the robotics
  version of motion capture.

### Safety: `safety.js` and the firmware

Think of it as layers, like collision channels and blocking volumes all applying at once:

1. **Joint limits** — every angle is clamped into its legal range, in the browser *and* in the
   firmware (`SAFE_MIN`/`SAFE_MAX`).
2. **Max speed** — `stepTowards` first eases towards the goal (natural motion), then **caps** how far
   each joint may turn this frame: `maxSpeed × dt`. However fast you drag, or however wild an Unreal
   animation gets, the joints can't exceed it. The firmware applies its own cap too.
3. **Homing** — a slow (30°/s), predictable move back to a known pose. The firmware also homes on
   power-up, starting from the real measured position when it has sensors so nothing jumps.
4. **Emergency stop** — STOP button, Esc key, the button on the arm, or `/arm/estop` from Unreal.
   It's **latched** (it stays on) until a *person* clicks Reset in the browser. Remote systems may
   stop the arm but never restart it. That's a common rule in real industrial robotics.
5. **Watchdog** — if the browser crashes or the cable is pulled mid-move, the firmware hears nothing
   for 1 s and holds still instead of carrying on.

`safety.js` is a tiny **state machine**: `normal`, `homing` or `estop`, with rules for moving
between them. You've built these before as animation state machines in Unreal or Blender drivers.

### The firmware: `firmware/robot_arm_twin/robot_arm_twin.ino`

Arduino code is C++. Every sketch has `setup()` (runs once, like *BeginPlay*) and `loop()` (runs
forever, like *Tick*). Ours never uses long `delay()`s in the loop. Instead it checks `millis()`
(time since boot) and does a little work every 20 ms, so it can always hear an `E` instantly.
Calibration values are stored in **EEPROM**, a tiny memory that survives power-off (a save game).

### Unreal: `bridge/`, `unreal-link.js`, `unreal/`

- **OSC** (Open Sound Control) is a simple message format used by Unreal, TouchDesigner, lighting desks,
  etc. A message is an address like `/arm/target` plus some numbers. `bridge/osc.mjs` encodes and
  decodes it by hand: text padded to 4 bytes, then big-endian floats.
- Browsers can't receive OSC (it uses UDP, which pages aren't allowed to open), so
  `bridge/osc-bridge.mjs` relays it to the page over a **WebSocket**, a long-lived two-way
  connection. It only accepts pages from your own computer, so a random website can't move your robot.
- **Coordinate conversion** is the classic gotcha: Unreal is Z-up, left-handed, in cm; Three.js is
  Y-up, right-handed. `unrealToSim` swaps Y and Z (which also flips handedness) and scales cm to sim units.
- `unreal/UNREAL.md` shows both a Blueprint-only setup and a C++ component that makes any actor, or a
  character's hand socket, drive the arm.

---

## 6. Driving a real arm — checklist

The full guide, with a parts list, wiring diagram and calibration, is **`docs/HARDWARE.md`**. Short version:

1. Build an arm whose proportions match `ARM` in `ik.js` (or change `ARM` to match yours — the
   simulator and exports follow).
2. Mount each servo so that **servo 90°** matches this pose: base facing forward, upper arm pointing
   straight up, forearm pointing forward 30° above horizontal (elbow −60°), gripper half open.
   The slider labels show the servo angle for every pose, so use them to check.
3. If a joint moves the wrong way, set its `INVERT` to true. If it's a few degrees off, use `TRIM`.
4. **Power servos from a separate 5–6 V supply** and connect its ground to the Arduino's ground.
   Four servos can draw far more current than USB provides.
5. Tighten `SAFE_MIN`/`SAFE_MAX` in the firmware so the arm can't hit the table.
6. First test with a low speed limit and keep a hand near the STOP button and the power switch.

---

## 7. Things to try next

- Change `ARM.L1` / `ARM.L2` or a joint's `min`/`max` in `ik.js` and watch the solver adapt.
- Add a wrist pitch joint. FK gets one more arrow; IK can keep the wrist at a fixed angle
  relative to the ground and solve the rest exactly as now.
- In `shaders.js`, change the grid spacing or make the reach ring pulse with `uTime`.
- Replace the box geometry in `arm.js` with a model from Blender exported as `.glb`
  (`GLTFLoader` in Three.js) — keep the same parent/child pivot structure.
- In Unreal, attach the driver component to a VR motion controller and puppeteer the arm with your hand.
- Add a "collision box" to `safety.js` that rejects targets inside a no-go zone (e.g. a cup on the desk).
