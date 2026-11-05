# Driving the robot arm from Unreal Engine (OSC)

With this setup, anything in an Unreal level can move the simulated arm, and the real arm if one is connected. That includes a sphere you move by hand, an animated character's hand, a VR motion controller, or a Sequencer track. The arm also reports its pose back, so Unreal can show a live twin.

```
 Unreal Engine                       your computer                              desk
┌──────────────────┐  OSC/UDP :8000  ┌────────────┐  WebSocket :8080  ┌───────────────┐  USB   ┌──────────┐
│ ArmOscDriver     │ ───────────────▶│ osc-bridge │ ─────────────────▶│ browser sim   │ ──────▶│ real arm │
│ component / BP   │ ◀───────────────│ (Node.js)  │ ◀─────────────────│ IK + safety   │ ◀──────│ Arduino  │
└──────────────────┘  OSC/UDP :8001  └────────────┘                   └───────────────┘        └──────────┘
```

**Why the bridge?** Web pages can't open UDP sockets, and OSC runs over UDP. The bridge (`bridge/osc-bridge.mjs`) is about 100 lines of Node.js that translates between OSC and WebSocket.

**The browser always stays in the loop.** Unreal sends *wishes* (targets) and never raw servo commands. Every motion still goes through the simulator's IK, joint limits, speed limit and e-stop, and then through the firmware's own limits.

---

## 1. Quick start (5 minutes, no Unreal needed yet)

1. Start the bridge:

   ```bash
   npm run bridge
   ```

2. Start the simulator (`npm run dev`, or use your Vercel URL; see the note in Troubleshooting). Open **Unreal Engine · OSC** in the panel and click **Connect**. The **UE** badge in the header turns green.
3. Pretend to be Unreal:

   ```bash
   python3 tools/osc_test_sender.py
   ```

   The arm traces a circle and the gripper opens and closes. The script prints the `/arm/state` messages that come back.

If that works, the bridge and the simulator are fine and anything that goes wrong later is on the Unreal side.

---

## 2. OSC message reference

### Unreal → arm (send to UDP **8000**)

| Address | Arguments (floats) | Meaning |
|---|---|---|
| `/arm/target` | `x y z` | Where the gripper should go, in **cm relative to the arm base**: X forward, Y right, Z up (normal Unreal axes). The arm reaches about 30 cm. |
| `/arm/joints` | `base shoulder elbow [gripper]` | Direct joint angles in simulator degrees (switches the sim to sliders mode). |
| `/arm/gripper` | `open` | 0 = closed … 1 = fully open. |
| `/arm/home` | — | Run the homing routine. |
| `/arm/estop` | — | **Emergency stop.** Unreal can trigger it but **cannot clear it**: a person has to click *Reset* in the browser. |

### Arm → Unreal (listen on UDP **8001**, about 30 times a second)

| Address | Arguments | Meaning |
|---|---|---|
| `/arm/state` | `base shoulder elbow gripper` (floats, degrees) | Current joint angles. |
| `/arm/tip` | `x y z` (floats, cm) | Where the gripper actually is, in the same frame as `/arm/target`. |
| `/arm/status` | string | `ok`, `joint-limit`, `out-of-reach`, `estop`, `homing`, `reading`, `unreal`, … |

### Coordinate systems (the classic gotcha)

| | Forward | Right | Up | Units | Handedness |
|---|---|---|---|---|---|
| Unreal | +X | +Y | +Z | cm | left-handed |
| Simulator (Three.js) | +X | +Z | +Y | 1 unit = 10 cm | right-handed |

The browser converts with `sim = (ue.x, ue.z, ue.y) × 0.1` (`src/unreal-link.js`). Swapping two axes is exactly what converts between left- and right-handed coordinates, so you never have to think about it in Unreal. Just send normal Unreal positions relative to the arm base.

To pose an Unreal model from `/arm/state`:

- **Base** → the base part's relative **Yaw = −base**. A positive sim angle turns left (counter-clockwise from above), and a positive Unreal yaw turns right.
- **Shoulder** → the upper arm's relative **Pitch = shoulder**.
- **Elbow** → the forearm's relative **Pitch = elbow** (it's relative to the upper arm, exactly like a child bone).

---

## 3. Unreal project setup

1. **Edit → Plugins → search "OSC" → enable it** and restart the editor. The OSC plugin ships with Unreal.
2. Allow Unreal through your firewall for UDP if Windows asks.
3. Put an actor in the level where the arm's base should be (an Empty / `Actor`, or a mesh of the arm). This is your **ArmBase**: all targets are measured relative to it, so you can move or rotate it freely.

Then pick **A (Blueprint only)** or **B (C++ component)**.

---

## 4A. Blueprint only (no C++)

**Sender: make a sphere drive the arm**

1. Create a Blueprint Actor `BP_ArmTarget` with a small Sphere mesh.
2. Add variables:
   - `OscClient` (OSC Client object reference)
   - `ArmBase` (Actor, Instance Editable)
   - `WorkspaceScale` (Float, default 1.0)
3. **Event BeginPlay** → **Create OSC Client** (IP Address `127.0.0.1`, Port `8000`, Client Name `ArmClient`) → **Set OscClient**.
4. **Event Tick** (or a 30 Hz **Set Timer by Event**, which is lighter):
   1. **Get Actor Location** (self) → **Inverse Transform Location** with Transform = `ArmBase → Get Actor Transform` → multiply by `WorkspaceScale`.
   2. **Break Vector** into X, Y, Z.
   3. **Convert String to OSC Address** (`/arm/target`).
   4. Make an empty **OSC Message** (right-click the pin → *Make OSC Message*) → **Set OSC Message Address**.
   5. **Add Float** (X) → **Add Float** (Y) → **Add Float** (Z).
   6. `OscClient` → **Send OSC Message**.
5. Drop `BP_ArmTarget` into the level, set `ArmBase`, and press Play. Move it in the viewport (Simulate / eject with F8) and the browser arm follows.

**Receiver: a live twin of the arm inside Unreal**

1. Build a simple arm Blueprint `BP_ArmTwin` with Scene components arranged like the simulator (sizes in cm, 1 sim unit = 10 cm):
   `Root → BaseYaw (Z = 0) → Shoulder (Z = 9) → UpperArm mesh, Elbow (X = 16) → Forearm mesh, Tip (X = 14)`.
2. **BeginPlay** → **Create OSC Server** (Receive IP `127.0.0.1`, Port `8001`, Start Listening ✔) → store it → **Bind Event to On Osc Message Received**.
3. In the event: **Get OSC Message Address** → **Get OSC Address Full Path** → **Switch on String**:
   - `/arm/state` → **Get All Floats** → set relative rotations: `BaseYaw` Yaw = −[0], `Shoulder` Pitch = [1], `Elbow` Pitch = [2].
   - `/arm/status` → **Get All Strings** → e.g. turn a light red on `estop`.

**Gripper and buttons:** send `/arm/gripper` with one float from an input action (for example the trigger axis of a VR controller), and `/arm/estop` from a big red widget button.

---

## 4B. C++ component (`unreal/Source/ArmOscDriverComponent.*`)

The component wraps everything above. Attach it to any actor and it sends that actor's location, or a socket on its skeletal mesh, as `/arm/target`, and fires Blueprint events with the arm's state.

1. Copy `ArmOscDriverComponent.h` and `.cpp` into your project's `Source/<YourProject>/` folder.
2. In both files, replace `YOURPROJECT_API` with your module's API macro (for a project called `MyGame` it is `MYGAME_API`).
3. In `Source/<YourProject>/<YourProject>.Build.cs`, add `"OSC"`:

   ```csharp
   PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "OSC" });
   ```

4. Make sure the OSC plugin is enabled (step 3.1), then regenerate project files and build.
5. Add **Arm Osc Driver** to an actor. In Details:
   - **Arm Base**: the base actor in the level.
   - **Socket Name**: leave empty to follow the actor, or e.g. `hand_r` to follow a character's right hand.
   - **Workspace Scale**: `1` for 1:1. A person-sized motion (about 2 m) mapped onto the 30 cm desktop arm needs about `0.15`. Place ArmBase near the character's shoulder so the hand's offset is centred on the arm.
6. Bind **On Arm State** / **On Arm Tip** / **On Arm Status** in Blueprint to animate a twin or drive UI.
7. Call **Send Estop**, **Send Home**, **Set Gripper** and **Send Joints** from Blueprint as needed.

### Example: let a character's hand puppeteer the arm

1. Add the component to your character Blueprint, set **Socket Name** = `hand_r`, **Workspace Scale** = `0.15`, and **Arm Base** = an actor placed at the character's right shoulder height. You can also attach that actor to the character so it moves along.
2. Map an input (e.g. a gamepad trigger) to **Set Gripper**.
3. Play an animation or Sequencer clip. The real arm mimics the hand, with the speed limit smoothing out anything too fast for the servos.

> This C++ was written against the UE 5.3+ OSC plugin API but has **not been compiled here**. Engine versions occasionally rename OSC functions. If a name doesn't resolve, search the engine's `Plugins/Runtime/OSC/Source/OSC/Public/OSCManager.h` for the matching function. The Blueprint route in 4A uses the same functions under their display names.

---

## 5. Troubleshooting

| Symptom | Fix |
|---|---|
| Browser says "Bridge not reachable" | Is `npm run bridge` running? Is the URL `ws://localhost:8080`? |
| Bridge counts nothing "from Unreal" | Unreal must send to `127.0.0.1:8000`. Check the firewall. Try `tools/osc_test_sender.py` to confirm the bridge works. |
| Unreal receives nothing | OSC server port 8001 (`--reply-port` on the bridge). Only one program can listen on a port. |
| Arm doesn't move, status "Out of reach" | Targets are in cm relative to ArmBase and the arm reaches about 30 cm. Lower **Workspace Scale** or move ArmBase. |
| Arm moves mirrored left/right | Check you're sending Unreal coordinates (Y = right) and not converting yourself. |
| Works with `npm run dev` but not from the Vercel URL | The bridge only accepts pages from localhost by default. Run `npm run bridge -- --allow-origin https://your-app.vercel.app`. Some browsers also block public sites from talking to `localhost`. If so, use the local dev server. |
| Unreal on another PC | Run `npm run bridge -- --osc-host 0.0.0.0 --reply-host <unreal-pc-ip>` and point Unreal at the bridge PC's IP. |

**Safety reminder:** Unreal can request motion and trigger the e-stop, but it can't bypass joint limits or the speed limit, and it can't clear an e-stop. Keep a hand near the STOP button (or the real power switch) whenever an animation is driving real hardware.
