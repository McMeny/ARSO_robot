# Eye to Steering: a Humanistic-Intelligence Smart Walker

An assistive robotic walker that uses the **human visual system as a second sensor** alongside its LiDAR, and the
user's **head direction** to learn where they want to go. The human stays in the walker's feedback loop the whole
time. The walker adjusts how it handles each hazard depending on whether the user appears to have seen it, and the
balance of steering control between human and machine shifts with the level of risk.

This repository is the **VR evaluation build**: the walker, the store and the hazards are simulated (ROS 2 + Gazebo),
the user experiences them in a **Meta Quest 3** in full VR, and a **Muse EEG headband with an Oz electrode** reads
their visual response. The aim is to evaluate the concept before any hardware is funded.

> **Status: in progress.** This README describes the system as designed. The [Status](#status) table shows which
> parts already exist and which are still being built.

> **HAVE A QUEST 3 AND A MUSE? [READ HERE: SETUP AND USE INSTRUCTIONS FOR VR](#read-here-setup-and-use-instructions-for-vr-quest-3-and-muse)**

---

## Contents
1. [Background](#background)
2. [How it works](#how-it-works)
3. [Safety rules](#safety-rules)
4. [The VR evaluation build](#the-vr-evaluation-build)
5. [**READ HERE: SETUP AND USE INSTRUCTIONS FOR VR (QUEST 3 AND MUSE)**](#read-here-setup-and-use-instructions-for-vr-quest-3-and-muse)
6. [Trying it without a headset or Muse (testing only)](#trying-it-without-a-headset-or-muse-testing-only)
7. [Operator controls](#operator-controls)
8. [Settings you might change](#settings-you-might-change)
9. [Troubleshooting](#troubleshooting)
10. [Repository layout](#repository-layout)
11. [Status](#status)
12. [Why this is humanistic intelligence](#why-this-is-humanistic-intelligence)

---

## Background

**Eye as a Camera** uses the human eye as an imaging sensor. A flickering square scans across an image, and a
consumer EEG headband records the steady-state visually evoked potential (SSVEP) at each position. The strength of
that response reports the brightness at each point, so the scene can be reconstructed from what the person saw.
The body's own sensing apparatus (eye and visual cortex) becomes a working sensor, read out by a Muse with an added
Oz cup electrode and a VR headset. But that system is **open-loop and read-only**: it reports what the subject saw,
and nothing acts on it.

This project **closes the loop between sensing and actuation**. The same SSVEP method tells a walker whether its
user has registered each hazard, and the walker changes how it behaves accordingly.

---

## How it works

The walker drives forward at a slow walking pace through an unfamiliar environment, while the user walks with it,
holding the handles.

### The hazard loop

The user doesn't need to do anything deliberate. There is no target to select and nothing to stare at. The system
decides what to probe, and treats the human as a measurement.

| Step | What happens |
|---|---|
| **1. Detect** | The LiDAR picks up an obstacle and the walker tracks it on its map. |
| **2. Decide if it matters** | Only *relevant* obstacles are probed: on or near the walker's path, a few metres ahead, and not already confirmed as seen. Walls and long shelves are treated as structure, not hazards. |
| **3. Send to the headset** | The hazard's position goes to the headset. (In VR the headset and the map share one frame. On a real walker a calibration step aligns them.) |
| **4. Tag it** | The headset puts a subtle flickering patch on the object, at a frequency assigned to that hazard. Each hazard in view has its own frequency, so up to two can be probed at once. Others wait in a queue. |
| **5. Measure** | The decoder checks whether that frequency appears in the EEG at Oz, compared with frequencies that aren't being shown. |
| **6. Act** | The walker **always** avoids the obstacle. If the response is present, it avoids it gently, at normal speed with normal clearance, and the tag fades. If the response is absent, or there's no reliable EEG signal, it **escalates**: a stronger visual cue, a lower speed and a wider berth. |

**Probing timeline:** each hazard is probed from about **4 m** away along the path, at normal speed. The decision is
made by about **2.5 m**. Only then, if the hazard still isn't registered, does the walker escalate. At walking pace
this gives about 5 s, which is room for 2–3 EEG decoding windows. A hazard that appears closer than 4 m (for
example, from behind a shelf) is probed straight away, and defaults to cautious if there isn't time to decide.

### The trajectory display

The headset projects the walker's short-term path onto the floor: a **centre line and two edge lines** marking its
full footprint over the next **2 seconds**. The user always sees where they and the walker are about to go.
- The edges **widen** around a hazard that's getting extra clearance.
- The lines **bend** to preview a head-guided turn before the walker follows it.
- The edges become **more solid** as the machine takes more control.

### Head-guided direction

A planner on its own doesn't know where the user wants to go, which is one reason autonomous paths in open space can
look aimless. People tend to turn their heads shortly before changing walking direction, so the walker uses head
direction to choose where to go. Steering authority is **shared according to risk**, and risk is measured as
clearance: the distance to the nearest obstacle along the walker's path.

| Situation | Who steers |
|---|---|
| **Open space** (little at stake) | The user's head direction sets the heading of the walker's forward goal, and the planner plans a path to it. The walker goes where the user is facing. |
| **Obstacles nearby** | The head's influence fades as clearance shrinks. It then only chooses between open ways forward at decision points, such as the end of an aisle, a junction or a doorway. |
| **Collision imminent** | The planner and the collision monitor are fully in charge. |

To find the open ways forward, the walker checks a **fan of candidate arcs** ("tentacles"), from hard left through
straight to hard right, against the LiDAR. Head direction picks among the arcs that are clear.

**Safeguards against accidental steering:**
- Head direction is low-pass filtered over 1–2 s, with a deadband of about ±12° and a limit on turn rate. Brief
  glances average out, and only a sustained look produces a gentle turn.
- Looks towards a tagged hazard are ignored, because those are the hazard probe at work, not a choice of direction.
- Head direction only **biases** the planner's goal. It's never a direct command, so obstacle avoidance still plans
  every path.
- Every turn is previewed on the floor display first. Looking back straightens the preview and cancels the turn.

### Speed: the lowest limit always wins

```
actual speed = lowest of ( grip ,  hazard cap ,  collision monitor )
```

The **grip** sets the speed the user asks for. The **hazard cap** comes from the EEG result and can only *lower* the
ceiling, never raise it. The **collision monitor** can always slow or stop the walker.

| Grip | Hazard nearby? | EEG result | Result |
|---|---|---|---|
| Held | none | – | normal walking pace |
| Held | yes | registered | normal pace, normal clearance |
| Held | yes | not registered, or no signal | reduced pace (40 %), wider clearance, stronger cue |
| Released | – | – | stop |

On the real walker the grip is a **two-stage lever** on the handle (half-squeezed means slow, fully squeezed means
normal, released means stop), read directly by the motor controller and wired fail-safe. **In this VR build the
user's hands are assumed to be on the walker at all times, so the grip counts as held.** The operator starts and
stops the walker from the PC.

---

## Safety rules

These hold in every mode, and nothing in the software can switch them off.

1. **The EEG never commands motion.** The walker avoids every obstacle regardless. The EEG only changes how
   cautiously it does so.
2. **Head direction only biases the planner's goal**, and its influence fades as obstacles get closer.
3. **Obstacle avoidance and the collision monitor run at all times.**
4. **Missing or uncertain EEG means cautious behaviour.**
5. **The walker stops if commands stop.** If the planner or any input goes silent, the walker receives a zero
   command rather than carrying on at its last speed.
6. **No surprise motion.** Any slowing or change of course is shown on the floor display before it happens. The
   walker does not perform sudden recovery spins.
7. **Speed is capped** at a slow walking pace.
8. **Photosensitivity:** flicker is limited to **6–13 Hz** (avoiding roughly 15–25 Hz), flicker contrast is capped,
   and the operator can stop all flicker instantly. This build assumes participants don't have photosensitive
   epilepsy. A formal study will usually need screening added.

---

## The VR evaluation build

Testing in VR changes some things compared with the real walker. These are deliberate, and they're listed here so
the results are read correctly.

| Real walker | VR evaluation build |
|---|---|
| Passthrough mixed reality: tags on real objects | **Full VR**: the user sees a virtual store, and tags are placed on virtual objects |
| Headset-to-map calibration needed | Not needed: the headset view *is* the map frame |
| User walks behind the walker | User is **seated** (swivel chair recommended). Their view moves with the simulated walker at a standing eye height of about 1.6 m. |
| Two-stage grip lever | Grip counts as **always held**. The operator starts and stops. |
| LiDAR, motors, real store | Simulated in Gazebo: LiDAR, IMU, wheel odometry, a 20 × 14 m store with aisles and clutter |

**Known limitations of the VR build:**
- **Clean EEG.** A seated user produces far fewer movement artifacts than someone walking. The VR build can't show
  whether SSVEP survives real walking, which is the biggest open risk for the funded build.
- **Motion sickness.** The user's view moves while their body doesn't. The walker moves slowly and the app uses a
  comfort vignette, but some people may still feel it.
- **What SSVEP measures.** A present response is evidence that the tag reached the user's visual cortex. Whether that
  predicts *noticing* the hazard, especially in peripheral vision, is something the experiments are meant to test.

### How the pieces connect

```
                 Windows PC                                   WSL2 (Ubuntu 22.04, ROS 2 Humble)
 ┌──────────────────────────────────┐        ┌──────────────────────────────────────────────────────┐
 │ Muse ──Bluetooth──► BlueMuse /   │  LSL   │  EEG decoder ──► probe results ──► hazard loop        │
 │                     muselsl ─────┼───────►│                                       │               │
 └──────────────────────────────────┘        │  Gazebo (walker + store) ◄── Nav2 ◄── walker brain    │
                                             │      │ LiDAR                     ▲        ▲           │
 ┌──────────────────────────────────┐  Wi-Fi │      └──► hazard tracker ────────┘        │           │
 │ Quest 3 browser (WebXR app)      │◄──────►│  rosbridge (wss :9090)   head steering ───┘           │
 │  draws store, walker, tags,      │        │  web server (https :8443)                            │
 │  floor trajectory; sends head    │        │  RViz (operator view)                                │
 │  pose                            │        └──────────────────────────────────────────────────────┘
 └──────────────────────────────────┘
```

- The Muse connects by Bluetooth, which **WSL2 can't use**, so it streams from Windows over LSL.
- The Quest's browser loads the VR app from the PC, and talks to ROS through rosbridge.
- Gazebo simulates the walker. The headset only *draws* the scene, rebuilt from the same world file.

---

## READ HERE: SETUP AND USE INSTRUCTIONS FOR VR (QUEST 3 AND MUSE)

This is the full setup, with the real headset and the real EEG.

### What you need
- A Windows 10/11 PC with **WSL2 (Ubuntu 22.04)** and **ROS 2 Humble**, with this repository built
- A **Meta Quest 3** on the **same Wi-Fi network** as the PC
- A **Muse 2 or Muse S** with an **Oz cup electrode** on its AUX input (Ag/AgCl, with gel), as in Eye as a Camera
- **BlueMuse** (or Python `muselsl`) on Windows
- A swivel chair for the participant

### Part A: one-time setup (about 30 minutes)

**A1. Install the extra ROS packages** (in WSL):
```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite
pip install pylsl
```

**A2. Build the workspace** (in WSL):
```bash
cd ~/eye2steering_ws
colcon build
source install/setup.bash
```

**A3. Turn on mirrored networking for WSL.** This lets the Quest reach the PC, and lets LSL find the Muse stream.
On Windows, open `C:\Users\<you>\.wslconfig` in Notepad (create it if it doesn't exist) and add this under
`[wsl2]`:
```ini
[wsl2]
networkingMode=mirrored
```
Then, in **PowerShell**:
```powershell
wsl --shutdown
```
and reopen your Ubuntu terminal.

**A4. Open the firewall ports.** In **PowerShell as Administrator**:
```powershell
New-NetFirewallRule -DisplayName "Walker VR web"       -Direction Inbound -Protocol TCP -LocalPort 8443,9090 -Action Allow
New-NetFirewallRule -DisplayName "Walker LSL (TCP)"    -Direction Inbound -Protocol TCP -LocalPort 16571-16604 -Action Allow
New-NetFirewallRule -DisplayName "Walker LSL (UDP)"    -Direction Inbound -Protocol UDP -LocalPort 16571-16604 -Action Allow
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

**A5. Make the HTTPS certificate.** The Quest only allows VR on secure (https) pages. In WSL:
```bash
ros2 run walker_vr make_certs
```

**A6. Find the PC's IP address.** In PowerShell, run `ipconfig` and note the **IPv4 address** of your Wi-Fi
adapter, for example `192.168.1.42`. You'll type it into the Quest. Below it's written as `<PC-IP>`.

**A7. Let the Quest trust the certificate** (once per certificate). In the Quest browser, open
`https://<PC-IP>:8443`, choose **Advanced → Proceed** on the warning, then also open `https://<PC-IP>:9090` and do the
same. (The second page will show an error after you accept. That's expected.)

### Part B: each session (about 15 minutes)

**B1. Fit the Muse and the Oz electrode.** Put the Muse on, put gel on the Oz cup electrode and place it on the back
of the head, on the midline just above the bump at the base of the skull. Connect it to the Muse's AUX input.

**B2. Start the Muse stream on Windows.** Open **BlueMuse**, select your Muse, click **Start Streaming**. (With
`muselsl` instead: `muselsl stream` in a Windows terminal.)

**B3. Check the EEG in WSL.** Each check prints PASS or FAIL with what to fix:
```bash
ros2 run my_robot_autonomy eeg_check stream   # the stream arrives, 256 Hz, 5 channels including AUX (Oz)
ros2 run my_robot_autonomy eeg_check alpha    # participant closes their eyes for 10 s: alpha shows at Oz = good contact
```

**B4. Start the system.** Open a WSL terminal (**Terminal 1**) and paste:
```bash
cd ~/eye2steering_ws
colcon build
source install/setup.bash
ros2 launch my_robot_bringup my_robot_nav.launch.xml vr:=true eeg:=muse
```
This starts the simulated store and walker, the planner, the hazard loop, the EEG decoder, rosbridge, the web server
and RViz. The walker **waits** until the operator starts it.

**B5. Open the 3D view, so other people can watch.** Open a second WSL terminal (**Terminal 2**) and paste:
```bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/eye2steering_ws/install/my_robot_description/share
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -g --render-engine ogre2
```
This opens Gazebo's 3D window on the simulation that's already running: the store and the walker moving through it,
from any angle (drag to orbit). Closing it doesn't affect the session.

Gazebo only shows the physical world, so the flickering tags and floor lines aren't drawn there. To see **exactly
what the participant sees**, also open `https://localhost:8443/spectator.html` in a PC browser (the headset mirror),
or cast the Quest (**Camera → Cast**).

**B6. Open the operator page** on the PC: `https://localhost:8443/operator.html`. It shows the EEG signal quality,
each hazard's state, and the controls in [Operator controls](#operator-controls).

**B7. Put the headset on the participant.**
1. On the Quest, open the browser and go to `https://<PC-IP>:8443`.
2. Wait for **"Connected"**, then tap **Enter VR**. (This is the only tap. Do it for the participant if needed.)
3. Seat the participant facing forward, and press **Recentre** on the operator page so the walker's handles are in
   front of them.
4. Ask them to rest their hands as if holding the walker's handles.

**B8. Check the SSVEP response** (about 1 minute). On the operator page press **SSVEP check**. A test patch flickers
in front of the participant at two frequencies in turn. The page shows PASS if the decoder detects each one. If it
fails, check the Oz electrode (B1) before continuing.

**B9. Press Start walking** on the operator page.

**B10. The participant just looks around naturally.**
- The walker drives forward. Looking somewhere for a couple of seconds steers it that way, in open space.
- Flickering patches appear on obstacles ahead, and fade if the participant's EEG shows they've seen them.
- The floor lines show where the walker is about to go.

**B11. To stop**, press **Stop walking** on the operator page (the walker stops), or **Stop flicker** (all tags
stop flickering immediately). Take the headset off, then press Ctrl+C in the WSL terminal.

**B12. Results** are saved to `~/eye2steering_ws/sessions/<date-time>/`: every hazard, when it was probed, the EEG
result, speed and clearance, and the walker's path.

---

## Trying it without a headset or Muse (testing only)

> With the Quest 3 and Muse, use
> [the step-by-step section above](#read-here-setup-and-use-instructions-for-vr-quest-3-and-muse) instead. This section
> is only for testing when one or both are missing.

### The simulation on its own (no VR, no Muse)

To run just the simulated walker in the store, with live mapping, Nav2 and the autonomy, but none of the VR or EEG
parts:
```bash
cd ~/eye2steering_ws
colcon build
source install/setup.bash
ros2 launch my_robot_bringup my_robot_nav.launch.xml
```
This opens RViz. To watch the walker in 3D as well, in a second terminal:
```bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/eye2steering_ws/install/my_robot_description/share
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -g --render-engine ogre2
```
To drive by hand instead of letting the walker drive itself, add `autonomy:=false` to the launch command, and in a
third terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

### With simulated stand-ins for the headset or Muse

Every part can be tried with simulated stand-ins.

| You have | Command | What replaces the missing part |
|---|---|---|
| Nothing | `ros2 launch my_robot_bringup my_robot_nav.launch.xml vr:=true eeg:=emulator` and open `https://localhost:8443` on the PC | The browser shows the VR scene on the desktop (look around with the mouse). Simulated EEG "sees" tags that are in view. |
| Quest, no Muse | `... eeg:=emulator` | Simulated EEG, driven by where the headset is actually looking |
| Muse, no Quest | `... eeg:=muse` and open the scene on a PC monitor | The flicker is drawn on the monitor. For SSVEP testing, use a 60 Hz or higher monitor. |

The emulator's chance of "seeing" a tag can be set, for example `emulator_detect_prob:=0.5`, to test the cautious
behaviour.

---

## Operator controls

On `https://localhost:8443/operator.html`:

| Control | Effect |
|---|---|
| **Start walking / Stop walking** | Stands in for the grip: held or released |
| **Stop flicker** | Turns off every flickering tag immediately (safety) |
| **SSVEP check** | The pre-session EEG test (B8) |
| **Recentre** | Puts the walker's handles in front of the participant |
| **EEG status** | Live signal quality: OK, poor contact, or no stream |
| **Hazard list** | Each hazard: frequency, state (waiting, probing, registered, missed) and confidence |

RViz on the PC shows the operator's view: the store, the SLAM map, the planned path, the tentacle fan, the hazards
and their states, and the floor trajectory.

---

## Settings you might change

| Setting | Default | Where | What it does |
|---|---|---|---|
| Probe start distance | 4.0 m | `config/hazards.yaml` | How far ahead a hazard starts being probed |
| Decision distance | 2.5 m | `config/hazards.yaml` | Escalate here if the hazard still isn't registered |
| Hazards probed at once | 2 | `config/hazards.yaml` | The rest wait in a queue |
| Careful speed | 40 % | `config/hazards.yaml` | Speed cap for a missed hazard |
| Extra clearance | +0.35 m | `config/hazards.yaml` | Wider berth for a missed hazard |
| Flicker frequencies | 6.43–12.86 Hz | `config/hazards.yaml` | Taken from the headset refresh rate (90 Hz) |
| Flicker contrast | subtle 0.25, strong 0.5 | `config/hazards.yaml` | Capped for photosensitivity |
| Head filter / deadband | 1.5 s / ±12° | `config/head_steering.yaml` | How long and how far the user must look to steer |
| Detection threshold | set by SSVEP check | `config/eeg.yaml` | How strong a response counts as present |

---

## Troubleshooting

| Problem | Likely cause and fix |
|---|---|
| Quest shows "This site can't be reached" | Quest and PC not on the same Wi-Fi, mirrored networking not on (A3), or firewall (A4) |
| Quest says "Connecting…" forever | The certificate for port 9090 hasn't been accepted (A7) |
| No **Enter VR** button | The page was opened with `http://` instead of `https://` |
| `eeg_check stream` finds no stream | BlueMuse isn't streaming, or mirrored networking (A3) isn't on |
| Stream found but no AUX channel | Turn on the AUX channel in the streaming tool's settings |
| `eeg_check alpha` fails | Poor Oz contact: add gel, part the hair, press the cup on firmly |
| SSVEP check fails | Oz contact, or the headset not running at 90 Hz (the operator page shows the measured refresh rate) |
| Every hazard escalates | No reliable EEG, so the walker defaults to cautious. See the EEG status on the operator page. |
| Scene stutters in the headset | Close other programs on the PC. The sim runs a lot on the CPU. |
| Participant feels sick | Stop walking, take the headset off. Reduce speed in `config/hazards.yaml` for the next session. |

---

## Repository layout

```
src/
├── my_robot_description/   robot model (URDF/xacro, meshes), LiDAR, IMU, camera
├── my_robot_bringup/       launch files, Nav2 / EKF / twist_mux config, the store world, RViz layouts
├── my_robot_autonomy/      the walker's behaviour (Python nodes)
│   ├── hazard_tracker      LiDAR → tracked hazards, relevance, probe queue, frequencies
│   ├── tentacles           fan of candidate arcs, the open ways forward
│   ├── walker_brain        per-hazard decisions → speed cap and extra clearance
│   ├── forward_goal        the walker's forward goal, biased by head direction
│   ├── trajectory_overlay  the 2 s floor display (centre and edge lines, authority)
│   ├── eeg_decoder         Muse/LSL → SSVEP present / absent per hazard
│   ├── muse_emulator       simulated EEG for testing without a Muse
│   └── eeg_check           EEG setup checks
├── walker_interfaces/      ROS messages: hazards, probe results, trajectory
└── walker_vr/              the WebXR app (headset + operator page), web server, certificates
```

---

## Status

| Part | State |
|---|---|
| Simulated walker: LiDAR, IMU, camera, EKF, Gazebo store world | ✅ done |
| Live SLAM, Nav2 with the walker's footprint, collision monitor, twist_mux | ✅ done |
| Forward driving with smooth turns; obstacle lane; two-speed logic | ✅ done (to be replaced by the parts below) |
| RViz operator view | ✅ done |
| Stop when commands stop; no recovery spins | ⏳ to build |
| Hazard tracking, relevance, probe queue, frequencies | ⏳ to build |
| Tentacle fan | ⏳ to build |
| Per-hazard walker brain (speed cap + wider berth) | ⏳ to build |
| Head-guided direction with risk-based authority | ⏳ to build |
| Floor trajectory display | ⏳ to build |
| EEG decoder, Muse emulator, EEG checks | ⏳ to build |
| WebXR headset app and operator page | ⏳ to build |
| Headset mirror page (`spectator.html`) | ⏳ to build |
| Gazebo 3D view of the running simulation (`ign gazebo -g`) | ✅ works now |
| VR option on `my_robot_nav.launch.xml` (`vr:=true eeg:=muse/emulator`), session logging | ⏳ to build |

**Future extension, manual mode.** The user pushes the walker like a conventional one, and the robot steps in only
when an obstacle is in the path, by braking one wheel more than the other, so the user's own push carries the walker
round the hazard. The robot only resists and never pushes, so it can't drive the user into anything. The same SSVEP
logic sets how strongly it intervenes. Not part of this build.

---

## Why this is humanistic intelligence

Mann defines humanistic intelligence (HI) as intelligence that arises when a human is part of the feedback loop of a
computational process, with the human and computer inextricably intertwined [Mann 1998; Mann 2001].

**The loop.** The machine senses the scene (LiDAR) and shows the human what it sensed (a tag). The human senses the
scene and the tag. The machine senses the human (the EEG response at Oz, and head direction) and acts (avoiding
gently, or escalating, and steering towards where the user faces when clearance allows). The human senses the
machine's response (the tag fading or intensifying, the trajectory bending or widening, the walker slowing) and acts
by looking where they want to go, or looking back to cancel. Neither side alone can do what the combined system
does: the LiDAR can't see what the user has perceived or where they want to go, and the user can't see everything
the LiDAR has detected.

| HI property | In this system |
|---|---|
| **Constancy** | The hazard loop and head tracking run continuously. There's no "BCI mode" to switch on. |
| **Augmentation** | Walking is the primary task. The system adds a machine sense of the scene and only interrupts when that sense and the user's perception disagree. |
| **Mediation** | The headset filters what the user sees: registered hazards fade, only missed ones are escalated. EEG is processed locally, and only "registered / not registered" per hazard reaches the motion system. |
| **Unmonopolizing** | Tags and the trajectory are anchored in the scene rather than replacing it. The flicker is subtle and there's nothing to stare at. |
| **Unrestrictive** | The user can look around and talk. Filtering and the deadband mean ordinary glances don't steer. |
| **Observable** | Tags show what the LiDAR detected. The floor display shows intended motion, extra clearance and how much control the machine has. |
| **Controllable** | The user steers by facing a direction and can cancel any previewed turn by looking back. Authority moves to the machine only as risk rises, and the shift is always shown. |
| **Attentive** | The machine combines the LiDAR, the user's perception (EEG), the user's direction of interest (head) and the grip. |
| **Communicative** | The weakest of the six. The headset is a channel from machine to human, and the EEG and head tracking are channels back. Sharing which hazards were missed with a caregiver is a future extension. |

### References
- S. Mann, "Humanistic Computing: 'WearComp' as a New Framework and Application for Intelligent Signal Processing,"
  *Proc. IEEE*, 1998.
- S. Mann, "Wearable Computing: Toward Humanistic Intelligence," *IEEE Intelligent Systems*, 2001.
- *[cite: head anticipation in human locomotion]*
- *[cite: risk-based arbitration in shared-control wheelchairs]*
