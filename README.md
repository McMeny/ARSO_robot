# Eye to Steering: a Humanistic-Intelligence Smart Walker

An assistive robotic walker that uses the **human visual system as a second sensor** alongside its LiDAR, and the
user's **head direction** to learn where they want to go. The human stays in the walker's feedback loop the whole
time. The walker adjusts how it handles each hazard depending on whether the user appears to have seen it, and the
balance of steering control between human and machine shifts with the level of risk.

This repository is the **VR evaluation build**: the walker, the store and the hazards are simulated (ROS 2 + Gazebo),
the user experiences them in a **Meta Quest 3** in full VR, and a **Muse EEG headband with an Oz electrode** reads
their visual response. The aim is to evaluate the concept before any hardware is funded.

> **Branch `tentacle-only-driving`:** this version has **no Nav2 planner and no SLAM**. A single reactive
> **tentacle planner** drives the walker, and the user **squeezes the handles to steer** - see
> [How it drives](#how-it-drives-the-tentacle-planner). Launch file: `walker_tentacle.launch.xml`.
> The Nav2 version is on `MENY_MANUAL_EXTENDED`.

> **Status: built and tested in simulation** (with the emulated Muse and a simulated headset). Not yet tried with a
> real Quest 3 and Muse - see [Status](#status).

> **HAVE A QUEST 3 AND A MUSE? [READ HERE: SETUP AND USE INSTRUCTIONS FOR VR](#read-here-setup-and-use-instructions-for-vr-quest-3-and-muse)**

---

## Contents
1. [Background](#background)
2. [How it works](#how-it-works)
3. [Safety rules](#safety-rules)
4. [The VR evaluation build](#the-vr-evaluation-build)
5. [**READ HERE: SETUP AND USE INSTRUCTIONS FOR VR (QUEST 3 AND MUSE)**](#read-here-setup-and-use-instructions-for-vr-quest-3-and-muse)
6. [Trying it without a headset or Muse (testing only)](#trying-it-without-a-headset-or-muse-testing-only)
7. [Tentacle planner reference: topics, parameters, scenarios](#tentacle-planner-reference-topics-parameters-scenarios)
8. [Operator controls](#operator-controls)
9. [Settings you might change](#settings-you-might-change)
10. [Troubleshooting](#troubleshooting)
11. [Repository layout](#repository-layout)
12. [Status](#status)
13. [Why this is humanistic intelligence](#why-this-is-humanistic-intelligence)

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
holding both handles. **The grip decides whether it moves at all**:

| Grip | State | What the walker does |
|---|---|---|
| Either hand off (for more than 0.2 s) | **STOP** | Stops and holds. Nothing else can make it move. |
| Both hands holding | **WALK** | Drives on the way it is going: straight unless an obstacle forces a curve. It never turns more than 45° on its own, so at a junction or a dead end it stops and waits for the user. |
| Both hands squeezing firmly (for 0.5 s) | **STEER** | **Squeeze to steer**: it heads where the user's head points, turning on the spot if the head is turned far round. |

A sudden squeeze shorter than 0.5 s (a startle) is ignored. On the real walker the handles have force sensors (FSRs)
read by an Arduino, which also stops the motors itself in firmware. In VR, the Quest controllers' **grip triggers**
stand in for them.

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

### How it drives (the tentacle planner)

There's no map and no route planner. A single reactive **tentacle planner** (after von Hundelshausen et al., 2008,
*Driving with Tentacles*) runs 20 times a second:

1. **31 tentacles**, constant-curvature arcs from hard left (radius 0.67 m) through straight to hard right, 2.5 m
   long, **plus 20 lane-change tentacles**: S-shapes that curve one way then back, ending parallel to where they
   started but shifted sideways. Those let it step round an obstacle and carry on, which matters because the walker
   with the user behind it is 1.25 m long and has to pass things nearly parallel.
2. Each one is checked by **sweeping a footprint that covers the walker *and* the user behind it** (1.25 × 0.6 m,
   plus a 12 cm safety margin) along it against the LiDAR. It counts as valid if it stays clear for at least 0.7 m,
   enough to stop. Moving away from something that's already close is always allowed.
3. Among the valid ones it picks the lowest cost:
   ```
   cost = clearance (keep away from things)  +  clear length (prefer open ways)
        + head direction x authority (STEER only)  +  hazard berths
        + hysteresis (stay on the previous arc: no jitter)  +  curvature (prefer straight)
        + WALK only: back to the heading WALK started with (after going round something)
   ```
4. **Speed** = 0.4 m/s × (room ahead) × (0.5 if an unregistered or no-EEG hazard is within 2 m ahead), with smooth
   acceleration.
5. **Blocked** (no valid tentacle): it stops, and after 1 s the headset shows **"Path blocked. Squeeze to steer
   out"**. Squeezing and turning the head turns it on the spot (if the turn is clear) and drives it out.

After the planner the command goes through the Nav2 **velocity smoother** and then the Nav2 **collision monitor**,
an independent safety layer that stops or slows the walker by itself if anything gets inside its zones. The planner
keeps further away than those zones, so in normal driving the collision monitor never has to act.

```
tentacle_planner ─► velocity_smoother ─► twist_mux (teleop wins) ─► collision_monitor ─► walker
```

**What it can't do:** plan a route round a whole shelf to reach somewhere specific. It reacts to what's in front, and
the user chooses directions at junctions by squeezing to steer.

### The trajectory display

The headset draws the walker's short-term path on the floor as faint, steady lines (no flicker, no animation): a
**centre line and two edge lines** marking the walker's width over the next **2 seconds**.
- The edges **widen and brighten** (orange) where the walker gives an unregistered hazard a wider berth.
- The edges become **more solid** as the machine takes more control.
- **Turning on the spot:** the lines show the walker's outline where the turn will end.
- **Blocked:** a short **amber** outline just in front of the walker.

### Head-guided direction (squeeze to steer)

The head only steers in **STEER** (both handles squeezed firmly). In WALK the walker ignores the head, so looking
around never moves it. In STEER, steering authority is **shared according to risk**, measured as clearance: the
distance from the walker-and-user footprint to the nearest obstacle.

| Clearance around the walker | Who steers (STEER) |
|---|---|
| **0.8 m or more** | The user's head fully: the walker picks the clear tentacle that ends up facing where they look |
| **Between** | The head's influence fades as clearance shrinks |
| **0.15 m or less** | The machine fully. The head can't steer it into anything: only clear tentacles can ever be chosen |

- Head turned more than 60° away (or 20° while blocked): the walker slows to a stop and **turns on the spot** to
  match, if the turning footprint is clear, and stops turning within 10° of the head direction.
- Head direction is low-pass filtered (0.6 s), and looks towards a tagged hazard are ignored: those are the hazard
  probe at work, not a choice of direction.
- **Hint:** if the user keeps looking more than 30° away from where the walker is going for 3 s in WALK (and no
  hazard is in caution), the headset shows "Turn left? Squeeze to steer" (or right, or "Turn around?"). At most once
  every 30 s, and no more after 3 successful steers.

### Speed: the lowest limit always wins

```
actual speed = lowest of ( grip ,  planner (room ahead, hazard caution) ,  velocity smoother ,  collision monitor )
```

| Grip | Hazard nearby? | EEG result | Result |
|---|---|---|---|
| WALK / STEER | none | – | normal pace, 0.4 m/s |
| WALK / STEER | yes | registered | normal pace, normal berth (0.15 m) |
| WALK / STEER | yes | not registered, or no reliable EEG | half pace, wider berth (0.45 m), stronger cue |
| STOP | – | – | stop and hold |

The EEG result can only lower the speed and widen the berth. It never chooses a direction or starts motion.

---

## Safety rules

These hold in every mode, and nothing in the software can switch them off.

1. **The EEG never commands motion.** It only changes speed scaling and the hazard berth.
2. **No motion unless the grip is WALK or STEER.** STOP means zero and hold. No grip readings, no LiDAR, or the
   operator's Stop walking also mean STOP.
3. **The collision monitor is an independent safety layer**, last in the command chain, after the planner and the
   velocity smoother. The planner is never the only thing preventing a collision.
4. **The head only steers in STEER**, and its weight drops as clearance drops.
5. **Missing or uncertain EEG means cautious behaviour.**
6. **No surprise motion.** The walker never reverses and never turns on the spot on its own: turning on the spot only
   happens in STEER, towards where the user is looking, and only if the turning footprint is clear.
7. **Speed is capped** at 0.4 m/s.
8. **VR room scale:** near the Guardian edge, or too far from the handles, the walker stops ("Room edge reached"),
   and stays stopped until the user lets go and grips again.
9. **Photosensitivity:** flicker is limited to **6–13 Hz** (avoiding roughly 15–25 Hz), flicker contrast is capped,
   and the operator can stop all flicker instantly. This build assumes participants don't have photosensitive
   epilepsy. A formal study will usually need screening added.

---

## The VR evaluation build

Testing in VR changes some things compared with the real walker. These are deliberate, and they're listed here so
the results are read correctly.

| Real walker | VR evaluation build |
|---|---|
| Passthrough mixed reality: tags on real objects | **Full VR**: the user sees a virtual store at 1:1 scale, and tags are placed on virtual objects |
| Headset-to-map calibration needed | **Recentre** puts the sim walker's handles at the user's hands |
| User walks behind the walker | **Room scale** (default): the user walks for real in a cleared room, alongside the sim walker. **Stationary**: the user stands still and the view moves with the walker. Eye height is always the user's real one. |
| Handle FSRs (Arduino) | The Quest controllers' **grip triggers** |
| LiDAR, motors, real store | Simulated in Gazebo: LiDAR, IMU, wheel odometry, a 20 × 14 m store with aisles and clutter |

**Known limitations of the VR build:**
- **EEG while walking.** In room scale the user really walks, which is a fair test of movement artifacts. In
  stationary mode they don't, so stationary results overstate how clean the EEG will be.
- **Motion sickness.** In stationary mode the view moves while the body doesn't. The walker moves slowly, the view
  is never turned for the user, and there's a comfort vignette, but some people may still feel it. Room scale avoids
  this.
- **Room size.** Room scale needs a cleared space of about 3 × 5 m. Use **Reset** (hold A or X for 1 s) to re-align
  the sim world when you reach the edge, then turn round and carry on.
- **What SSVEP measures.** A present response is evidence that the tag reached the user's visual cortex. Whether that
  predicts *noticing* the hazard, especially in peripheral vision, is something the experiments are meant to test.
- **Only two hazards are probed at once.** A single Oz electrode can only separate a few frequencies, and only
  frequencies **outside the alpha band (about 8–12 Hz)** work reliably: near alpha the response is buried in the
  brain's own rhythm. In simulated tests, 7.5 and 12.86 Hz were detected 93–95 % of the time with 0–2 % false alarms,
  against 28 % at 11.25 Hz. So tags use 7.5, 12.86, 6.92 and 6.43 Hz, at most two at a time. The store can hold any
  number of obstacles (frequencies are reused once a hazard is registered or passed), but a third hazard on the path
  waits in the queue, and if it reaches the decision distance before a frequency frees up it is treated as missed
  (cautious). Phase coding (several hazards on one frequency) was tested and rejected: it needs timing accurate to
  about 10 ms, which the Muse's Bluetooth link can't guarantee.

### How the pieces connect

```
                 Windows PC                                   WSL2 (Ubuntu 22.04, ROS 2 Humble)
 ┌──────────────────────────────────┐        ┌──────────────────────────────────────────────────────┐
 │ Muse ──Bluetooth──► BlueMuse /   │  LSL   │  EEG decoder ──► probe results ──► hazard loop        │
 │                     muselsl ─────┼───────►│                                       │               │
 └──────────────────────────────────┘        │  Gazebo (walker + store) ◄─ collision monitor ◄─ ...  │
                                             │      │ LiDAR          ... velocity smoother ◄─┐       │
 ┌──────────────────────────────────┐  Wi-Fi │      └──► hazard tracker ──► tentacle planner         │
 │ Quest 3 browser (WebXR app)      │◄──────►│  web bridge (https + wss :8443) ──► grip state ┘      │
 │  draws store, walker, tags,      │        │     head pose, grip triggers, safety stop             │
 │  floor lines, hint/notice; sends │        │  RViz (operator view)                                │
 │  head pose + grip triggers       │        └──────────────────────────────────────────────────────┘
 └──────────────────────────────────┘
```

- The Muse connects by Bluetooth, which **WSL2 can't use**, so it streams from Windows over LSL.
- The Quest's browser loads the VR app from the PC, and talks to ROS through the walker's web bridge, over the same
  port (8443).
- Gazebo simulates the walker. The headset only *draws* the scene, rebuilt from the same world file.

---

## READ HERE: SETUP AND USE INSTRUCTIONS FOR VR (QUEST 3 AND MUSE)

This is the full setup, with the real headset and the real EEG.

### What you need
- A Windows 10/11 PC with **WSL2 (Ubuntu 22.04)** and **ROS 2 Humble**, with this repository built
- A **Meta Quest 3** on the **same Wi-Fi network** as the PC
- A **Muse 2 or Muse S** with an **Oz cup electrode** on its AUX input (Ag/AgCl, with gel), as in Eye as a Camera
- **BlueMuse** (or Python `muselsl`) on Windows
- Both **Quest Touch controllers** (their grip triggers are the walker's handles)
- A **cleared space of about 3 × 5 m** for room scale, with the Quest **Guardian** set up round it (or a spot to
  stand for stationary mode)

### Part A: one-time setup (about 30 minutes)

**A1. Install the EEG library** (in WSL). Nothing else is needed: the web bridge is built in.
```bash
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
New-NetFirewallRule -DisplayName "Walker VR web"       -Direction Inbound -Protocol TCP -LocalPort 8443 -Action Allow
New-NetFirewallRule -DisplayName "Walker LSL (TCP)"    -Direction Inbound -Protocol TCP -LocalPort 16571-16604 -Action Allow
New-NetFirewallRule -DisplayName "Walker LSL (UDP)"    -Direction Inbound -Protocol UDP -LocalPort 16571-16604 -Action Allow
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

**A5. Make the HTTPS certificate.** The Quest only allows VR on secure (https) pages. In WSL:
```bash
cd ~/eye2steering_ws
source install/setup.bash
ros2 run walker_vr make_certs
```
It prints the addresses the certificate covers. If your PC's Wi-Fi address (A6) isn't listed, run it again with
that address, e.g. `ros2 run walker_vr make_certs 192.168.1.42`.

**A6. Find the PC's IP address.** In PowerShell, run `ipconfig` and note the **IPv4 address** of your Wi-Fi
adapter, for example `192.168.1.42`. You'll type it into the Quest. Below it's written as `<PC-IP>`.

**A7. Let the Quest trust the certificate** (once per certificate). Start the system once (B4), then in the Quest
browser open `https://<PC-IP>:8443` and choose **Advanced → Proceed** on the warning. Do the same once in the PC's
browser for `https://localhost:8443`.

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
ros2 launch my_robot_bringup walker_tentacle.launch.xml vr:=true eeg:=muse
```
This starts the simulated store and walker, the tentacle planner, the grip state (from the Quest's grip triggers),
the hazard loop, the EEG decoder, the web bridge and RViz (for the operator). Options:
- `vr_mode:=stationary` if there's no room to walk (default `room_scale`)
- `gazebo_view:=true` to also open the Gazebo 3D window (B5)

The walker **won't move** until the operator presses Start walking **and** the participant grips both handles.

**B5. Let other people watch, live.** While the participant is in VR, anyone at the PC can watch the walker moving
and operating **in real time, as the participant experiences it**. Each view joins the running session, and closing
one doesn't affect anything.

**Watch in RViz**: the robot moving and thinking: the tentacle fan coloured by cost, the chosen trajectory and its
edges, the hazards and their probe states, and the collision monitor's zones. Open a new WSL terminal and paste:
```bash
source ~/eye2steering_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/eye2steering_ws/install/my_robot_bringup/share/my_robot_bringup/rviz/walker_tentacle.rviz --ros-args -p use_sim_time:=true
```

**Watch in Gazebo 3D**: the store and the walker moving through it, from any angle (drag to orbit). Open a new WSL
terminal and paste:
```bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/eye2steering_ws/install/my_robot_description/share
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -g --render-engine ogre2
```

**Watch through the participant's eyes**: the exact headset view, with the flickering tags and floor lines (which
RViz and Gazebo don't draw). Open `https://localhost:8443/spectator.html` in a PC browser, or cast the Quest
(**Camera → Cast**).

**B6. Open the operator page** on the PC: `https://localhost:8443/operator.html`. It shows the EEG signal quality,
each hazard's state, and the controls in [Operator controls](#operator-controls).

**B7. Put the headset on the participant.**
1. **Room scale:** stand the participant at one end of the cleared space, facing along its long side.
2. On the Quest, open the browser and go to `https://<PC-IP>:8443`.
3. Wait for **"Connected"**, then tap **Enter VR**.
4. Give them both controllers. Ask them to hold them where the walker's handles would be, about 40 cm in front of
   them at hip height. After 1 s the view **recentres** automatically: the sim walker's handles appear at their hands,
   facing the way they face. To redo it, hold **A or X for 1 s** (or press **Recentre** on the operator page).

**B8. Check the SSVEP response** (about 1 minute). On the operator page press **SSVEP check**. A test patch flickers
in front of the participant at two frequencies in turn. The page shows PASS if the decoder detects each one. If it
fails, check the Oz electrode (B1) before continuing.

**B9. Press Start walking** on the operator page.

**B10. Tell the participant:**
- **Hold both grip triggers lightly to walk.** The walker drives forward slowly; walk along with it.
- **Let go with either hand to stop.**
- **Squeeze both firmly and look where you want to go, to steer.** Look far round and it turns on the spot.
- Flickering patches appear on obstacles ahead, and fade if the EEG shows they've seen them.
- The floor lines show where the walker is about to go.
- If the walker stops at a dead end or a junction, the headset says **"Path blocked. Squeeze to steer out"**.
- Room scale: near the edge of the space it stops and says **"Room edge reached"**. Let go, hold **A or X for 1 s**
  to reset, turn round, and carry on.

**B11. To stop**, the participant lets go, or press **Stop walking** on the operator page, or **Stop flicker** (all
tags stop flickering immediately). Take the headset off, then press Ctrl+C in the WSL terminal.

**B12. Results** are saved to `~/eye2steering_ws/sessions/<date-time>/`, timestamped: every hazard and its states, the
EEG results, grip and steering changes (`events.jsonl`), and 10 times a second the walker pose, head pose, grip
state, authority, turning in place, blocked and caution (`path.csv`).

---

## Trying it without a headset or Muse (testing only)

> With the Quest 3 and Muse, use
> [the step-by-step section above](#read-here-setup-and-use-instructions-for-vr-quest-3-and-muse) instead. This section
> is only for testing when one or both are missing.

### The simulation on its own (no VR, no Muse)

The simulated walker in the store, with the keyboard as the handles. **Terminal 1:**
```bash
cd ~/eye2steering_ws
colcon build
source install/setup.bash
ros2 launch my_robot_bringup walker_tentacle.launch.xml
```
**Terminal 2** (the handles, and a pretend head):
```bash
source ~/eye2steering_ws/install/setup.bash
ros2 run my_robot_autonomy walker_keys
```
`w` both hands on (WALK) · `s` squeeze (STEER) · `space` let go (STOP) · `l` / `r` one hand · `a` / `d` turn the head
30° left / right · `f` head straight · `e` "the user noticed it" (with `eeg:=toggle`) · `q` quit.

To watch in 3D as well, in another terminal:
```bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/eye2steering_ws/install/my_robot_description/share
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -g --render-engine ogre2
```
To drive by hand instead (teleop beats the planner):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

### With simulated stand-ins for the headset or Muse

| You have | Launch options | What replaces the missing part |
|---|---|---|
| Nothing | `vr:=true grip:=keyboard eeg:=model`, then open `https://localhost:8443` on the PC (`http://` without the certificate, A5) | The browser shows the VR scene on the desktop. Drag to look; hold **W** for hands on, **W + Space** to squeeze. `ssvep_emulator` decides what the "user" registers. |
| Quest, no Muse | `vr:=true eeg:=emulator` | Simulated raw EEG through the real decoder, driven by where the headset is actually looking |
| Muse, no Quest | `vr:=true grip:=keyboard eeg:=muse` and open the scene on a PC monitor | The flicker is drawn on the monitor. For SSVEP testing, use a 60 Hz or higher monitor. |

EEG stand-ins: `eeg:=model` (each probed hazard registered after 2–4 s with probability `p_detect:=0.8`),
`eeg:=toggle` (registered only when you press `e` in `walker_keys`), `eeg_valid:=false` (no reliable EEG: everything
cautious), `eeg:=emulator` (simulated raw EEG, real decoder), `eeg:=muse` (the real thing).

---

## Tentacle planner reference: topics, parameters, scenarios

### Nodes and topics

| Node | In | Out |
|---|---|---|
| `grip_state_node` | `/walker/grip_raw` (Quest), `/walker/grip_keys` (keyboard), serial (Arduino), `/walker/force_stop`, `/walker_enabled` | `/walker/grip_state` (GripState, 50 Hz) |
| `tentacle_planner_node` | `/scan`, `/odometry/filtered`, `/walker/grip_state`, `/head_pose`, `/hazards` | `/cmd_vel_tentacle`, `/walker/trajectory`, `/walker/trajectory_left`, `/walker/trajectory_right`, `/walker/tentacles_debug`, `/walker/steering_status` (20 Hz); also `/plan` (for the hazard tracker), `/walker_status` |
| `velocity_smoother` (Nav2) | `/cmd_vel_tentacle` | `/cmd_vel_smoothed` |
| `twist_mux` | `/cmd_vel_smoothed`, `/cmd_vel_joy` (teleop, wins) | `/cmd_vel_mux` |
| `collision_monitor` (Nav2) | `/cmd_vel_mux`, `/scan` | `/cmd_vel_out` (the walker) |
| `hazard_tracker` | `/scan`, `/plan`, `/probe_results`, `/eeg_status` | `/hazards` (with `registered`, `eeg_valid`) |
| `ssvep_emulator` / `eeg_decoder` | hazards / the Muse over LSL | `/probe_results`, `/eeg_status` |
| `web_server` | the headset | `/head_pose`, `/walker/grip_raw`, `/walker/force_stop`; sends trajectory, steering status, hazards to the headset |

Messages (`walker_interfaces`): `GripState` (left, right, state STOP / WALK / STEER), `SteeringStatus` (grip state,
authority 0 = machine … 1 = user, desired yaw, turning in place, blocked, caution, hint, notice), `Hazard` /
`HazardArray`, `ProbeResult`.

### Parameters

All in `src/my_robot_bringup/config/`:

| File | What's in it |
|---|---|
| `tentacle_planner.yaml` | tentacles (count, k_max, lengths), footprint (walker + user) and safety margin, cost weights, berths, authority (c_low / c_high), speed and accelerations, caution, turning in place, hint and notice timing |
| `grip.yaml` | source, light / firm thresholds, hysteresis, release grace, steer hold, one-handed mode, serial port |
| `velocity_smoother.yaml` | max velocity [0.4, 0, 0.8], acceleration 0.3 m/s² (braking 0.5), 0.8 rad/s², open loop |
| `collision_monitor.yaml` | stop zone (walker body + 8 cm), slowdown zone (0.5 m ahead, half speed) |
| `twist_mux_tentacle.yaml` | planner vs teleop priority |

**Where the planner differs from the brief, and why** (all are parameters):
- **Tentacle length 2.5 m** (brief: `v × 2 s` clamped to 1.0–2.5 m). At 0.4 m/s that is always 1 m, which leaves no
  room to swerve gently round an obstacle. The floor display still shows the next 2 s.
- **Lane-change (S-shaped) tentacles** as well as the brief's arcs. With arcs alone, a walker this long can't get
  past an obstacle in an aisle: every arc that swerves round it stays angled, and the footprint is then too wide for
  the gap.
- **A tentacle is valid if it's clear for 0.7 m**, not its whole length. Constant-curvature arcs that swerve round
  something in an aisle keep curving into the aisle wall later, so whole-length validity rules out every swerve. A
  shorter clear length costs more, and the choice is re-made 20 times a second.
- **Hazard berth is a linear hinge**, `(berth − gap) / berth_registered`, not a plain Gaussian. With a wider Gaussian
  the cost is *flatter* close to the hazard, so the "wider berth" pushed less. The hinge has the same slope for both
  berths and the wide one reaches further, so it is never weaker. It is bounded, so it can't shove the walker into
  another obstacle. (A squared hinge was tried and did exactly that in clutter.)
- **Heading is judged 0.6 m and 1.2 m along each tentacle**, not at its end. Only the start of a tentacle is ever
  driven before re-planning: judged by its end, an S-curve that first swings the wrong way looked free (the walker
  turned 66° for a 45° look), and turns towards the head were sluggish.
- **WALK keeps its heading** (returns to it after going round something) and never turns more than 45° on its own.
- **Safety margin 12 cm**, more than the collision monitor's 8 cm stop zone, so the collision monitor never has to act
  in normal driving.
- The collision monitor's zones cover the **walker body only**, not the user: on the real walker the LiDAR would see
  the user's legs and stop for ever.
- **Velocity smoother in open loop** (brief: odom feedback). In closed loop it brakes relative to the measured speed,
  which lags, so stopping took 2.7 s and 0.58 m instead of about 1 s and 0.2 m.
- The sim drives through Gazebo's DiffDrive plugin (`/cmd_vel_out`), not `diff_drive_controller`.
  `ign_ros2_control` isn't installed here.

### Scenarios

Each scenario is its own world. Run it in one terminal and the automatic check in another: it acts as the user
(grip and head), measures what the walker does, and prints PASS or FAIL. The log of every tick goes to
`~/eye2steering_ws/sessions/scenarios/<name>.json`.
```bash
ros2 launch my_robot_bringup walker_tentacle.launch.xml scenario:=corridor
```
```bash
source ~/eye2steering_ws/install/setup.bash
ros2 run my_robot_autonomy run_scenario corridor
```

| # | Check | `scenario:=` (world) | `run_scenario` | Expected |
|---|---|---|---|---|
| 1 | Open space | `open` | `open` | WALK straight; STEER + head 45° curves to it; STEER + 180° turns on the spot |
| 2 | Corridor | `corridor` | `corridor` | Centred, steady, no oscillation |
| 3 | Aisle end / T-junction | `tjunction` | `tjunction` | WALK doesn't turn on its own; STEER turns towards the head |
| 4 | Obstacle, registered | `obstacle` | `registered` | Avoids at normal speed, normal berth |
| 5 | Obstacle, unregistered | `obstacle eeg_valid:=false` | `unregistered` | Half speed, visibly wider berth |
| 6 | Cluttered | `cluttered` | `cluttered` | No collisions; collision monitor never needed |
| 7 | Dead end | `deadend` | `deadend` | Stops, blocked; notice after ~1 s; STEER + turned head turns it out; notice clears when STEER starts |
| 8 | Grip release | `open` | `grip_release` | Stops within the deceleration limit and holds; one hand off = STOP |

Scenarios 9 (room scale) and 10 (stationary) are checked in the headset: after recentre the handles are at the
controllers and the eye height is the user's own; walking along stays aligned; near the Guardian edge it stops with
"Room edge reached"; reset re-aligns; in stationary the view follows the walker without ever being turned for the
user, and the vignette shows while moving.

**Unit tests** (geometry, collision, edge lines, cost ordering, authority, grip states, hint and notice timing):
```bash
cd ~/eye2steering_ws/src/my_robot_autonomy && python3 -m pytest test
```

### Swapping the stand-ins for the real hardware

| Stand-in | Real | How |
|---|---|---|
| Quest grip triggers (`grip:=quest`) or keyboard (`grip:=keyboard`) | Handle FSRs on an Arduino | `grip:=arduino`. The Arduino sends one line per reading, `left,right\n`, raw ADC counts (0–1023). Set `serial_port` in `grip.yaml`. Calibrate per user: `ros2 service call /grip_state/calibrate_relaxed std_srvs/srv/Trigger` while they hold relaxed, then `.../calibrate_firm` while they squeeze. |
| `eeg:=model` / `toggle` / `emulator` | The Muse | `eeg:=muse` (Part B). Both publish the same `/probe_results`, so nothing else changes. |

---

## Operator controls

On `https://localhost:8443/operator.html`:

| Control | Effect |
|---|---|
| **Start walking / Stop walking** | Allows or forbids motion. The walker also needs the participant's grip. |
| **Stop flicker** | Turns off every flickering tag immediately (safety) |
| **SSVEP check** | The pre-session EEG test (B8) |
| **Recentre** | Puts the walker's handles at the participant's hands (same as holding A or X for 1 s) |
| **Walker status** | State (normal, careful, turning, blocked, stopped), grip state, and any message shown to the participant |
| **EEG status** | Live signal quality: OK, poor contact, or no stream |
| **Hazard list** | Each hazard: frequency, state (waiting, probing, registered, missed) and confidence |

To watch the session in RViz, in Gazebo 3D, or through the participant's eyes, see B5.

---

## Settings you might change

| Setting | Default | Where | What it does |
|---|---|---|---|
| Walking speed | 0.4 m/s | `config/tentacle_planner.yaml` (`v_max`) and `velocity_smoother.yaml` | Normal pace |
| Caution speed | 50 % | `tentacle_planner.yaml` (`caution_scale`) | With an unregistered / no-EEG hazard within 2 m ahead |
| Hazard berth | 0.15 m registered, 0.45 m not | `tentacle_planner.yaml` | How far the walker's side keeps from a hazard |
| Safety margin | 0.12 m | `tentacle_planner.yaml` | Around the walker-and-user footprint |
| Head authority | full at 0.8 m clearance, none at 0.15 m | `tentacle_planner.yaml` (`c_high`, `c_low`) | Risk-scaled steering in STEER |
| Turn on the spot | head > 60° away (20° when blocked) | `tentacle_planner.yaml` | When STEER pivots instead of curving |
| WALK turn limit | 45° | `tentacle_planner.yaml` (`walk_max_turn_deg`) | Beyond this WALK stops and waits for STEER |
| Grip thresholds | light 0.2, firm 0.6, hold 0.5 s, grace 0.2 s | `config/grip.yaml` | WALK / STEER / STOP |
| VR mode | room scale | launch option `vr_mode:=stationary` | Walk in a room, or stand still |
| Room safety | 0.5 m from the Guardian edge, 0.8 m from the handles | `walker_vr` web server parameters | Forces STOP |
| Probe start distance | 4.0 m | `config/hazards.yaml` | How far ahead a hazard starts being probed |
| Decision distance | 2.5 m | `config/hazards.yaml` | Escalate here if the hazard still isn't registered |
| Hazards probed at once | 2 | `config/hazards.yaml` | The rest wait in a queue |
| Flicker frequencies | 7.5, 12.86, 6.92, 6.43 Hz | `config/hazards.yaml` | From the 90 Hz refresh rate, outside the alpha band (see limitations) |
| Flicker contrast | subtle 0.25, strong 0.5 | `config/hazards.yaml` | Capped for photosensitivity |
| Detection threshold | 3.5 (the SSVEP check resets it, never below 3.0) | `config/eeg.yaml` | How strong a response counts as present, for 3 windows in a row |
| Simulated user | notices 80 % of hazards | launch option `p_detect:=` | With `eeg:=model` or `eeg:=emulator` |

---

## Troubleshooting

| Problem | Likely cause and fix |
|---|---|
| Quest shows "This site can't be reached" | Quest and PC not on the same Wi-Fi, mirrored networking not on (A3), or firewall (A4) |
| Quest says "Connecting…" forever | The system isn't running (B4), or the page was opened before it started: reload |
| PC browser warns about the page | Expected with the self-signed certificate: choose Advanced → Proceed (A7) |
| Walker doesn't move after Start walking | The participant must hold **both** grip triggers. Check the grip state on the operator page. Or the system is still starting (about 10 s after launch). |
| Walker stops with "Room edge reached" | Near the Guardian edge, or the participant is too far from the handles. Let go, hold A or X for 1 s to reset. |
| "Path blocked. Squeeze to steer out" | Nothing ahead is clear (dead end, junction). Squeeze both triggers firmly and look the way to go. |
| No **Enter VR** button | The page was opened with `http://` instead of `https://` |
| `eeg_check stream` finds no stream | BlueMuse isn't streaming, or mirrored networking (A3) isn't on |
| Stream found but no AUX channel | Turn on the AUX channel in the streaming tool's settings |
| `eeg_check alpha` fails | Poor Oz contact: add gel, part the hair, press the cup on firmly |
| SSVEP check fails | Oz contact, or the headset not running at 90 Hz (the operator page shows the measured refresh rate) |
| Every hazard escalates | No reliable EEG, so the walker defaults to cautious. See the EEG status on the operator page. |
| Scene stutters in the headset | Close other programs on the PC. The sim runs a lot on the CPU. |
| Participant feels sick | Stop walking, take the headset off. Use room scale, or lower `v_max` in `config/tentacle_planner.yaml` and `velocity_smoother.yaml`. |

---

## Repository layout

```
src/
├── my_robot_description/   robot model (URDF/xacro, meshes), LiDAR, IMU, camera
├── my_robot_bringup/       launch files (walker_tentacle.launch.xml; the Nav2 / SLAM ones are kept), configs, the store
│                           and scenario worlds (worlds/scenarios, made by make_scenarios.py), RViz layouts
├── my_robot_autonomy/      the walker's behaviour (Python nodes)
│   ├── grip_state_node     handles → STOP / WALK / STEER (Quest, keyboard, Arduino FSRs, operator)
│   ├── tentacle_planner_node  the tentacle planner (maths in tentacle_core.py)
│   ├── hazard_tracker      LiDAR → tracked hazards, relevance, probe queue, frequencies
│   ├── ssvep_emulator      stand-in SSVEP verdicts (model / toggle)
│   ├── eeg_decoder         Muse/LSL → SSVEP present / absent per hazard; the SSVEP check
│   ├── muse_emulator       simulated raw EEG for testing without a Muse
│   ├── eeg_check           EEG setup checks
│   ├── walker_keys         keyboard handles and head, for desktop testing
│   ├── run_scenario        automatic scenario checks
│   ├── session_logger      records each session to sessions/
│   ├── world_markers       the store drawn in RViz
│   ├── test/               unit tests (pytest)
│   └── older nodes (my_robot_nav.launch.xml): tentacles, tentacle_driver, walker_brain, trajectory_overlay
├── walker_interfaces/      ROS messages: grip state, steering status, hazards, probe results, trajectory, tentacles
└── walker_vr/              the web bridge (HTTPS + WebSocket), certificates, and the pages:
    └── web/                index.html (headset), spectator.html, operator.html
```

---

## Status

| Part | State |
|---|---|
| Simulated walker: LiDAR, IMU, camera, EKF, Gazebo store world | ✅ done |
| Tentacle planner (no Nav2 planner, no SLAM), velocity smoother, collision monitor, twist_mux | ✅ done, tested in sim (8 scenarios) |
| Grip state: STOP / WALK / STEER, squeeze to steer, startle and slip protection, calibration | ✅ done, unit tested and tested through the web bridge |
| Head steering in STEER with risk-scaled authority, turning on the spot, blocked notice, hint | ✅ done, tested in sim |
| Hazard tracking, probe queue, frequencies, `registered` / `eeg_valid` | ✅ done, tested in sim |
| SSVEP emulator (model / toggle), EEG decoder, Muse emulator, EEG checks | ✅ done, tested with the emulators |
| Web bridge, headset app (grip triggers, room scale / stationary, Guardian stop, hint and notice text), operator page | ✅ done, tested with a simulated headset |
| Session logging (head, walker, grip, steering, hazards, timestamped) | ✅ done |
| **With a real Quest 3** (WebXR, grip triggers, 90 Hz flicker, recentre, Guardian bounds, comfort) | ⏳ not tried yet (scenarios 9 and 10) |
| **With a real Muse + Oz** (BlueMuse/muselsl over LSL from Windows, real SSVEP) | ⏳ not tried yet |
| **With the real handle FSRs** (Arduino) | ⏳ not tried yet |

**Test results** (Gazebo, automatic checks with `run_scenario`, one clean run of all eight):

| # | Scenario | Result | Measured |
|---|---|---|---|
| 1 | Open space | ✅ 5/5 | WALK: 0.0 cm drift; STEER 45°: reached 42.9° (it settles within ~3°); 180°: turned on the spot, ended 7° off |
| 2 | Corridor 1.5 m | ✅ 3/3 | 0.3 cm off centre, 0.1° heading wobble |
| 3 | T-junction | ✅ 4/4 | WALK stopped at the T without turning (blocked); STEER took the left branch, 88.7° |
| 4 | Obstacle, registered | ✅ 4/4 | Passed at full speed (0.40 m/s), 21.6 cm from it |
| 5 | Obstacle, no EEG | ✅ 4/4 | Half speed (0.20 m/s), 34.1 cm from it (wider berth) |
| 6 | Cluttered | ✅ 3/3 | Through 5 obstacles in 36 s, never blocked, closest 23.7 cm (walker), 18.8 cm (user) |
| 7 | Dead end | ✅ 7/7 | Stopped 0.65 m from the wall; notice after 1.00 s; cleared the moment STEER started; turned round and walked out |
| 8 | Grip release | ✅ 4/4 | From 0.40 m/s: stopped in 1.15 s and 0.29 m, then held (0.1 cm drift); one hand off = STOP |
| | **All** | **34/34** | **No contacts in any scenario. The collision monitor never had to act.** |

Rates: planner 19.8 Hz and grip state 49.6 Hz in wall-clock time (20 and 50 Hz in sim time; Gazebo runs just under
real time). The trajectory, its edges, the steering status, the hazards and the VR config all reach the headset
through the web bridge (about 20 messages a second). Unit tests: 27 pass.

**Known limitations, from testing in simulation:**
- **The walker and the user need a gap of about 0.85 m** (0.6 m wide, plus 12 cm either side), and a long one: with
  the user it is 1.25 m long, so it passes things nearly parallel. In a 2.4 m aisle with a box in the middle (1.05 m
  either side) it gets blocked and asks the user to steer. Clutter has to be about 3 m apart to weave through.
- In WALK it never turns more than 45° by itself, so it stops at junctions and dead ends until the user steers. This
  is deliberate.
- Which objects count as hazards comes from the world file's names, standing in for an object classifier.
- The emulated EEG is a simplified model (3 µV SSVEP in background noise). Real responses vary from person to person,
  which is why the SSVEP check sets the threshold per participant.

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
