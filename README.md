# Eye to Steering: a Humanistic-Intelligence Smart Walker

An assistive robotic walker that **the user drives, and the walker protects**. The user pushes the handles and the
walker moves with them, like a power-assisted rollator. It uses the **human visual system as a second sensor**
alongside its LiDAR: an EEG measure tells it whether the user has seen each hazard, and it steps in - braking, or
bending the user's path round an obstacle - earlier and more firmly for the hazards they missed. It never moves on
its own and never pushes the user anywhere.

This repository is the **VR evaluation build**: the walker, the store and the hazards are simulated (ROS 2 + Gazebo),
the user experiences them in a **Meta Quest 3** in full VR, and a **Muse EEG headband with an Oz electrode** reads
their visual response. The aim is to evaluate the concept before any hardware is funded.

> **Branch `assist-guardian`** combines the best of two branches: **ASSIST** and the **dead-man grip** from `main`
> (the walker moves only when pushed), and the **tentacle machinery and EEG hazard loop** from
> `tentacle-only-driving` (which now *protect* the user's own path instead of driving). No Nav2 planner, no SLAM.
> Launch file: `walker_assist.launch.xml`. See [How it moves](#how-it-moves-assist-and-the-guardian).
> The other branches are unchanged: `main`, `tentacle-only-driving` (the walker drives itself, squeeze to steer),
> `MENY_MANUAL_EXTENDED` (Nav2).

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
7. [ASSIST + guardian reference: topics, parameters, scenarios](#assist--guardian-reference-topics-parameters-scenarios)
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

The user walks through an unfamiliar environment pushing the walker, holding both handles. **They are the motor's
input; the walker is the safety net.**

| User | What the walker does |
|---|---|
| Hands off the handles (for more than 0.3 s) | **Stops and holds** (dead-man grip). Nothing can make it move. |
| Hands on, not pushing | **Stays put.** It never moves on its own. |
| Pushes both handles | **Rolls forward**: push harder, go faster (up to 0.5 m/s), like a heavy, smooth cart |
| Pushes one handle harder | **Turns** that way, like a rollator (more on the right turns left) |
| Pulls back | **Slows**, or backs up slowly |
| One forward, one back | **Turns on the spot** (if the turn is clear) |

Resting hands don't move it (a 4 N deadband per handle). On the real walker the handles have load cells and a grip
sensor, read by an Arduino that also stops the motors in firmware. In VR the Quest controllers stand in: holding the
**grip triggers** is the hand on the handle, and the push comes from **where your hands are** (room scale: a virtual
spring between each hand and its handle) or the **thumbsticks** (stationary).

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

### How it moves: ASSIST and the guardian

```
your push (handle forces) ─► ASSIST (a virtual cart, from main) ─► where you're heading
                                                                         │
           guardian: checks YOUR path - the walker and you swept along it - against the LiDAR
                                                                         │
                clear ──► passes through untouched (the walker is invisible)
                something ahead ──► brakes, and/or bends your path round it
                                                                         │
                     velocity smoother ─► twist_mux ─► collision monitor (independent) ─► wheels
```

**The guardian's rules:**
- **It never pushes.** Neither wheel ever goes faster than your push makes it. It can only *hold the walker back*:
  braking both wheels slows you, braking one more than the other bends your path. (Checked every tick in every
  test.)
- **It never moves on its own.** No push, no motion.
- **It never takes you somewhere else.** It bends your path at most 35° away from where you're pushing, and once
  you're past the obstacle it undoes its own correction. A junction or a dead end is your decision: it brakes and
  waits for you to turn.
- **The EEG sets how early and firmly it helps.** A hazard you registered gets late, light help. One you missed
  (or no reliable EEG) gets help sooner and firmer, a wider berth (0.45 m instead of 0.15 m) and a speed cap of
  half pace.

**How it finds a safer path:** the tentacle fan from `tentacle-only-driving` - 31 arcs and 20 lane-change S-curves -
each checked by sweeping the walker-and-user footprint (1.25 × 0.6 m plus 12 cm) along it. It picks the clear one
closest to what you're doing, and bends towards it as far as the risk and the EEG say. Its brake planning aims to
stop with its front about 55 cm from an obstacle, braking at 0.3 m/s² (0.15 m/s² for a missed hazard, so it starts sooner).

### The trajectory display

The headset draws the walker's short-term path on the floor as faint, steady lines (no flicker, no animation),
marking its width over the next **2 seconds**:
- **Cyan lines: where your push is taking you.** When the guardian isn't doing anything, that's all you see.
- **When it steps in, the lines turn orange** and show where the walker will *actually* go - bent round the
  obstacle, or stopping short of it - and a thin **white line** shows where your push alone would have gone.
- The edges **widen** round a hazard you didn't register (a wider berth).
- **Turning on the spot:** the walker's outline where the turn will end. **Blocked:** a short amber outline in front.

### Where the head comes in

The head doesn't steer: your push does. The headset still tracks the head, for the hazard loop (which tags are in
view) and for the session log.

### Speed: the lowest limit always wins

```
actual speed = lowest of ( your push ,  guardian (room ahead, hazard caution) ,  velocity smoother ,  collision monitor )
```

| Your push | Hazard nearby? | EEG result | Result |
|---|---|---|---|
| any | none | – | exactly your push |
| any | yes | registered | your push; a light, late correction only if you're heading into it |
| any | yes | not registered, or no reliable EEG | at most half pace near it, a wider berth, help sooner and firmer, stronger tag |
| hands off | – | – | stop and hold |

The EEG result can only hold you back more. It never chooses a direction or starts motion.

---

## Safety rules

These hold in every mode, and nothing in the software can switch them off.

1. **The walker never moves on its own.** Only the user's push moves it; no push, no motion.
2. **Dead-man grip:** hands off for 0.3 s and it locks at zero. No grip readings, no LiDAR, or the operator's Stop
   walking also lock it.
3. **The guardian never pushes:** neither wheel is ever faster than the user's push. It only brakes, and bends the
   path by braking one side more, at most 35° from where the user pushes.
4. **The EEG never commands motion.** It only changes how early and how firmly the guardian holds the user back.
5. **Missing or uncertain EEG means cautious behaviour.**
6. **The collision monitor is an independent safety layer**, last in the chain. The guardian is never the only
   thing preventing a collision.
7. **Speed is capped** at 0.5 m/s (0.25 m/s backwards).
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
| User walks behind the walker | **Room scale** (default): the user walks for real in a cleared room, and their walking pushes the sim walker. **Stationary**: the user stands still, pushes with the thumbsticks, and the view moves with the walker. Eye height is always the user's real one. |
| Handle grip sensor + load cells (Arduino) | Holding the controllers' **grip triggers** = hands on. The **push**: room scale - a virtual spring between each hand and its handle (walk ahead of the handles and the walker comes along); stationary - the thumbsticks |
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

### Part A: one-time setup (about 35 minutes)

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

**A8. SET UP THE QUEST BOUNDARY FOR WALKING (ROOMSCALE).** This has to be done in the Quest itself: Meta doesn't
let any app or web page change the boundary.
1. Clear a space of about **3 × 5 m** (no furniture, cables or rugs to trip on).
2. On the Quest: **Settings → Boundary** (on some versions **Settings → Physical space → Boundary**).
3. Choose **Roomscale** (sometimes "Create boundary"), **not Stationary**. Stationary is a small circle round you;
   step out of it and the Quest fades the VR away, which is what happens if you try to walk with it.
4. Trace the outline of the cleared space on the floor with a controller, and confirm.

The app checks this every time you press Enter VR. If it finds a stationary circle (or a boundary smaller than
4 m²), it says so in the headset and on the operator page, and **uses stationary mode for that session**: the user
stands still and pushes with the thumbsticks. To walk, fix the boundary and press Enter VR again. (To only warn,
without switching, launch with `auto_stationary:=false`.) No space to walk at all? Use `vr_mode:=stationary`
and a Stationary boundary is fine.

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
ros2 launch my_robot_bringup walker_assist.launch.xml vr:=true eeg:=muse
```
This starts the simulated store and walker, ASSIST and the guardian, the grip (from the Quest's grip triggers),
the hazard loop, the EEG decoder, the web bridge and RViz (for the operator). Options:
- `vr_mode:=stationary` if there's no room to walk (default `room_scale`)
- `gazebo_view:=true` to also open the Gazebo 3D window (B5)

The walker **won't move** until the operator presses Start walking, the participant holds both grip triggers,
**and** they push.

**B5. Let other people watch, live.** While the participant is in VR, anyone at the PC can watch the walker moving
and operating **in real time, as the participant experiences it**. Each view joins the running session, and closing
one doesn't affect anything.

**Watch in RViz**: the robot as the participant experiences it: **where their push is taking them (white) and where
the walker actually goes (green, orange when the guardian steps in)**, the trajectory edges, the hazards and their
probe states, and the collision monitor's zones. Open a new WSL terminal and paste:
```bash
source ~/eye2steering_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/eye2steering_ws/install/my_robot_bringup/share/my_robot_bringup/rviz/walker_assist.rviz --ros-args -p use_sim_time:=true
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
1. **Room scale:** check the Quest boundary is **Roomscale** round the cleared space (A8), then stand the participant
   at one end of it, facing along its long side.
2. On the Quest, open the browser and go to `https://<PC-IP>:8443`.
3. Wait for **"Connected"**, then tap **Enter VR**.
4. Give them both controllers, **holding the grip triggers** (that's their hands on the handles). Ask them to hold
   them where the walker's handles would be, about 40 cm in front of them at hip height. After 1 s the view **recentres** automatically: the sim walker's handles appear at their hands,
   facing the way they face. To redo it, hold **A or X for 1 s** (or press **Recentre** on the operator page).

**B8. Check the SSVEP response** (about 1 minute). On the operator page press **SSVEP check**. A test patch flickers
in front of the participant at two frequencies in turn. The page shows PASS if the decoder detects each one. If it
fails, check the Oz electrode (B1) before continuing.

**B9. Press Start walking** on the operator page.

The headset then guides the participant by itself. A panel says when to wait, then walks them through six steps:
hold the grips, go forward, stop, turn, back up, full stop. Each step moves on only when they've really done it. After
that it says how to start moving again whenever they stop (for example **"Press and hold both grip buttons to start
moving again"**). No Muse is needed for this: leave out `eeg:=muse` to walk with the simulated EEG.

**B10. Tell the participant:**
- **Hold both grip triggers: those are your hands on the handles.** Let go and the walker stops.
- **Room scale: just walk.** Walk forward and the walker comes with you. Lead with one hand to turn; hold back to
  slow down. (Stationary: the left thumbstick pushes and pulls, the right one turns.)
- **The walker only ever helps.** If you're heading into something, it slows you and bends your path round it
  (the floor lines turn orange first). It never pulls you anywhere.
- Flickering patches appear on obstacles ahead, and fade if the EEG shows you've seen them. Ones you miss get more
  help.
- At a dead end it stops and says **"Blocked ahead. Turn to get round it"**: push one handle and pull the other to
  turn on the spot.
- Room scale: near the edge of the space it stops and says **"Room edge reached"**. Let go, hold **A or X for 1 s**
  to reset, turn round, and carry on.

**B11. To stop**, the participant lets go of the triggers, or press **Stop walking** on the operator page, or **Stop flicker** (all
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
ros2 launch my_robot_bringup walker_assist.launch.xml
```
**Terminal 2** (the handles):
```bash
source ~/eye2steering_ws/install/setup.bash
ros2 run my_robot_autonomy walker_keys
```
`space` hands on / off · `w` / `s` push harder / softer (below zero pulls back) · `a` / `d` turn left / right ·
`x` stop pushing · `e` "the user noticed it" (with `eeg:=toggle`) · `q` quit.

To watch in 3D as well, in another terminal:
```bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/eye2steering_ws/install/my_robot_description/share
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -g --render-engine ogre2
```
To drive by hand instead (teleop beats the guardian):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

### With simulated stand-ins for the headset or Muse

| You have | Launch options | What replaces the missing part |
|---|---|---|
| Nothing | `vr:=true grip:=quest eeg:=model`, then open `https://localhost:8443` on the PC (`http://` without the certificate, A5) | The browser shows the VR scene on the desktop. Drag to look; **W / S** push / pull, **A / D** turn (hands count as on while a key is down). `ssvep_emulator` decides what the "user" registers. |
| Quest, no Muse | `vr:=true eeg:=emulator` | Simulated raw EEG through the real decoder, driven by where the headset is actually looking |
| Muse, no Quest | `vr:=true grip:=quest eeg:=muse` and open the scene on a PC monitor (W / A / S / D to push) | The flicker is drawn on the monitor. For SSVEP testing, use a 60 Hz or higher monitor. |

EEG stand-ins: `eeg:=model` (each probed hazard registered after 2–4 s with probability `p_detect:=0.8`),
`eeg:=toggle` (registered only when you press `e` in `walker_keys`), `eeg_valid:=false` (no reliable EEG: everything
cautious), `eeg:=emulator` (simulated raw EEG, real decoder), `eeg:=muse` (the real thing).

---

## ASSIST + guardian reference: topics, parameters, scenarios

### Nodes and topics

| Node | In | Out |
|---|---|---|
| `grip_state_node` | `/walker/grip_raw` (Quest triggers), `/walker/grip_keys` (keyboard), serial (Arduino), `/walker/force_stop`, `/walker_enabled` | `/walker/grip_state` (hands on or off, 50 Hz); from the Arduino also `/walker/handle_force` |
| `assist_guardian_node` | `/scan`, `/walker/grip_state`, `/walker/handle_force`, `/hazards` | `/cmd_vel_guardian`, `/walker/intended_path` (where the push alone goes), `/walker/trajectory` + `_left` + `_right` (where the walker actually goes), `/walker/tentacles_debug`, `/walker/steering_status` (20 Hz); also `/plan` (for the hazard tracker), `/walker_status` |
| `velocity_smoother` (Nav2) | `/cmd_vel_guardian` | `/cmd_vel_smoothed` |
| `twist_mux` | `/cmd_vel_smoothed`, `/cmd_vel_joy` (teleop, wins) | `/cmd_vel_mux` |
| `collision_monitor` (Nav2) | `/cmd_vel_mux`, `/scan` | `/cmd_vel_out` (the walker) |
| `hazard_tracker` | `/scan`, `/plan`, `/probe_results`, `/eeg_status` | `/hazards` (with `registered`, `eeg_valid`) |
| `ssvep_emulator` / `eeg_decoder` | hazards / the Muse over LSL | `/probe_results`, `/eeg_status` |
| `web_server` | the headset | `/head_pose`, `/walker/grip_raw`, `/walker/handle_force`, `/walker/force_stop`; sends both paths, the guardian's state, hazards to the headset |

Messages (`walker_interfaces`): `HandleForce` (left, right in N, + = pushing), `GripState`, `SteeringStatus`
(intervention 0 … 1, braking, nudging, user speed, blocked, caution, notice …), `Hazard` / `HazardArray`,
`ProbeResult`.

### Parameters

All in `src/my_robot_bringup/config/`:

| File | What's in it |
|---|---|
| `assist_guardian.yaml` | ASSIST cart (mass, damping, deadband, speed limits), dead-man timeout, footprint and safety margin, the guardian (look-ahead, braking, when and how firmly it bends the path, berths, caution, the 35° limit, undoing its correction) |
| `grip.yaml` | grip source and thresholds, serial port |
| `velocity_smoother.yaml` | max velocity [0.5, 0, 0.8], backwards 0.25, braking up to 0.8 m/s² (hands-off stop; the guardian itself plans 0.3), open loop |
| `collision_monitor.yaml` | stop zone (walker body + 8 cm), slowdown zone (0.35 m ahead, half speed; the brief said ~0.5 m, but the guardian stops 0.45–0.55 m from walls and Nav2 slows *any* motion, even backing away, while a point is inside the zone) |
| `tentacle_planner.yaml` | the tentacle fan the guardian uses (and the self-driving planner on `walker_tentacle.launch.xml`) |

**What came from where:**
- From **`main`**: ASSIST (handle forces → motion through a virtual cart: same model and defaults as
  `eyesteer_core/control/admittance.py`) and the dead-man grip (`DeadMan`, 0.3 s). Ported, not merged: `main` is
  a different codebase (MuJoCo + `eyesteer_core`) and is untouched.
- From **`tentacle-only-driving`**: the tentacle fan with lane-change S-curves, the swept walker-and-user
  footprint, the berths and caution from the EEG hazard loop, the collision monitor and velocity smoother chain, the
  headset (room scale / stationary, Guardian stop), the scenario worlds and test harness.
- **New:** the guardian (check the user's path, brake, bend it by holding a wheel back, the 35° limit, undoing its
  own correction), the virtual-spring and thumbstick push in the headset, the two-colour floor lines.

**Design notes:**
- The guardian copies the walker's actual *speed* back into the cart (so a braked push doesn't wind up and lurch
  when released) but not its *turn rate*: that stays the user's own, so its correction never becomes "what the user
  wanted".
- If the user keeps steering into a wall, the guardian bends them away up to 35°, and past that just stops them.
  It won't overrule them further.
- The collision monitor's zones cover the walker body only, not the user (on the real walker the LiDAR would see
  the user's legs).
- The sim drives through Gazebo's DiffDrive plugin (`/cmd_vel_out`), not `diff_drive_controller`
  (`ign_ros2_control` isn't installed here).

### Scenarios

Each scenario is its own world. Run it in one terminal and the automatic check in another: it acts as the user
(holds the handles and pushes: 12 N per handle is a 0.4 m/s walk), measures what the walker does, and prints PASS
or FAIL. Every scenario also checks: no collisions, the collision monitor never needed, and **never faster than the
push**. The log of every tick goes to `~/eye2steering_ws/sessions/scenarios/assist_<name>.json`.
```bash
ros2 launch my_robot_bringup walker_assist.launch.xml scenario:=obstacle
```
```bash
source ~/eye2steering_ws/install/setup.bash
ros2 run my_robot_autonomy run_assist_scenario registered
```

| # | Check | `scenario:=` (world) | `run_assist_scenario` | Expected |
|---|---|---|---|---|
| 1 | Open space | `open` | `open` | No push: no motion. Push straight: exactly the pushed speed. Right handle harder: turns left. Push + pull: turns on the spot |
| 2 | Corridor | `corridor` | `corridor` | Drifting towards a wall: kept off it |
| 3 | T-junction | `tjunction` | `tjunction` | Pushed straight at the T: stops, doesn't take a branch by itself; the user's turn takes it into the branch |
| 4 | Obstacle, registered | `obstacle` | `registered` | Gets past with light help, no caution |
| 5 | Obstacle, no EEG | `obstacle eeg_valid:=false` | `unregistered` | Earlier, firmer help, half speed |
| 6 | Cluttered | `cluttered` | `cluttered` | Pushed straight: gets through, no collisions |
| 7 | Dead end | `deadend` | `deadend` | Stops before the wall; notice; the user turns it on the spot and walks out |
| 8 | Grip release | `open` | `grip_release` | Hands off: stops within the deceleration limit, even if the handles are still pressed; hands back on, no push: stays put |

Scenarios 9 (room scale) and 10 (stationary) are checked in the headset: after recentre the handles are at the
controllers; walking forward pushes the walker along with you; near the Guardian edge it stops with "Room edge
reached"; in stationary the thumbsticks push it and the view is never turned for you.

**Unit tests** (ASSIST, dead-man, the never-pushes rule, braking, bending, EEG-dependent help, plus the tentacle
geometry and grip tests):
```bash
cd ~/eye2steering_ws/src/my_robot_autonomy && python3 -m pytest test
```

### Swapping the stand-ins for the real hardware

| Stand-in | Real | How |
|---|---|---|
| Quest grip triggers + virtual spring / thumbsticks (`grip:=quest`), or the keyboard (`grip:=keyboard`) | Grip sensor + handle load cells on an Arduino | `grip:=arduino`. The Arduino sends one line per reading, `grip_left,grip_right,force_left,force_right\n`: grip as raw ADC counts (0–1023), forces in newtons (+ = pushing forward). Set `serial_port` in `grip.yaml`. |
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
| How heavy the walker feels | 25 kg, 40 N·s/m | `config/assist_guardian.yaml` (`mass`, `damping`) | Steady speed = push past the deadband ÷ damping (24 N → 0.4 m/s) |
| Top speed | 0.5 m/s (0.25 backwards) | `assist_guardian.yaml` (`v_max`) and `velocity_smoother.yaml` | |
| Push deadband | 4 N per handle | `assist_guardian.yaml` | Resting hands don't move it |
| Dead-man | 0.3 s in total | `assist_guardian.yaml` (`release_timeout_s`, 0.1 s) + `grip.yaml` (`release_grace_s`, 0.2 s) | Hands off this long → locked |
| When it starts bending your path | 2.3 m clear (2.6 m for a missed hazard) | `assist_guardian.yaml` (`comfort_*`) | Sooner = more help |
| How firmly | 1.0 (all the way to the safer path) | `assist_guardian.yaml` (`nudge_*`) | A registered hazard gets help later, not weaker |
| Braking it plans for | 0.3 m/s² (0.15 for a missed hazard) | `assist_guardian.yaml` (`brake_*`) | Lower = starts slowing sooner |
| Most it turns you | 35° | `assist_guardian.yaml` (`max_deviation_deg`) | Beyond this it only brakes |
| Hazard berth | 0.15 m registered, 0.45 m not | `assist_guardian.yaml` | |
| Caution speed | 50 % of top speed | `assist_guardian.yaml` (`caution_scale`) | With a missed / no-EEG hazard within 2 m |
| Room scale push | 90 N per m your hand is ahead of the handle, at most 30 N | `walker_vr` web server parameters | How the virtual spring feels |
| VR mode | room scale | launch option `vr_mode:=stationary` | Walk in a room, or stand and use the thumbsticks |
| Small-boundary fallback | on, below 4 m² | launch option `auto_stationary:=false`; web server `min_room_area` | Room scale with a Stationary (or too small) Quest boundary: warn and use stationary mode |
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
| Walker doesn't move after Start walking | It only moves when pushed: the participant must hold **both** grip triggers *and* walk forward (room scale) or push the left thumbstick (stationary). Check the grip state on the operator page. Or the system is still starting (about 10 s after launch). |
| **The view fades away when the participant walks** | The Quest boundary is set to **Stationary** (a small circle). Set up a **Roomscale** boundary (A8). Until then the app uses stationary mode (thumbsticks) and says so in the headset and on the operator page. |
| Headset says "Boundary too small to walk" | Same: Quest Settings → Boundary → Roomscale, about 3 × 5 m (A8), then Enter VR again |
| Walker stops with "Room edge reached" | Near the Guardian edge, or the participant is too far from the handles. Let go, hold A or X for 1 s to reset. |
| "Blocked ahead. Turn to get round it" | Nothing the way they're pushing is clear (dead end, junction, wall). Turn: push one handle and pull the other (room scale: one hand forward, one back). |
| The walker keeps stopping in a corridor | The participant is steering into a wall: the guardian bends them away up to 35°, then just stops them. Push straighter. |
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
├── my_robot_bringup/       launch files (walker_assist.launch.xml; walker_tentacle and the Nav2 / SLAM ones are kept), configs, the store
│                           and scenario worlds (worlds/scenarios, made by make_scenarios.py), RViz layouts
├── my_robot_autonomy/      the walker's behaviour (Python nodes)
│   ├── assist_guardian_node   ASSIST + the guardian (maths in guardian_core.py)
│   ├── grip_state_node     hands on the handles (Quest, keyboard, Arduino) and, from the Arduino, the push
│   ├── hazard_tracker      LiDAR → tracked hazards, relevance, probe queue, frequencies
│   ├── ssvep_emulator      stand-in SSVEP verdicts (model / toggle)
│   ├── eeg_decoder         Muse/LSL → SSVEP present / absent per hazard; the SSVEP check
│   ├── muse_emulator       simulated raw EEG for testing without a Muse
│   ├── eeg_check           EEG setup checks
│   ├── walker_keys         keyboard handles (grip and push), for desktop testing
│   ├── run_assist_scenario automatic ASSIST scenario checks (run_scenario: the self-driving planner's)
│   ├── session_logger      records each session to sessions/
│   ├── world_markers       the store drawn in RViz
│   ├── test/               unit tests (pytest)
│   └── also: tentacle_planner_node (self-driving, walker_tentacle.launch.xml), and older nodes for my_robot_nav.launch.xml
├── walker_interfaces/      ROS messages: handle force, grip state, steering status, hazards, probe results, trajectory, tentacles
└── walker_vr/              the web bridge (HTTPS + WebSocket), certificates, and the pages:
    └── web/                index.html (headset), spectator.html, operator.html
```

---

## Status

| Part | State |
|---|---|
| Simulated walker: LiDAR, IMU, camera, EKF, Gazebo store world | ✅ done |
| ASSIST (from `main`): handle forces → motion; dead-man grip | ✅ done, unit tested, tested in sim |
| Guardian: checks the user's path, brakes and bends it by holding a wheel back, never pushes, never more than 35°, undoes its correction | ✅ done, unit tested, tested in sim (8 scenarios) |
| EEG-dependent help: earlier and firmer, wider berth and half pace for a missed hazard | ✅ done, tested in sim |
| Push inputs: Arduino load cells, keyboard, headset virtual spring (room scale) and thumbsticks (stationary) | ✅ done; headset push checked for syntax and through the bridge, not yet in a real headset |
| Hazard tracking, probe queue, frequencies, `registered` / `eeg_valid` | ✅ done, tested in sim |
| SSVEP emulator, EEG decoder, Muse emulator, EEG checks | ✅ done, tested with the emulators |
| Web bridge, headset app, operator page, two-colour floor lines | ✅ done, tested with a simulated headset |
| **With a real Quest 3** (grip triggers, virtual-spring push, room scale, Guardian bounds, 90 Hz flicker) | ⏳ not tried yet (scenarios 9 and 10) |
| **With a real Muse + Oz** | ⏳ not tried yet |
| **With real handle sensors** (grip + load cells, Arduino) | ⏳ not tried yet |

**Test results** (Gazebo, `run_assist_scenario`; 12 N per handle = a 0.4 m/s walk):

| # | Scenario | Result | Measured |
|---|---|---|---|
| 1 | Open space | ✅ 7/7 | Hands on, no push: 0.0 cm moved. Push straight: 0.40 m/s exactly as pushed, 0 cm drift, 0 % intervention. Right handle harder: turned left 69°. Push + pull: turned 63° on the spot |
| 2 | Corridor | ✅ 4/4 | Drifting towards a wall: kept 25 cm off centre (wall at 75 cm), 4.8 m along |
| 3 | T-junction | ✅ 5/5 | Pushed straight at the T: stopped, turned only 29° by itself (limit 35°); the user's turn took it into the branch (89°) |
| 4 | Obstacle, registered | ✅ 5/5 | Got past, 17 cm from it, no caution, slowest 0.22 m/s, 5 % intervention on average |
| 5 | Obstacle, no EEG | ✅ 5/5 | Got past, 18 cm from it, caution (half pace), slowest 0.23 m/s, 4 % intervention |
| 6 | Cluttered | ✅ 4/4 | Pushed straight through 5 obstacles in 42 s; closest 17.9 cm (walker), 12.9 cm (user); 3 % intervention |
| 7 | Dead end | ✅ 7/7 | Stopped 0.45 m from the wall; "Blocked ahead" notice; the user turned it round on the spot (0 cm moved) and walked out |
| 8 | Grip release | ✅ 5/5 | Hands off at 0.40 m/s (handles still pressed): stopped in 0.95 s, 0.27 m, then held; hands back on, no push: stayed put |
| | **All** | **42/42** | **No contacts. The collision monitor never had to act. Never faster than the push in any of 4,600+ ticks checked.** |

All on the final build, except that scenarios 1–7 ran before the velocity smoother's braking limit was raised from 0.5 to 0.8 m/s² for the hands-off stop (that change only lets it stop harder). Unit tests: 42 pass.

**Known limitations, from testing in simulation:**
- **The walker and the user need a gap of about 0.85 m** (0.6 m wide, plus 12 cm either side), and a long one: with
  the user it is 1.25 m long, so it passes things nearly parallel. The guardian can't get them through a 1.05 m gap
  by itself; the user has to line up straight.
- **The guardian bends the user's path at most 35°**, so at a junction or dead end it just stops and the user turns.
  This is deliberate: where to go is the user's decision.
- **In the single-obstacle test, registered vs missed differ less than the design suggests.** A missed hazard gets
  caution (half pace) and a slightly wider pass (18 cm vs 17 cm), but bending round the box slows a registered pass
  almost as much (slowest 0.22 vs 0.23 m/s; 5 % vs 4 % intervention on average). The difference mainly shows when
  the user isn't heading straight at it. Tuning `comfort_*` and the berths would separate them more; worth deciding
  with real users which matters.
- **Hands off to stop: 0.27 m from 0.4 m/s.** That's the 0.3 s dead-man grace (as `main`), about 0.2 s of system
  latency, then braking at 0.8 m/s² (nobody is holding on). A shorter grace stops sooner, but makes brief slips of the
  hand stop the walker.
- Which objects count as hazards comes from the world file's names, standing in for an object classifier.
- The emulated EEG is a simplified model (3 µV SSVEP in background noise). Real responses vary from person to person,
  which is why the SSVEP check sets the threshold per participant.

**This branch builds what was the "manual mode" future extension:** the user pushes, and the robot steps in only
when an obstacle is in the path, braking one wheel more than the other so the user's own push carries them round
it. It only resists and never pushes.

---

## Why this is humanistic intelligence

Mann defines humanistic intelligence (HI) as intelligence that arises when a human is part of the feedback loop of a
computational process, with the human and computer inextricably intertwined [Mann 1998; Mann 2001].

**The loop.** The human moves the walker: every motion starts with their push. The machine senses the scene (LiDAR)
and shows the human what it sensed (a tag). The human senses the scene and the tag. The machine senses the human -
their push, and their visual response at Oz to each tag - and acts only by holding them back: braking, or bending
their path round something, earlier and more firmly for the hazards they didn't register. The human senses the
machine's response (the tag fading or intensifying, the floor lines turning orange and bending, the handles
resisting) and acts on it. Neither side alone can do what the combined system does: the LiDAR can't see what the
user has perceived, and the user can't see everything the LiDAR has detected.

| HI property | In this system |
|---|---|
| **Constancy** | The hazard loop and the guardian run continuously. There's no "BCI mode" to switch on. |
| **Augmentation** | Walking - and pushing the walker - is the primary task, and the user's own push is the only thing that moves it. The machine adds a sense of the scene and steps in only when that sense and the user's path disagree. |
| **Mediation** | The headset filters what the user sees: registered hazards fade, only missed ones are escalated. EEG is processed locally, and only "registered / not registered" per hazard reaches the motion system. |
| **Unmonopolizing** | Tags and floor lines are anchored in the scene rather than replacing it. The flicker is subtle and there's nothing to stare at. |
| **Unrestrictive** | The user can look around and talk: nothing they look at moves the walker. |
| **Observable** | Tags show what the LiDAR detected. The floor lines show where the user's push is taking them, and turn orange to show any correction before it happens. |
| **Controllable** | The user is in charge of every motion: they start it, set its speed and steer it, and letting go stops it. The machine can only hold them back - never push, never move on its own, never turn them more than 35° from their push - and it undoes its correction once clear. |
| **Attentive** | The machine combines the LiDAR, the user's perception (EEG), and their push and grip. |
| **Communicative** | The weakest. The headset is a channel from machine to human (tags, floor lines, notices), and the EEG and the handles are channels back. Sharing which hazards were missed with a caregiver is a future extension. |

### References
- S. Mann, "Humanistic Computing: 'WearComp' as a New Framework and Application for Intelligent Signal Processing,"
  *Proc. IEEE*, 1998.
- S. Mann, "Wearable Computing: Toward Humanistic Intelligence," *IEEE Intelligent Systems*, 2001.
- *[cite: head anticipation in human locomotion]*
- *[cite: risk-based arbitration in shared-control wheelchairs]*
