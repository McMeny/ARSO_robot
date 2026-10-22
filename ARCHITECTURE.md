# Architecture

One codebase, three runtimes. The logic Steve sees in the videos is the logic on the robot:
decoding, intent, FSM, queue, mux priorities, admittance and footprints live in the ROS-free package
**`eyesteer_core`**; the ROS 2 nodes are thin wrappers; the standalone sim imports it directly.

| runtime | what runs | used for |
|---|---|---|
| **C: standalone** | `standalone/` + `eyesteer_core`, MuJoCo, built-in A*/pure-pursuit (Nav2 stand-in), synthetic EEG | presentation videos, fast tests, `preflight` sim check |
| **A: ROS sim** (`backend:=mujoco`) | full ROS 2 stack + `mujoco_bridge` as the robot driver | the demo in sim, scenario tests in ROS |
| **A2: ROS HIL** (`backend:=mujoco_hil`) | + MCU emulator (firmware C core on a pty) → `mcu_bridge` → `ros2_control` + `diffdrive_arduino` | closest-to-real rehearsal, full-system golden path |
| **B: real** | identical upper stack on the Jetson; real MCU, LiDAR, Muse | demo day |

## Dataflow

```mermaid
flowchart LR
  subgraph Headset["Headset (WebXR, Quest browser)"]
    M[world-locked flicker markers<br/>frame-locked to the display]
    O[robot-state overlay]
    C[consent · calibration · placement]
  end
  subgraph EEG["Muse + Oz cup electrode"]
    E[LSL stream 'Muse' 256 Hz<br/>TP9 AF7 AF8 TP10 AUX=Oz]
  end
  EM[Muse LSL emulator<br/>SIMULATED / REPLAYED] -. sim .-> E
  M -- photons --> eye((user's eye + visual cortex)) --> E
  E --> SSVEP[ssvep_node<br/>CCA · intent · quality]
  SSVEP -- /bci/selection --> MM[mode_manager<br/>FSM + queue]
  C -- /xr/anchors · calibration --> AB[anchor_bridge] -- /markers (map) --> MM
  MM -- NavigateToPose --> NAV[Nav2<br/>AMCL · planner · RPP · BT]
  NAV -- cmd_vel_nav_raw --> SAF1[safety<br/>pull-back scale]
  SAF1 -- cmd_vel_nav --> MUX[twist_mux]
  TEL[teleop] --> MUX
  ADM[admittance<br/>walker ASSIST] --> MUX
  MUX -- cmd_vel_mux --> CM[collision monitor] -- cmd_vel_cm --> SAF2[safety<br/>rotation guard · watchdog]
  SAF2 -- /cmd_vel --> BE{{backend}}
  BE -- /scan /odom TF --> NAV
  MM -- /robot/status --> O
  HM[health_monitor] -- /lock/health --> MUX
  HM -- /system/health --> O
```

**The BCI never outputs velocities** (D7): `ssvep_node` emits selection events, `mode_manager` turns them into
`NavigateToPose` goals; Nav2 and the safety layers own all motion.

## Mode manager FSM

```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE: IDLE (WANDER field / ASSIST walker)
  IDLE --> ENGAGED: selection(marker)
  ENGAGED --> ENGAGED: selection -> queue (FIFO, dedupe, max 3)
  ENGAGED --> ARRIVED: Nav2 succeeded (0.30 m, 20°)
  ENGAGED --> IDLE: CANCEL / Nav2 failed x3
  ARRIVED --> ENGAGED: queue non-empty after 1.5 s hold / selection in the 2 s grace window
  ARRIVED --> IDLE: grace window over, queue empty
  ENGAGED --> PAUSED: walker: EEG or headset link lost
  PAUSED --> ENGAGED: link restored (same goal)
  IDLE --> HOLD: field robot: headset link lost (no wander)
  HOLD --> IDLE: headset back
  IDLE --> LOST: scan/map mismatch or AMCL covariance
  ENGAGED --> LOST
  LOST --> IDLE: recovered
  IDLE --> ESTOP: e-stop
  ENGAGED --> ESTOP
  ESTOP --> IDLE: explicit reset (latched)
```

Selection ≠ intent (D3): a selection needs argmax = m for N = 3 consecutive 2 s windows (0.25 s step),
ρ ≥ θ = 0.40, margin ≥ δ = 0.08, the engagement gate, and a 2 s per-marker refractory period; otherwise NO-CONTROL.

## Velocity arbitration (HI controllability)

```mermaid
flowchart TB
  L1["255 lock/estop (latched)"] --> Z[zero]
  L2["254 lock/deadman (walker grip released > 0.3 s)"] --> Z
  L3["253 lock/health (gated startup, red runtime gates)"] --> Z
  A["100 cmd_vel_admittance (walker, pushed)"] --> W{highest active wins}
  T["90 cmd_vel_teleop"] --> W
  N["10 cmd_vel_nav (BCI goal via Nav2)"] --> W
  W --> CMn[collision monitor: stop +0.1 m / slow +0.4 m ahead] --> R[rotation guard + 0.5 s watchdog] --> MCU[MCU: 200 ms watchdog + physical e-stop]
```

The human always wins: e-stop > dead-man > handle force > teleop > BCI navigation. In ENGAGED, pulling back on
the walker's handles scales the nav command down (≥ 12 N) to zero (≥ 30 N).

## Sim / real backend swap (D15, D20)

```mermaid
flowchart LR
  subgraph Same["identical in every runtime"]
    US[Nav2 · twist_mux · collision monitor · behaviour nodes · BCI · rosbridge · WebXR app]
  end
  US -- /cmd_vel --> B1[mujoco_bridge<br/>backend:=mujoco]
  US -- /cmd_vel --> DD[diff_drive_controller<br/>+ diffdrive_arduino]
  DD --> MB[mcu_bridge<br/>serial proxy]
  MB -- pty --> EMU[MCU emulator<br/>firmware C core] --> MJ[MuJoCo wheels]
  MB -- /dev/eyesteer_mcu --> FW[real MCU firmware<br/>same C core] --> MOT[motors]
  B1 --> MJ2[MuJoCo]
```

`sim.launch.py backend:=mujoco|mujoco_hil` vs `real.launch.py`: only the bottom layer changes.

## TF tree

```mermaid
flowchart TB
  map -->|AMCL / slam_toolbox| odom
  odom -->|bridge (sim) / diff_drive_controller (HIL, real)| base_footprint
  base_footprint --> base_link
  base_link --> laser_frame
  base_link --> imu_link
  base_link --> camera_link --> camera_optical_frame
  base_link --> left_wheel_link
  base_link --> right_wheel_link
  base_link --> casters
  base_link --> handle_left_ft --> grip_left
  base_link --> handle_right_ft --> grip_right
```

## Headset ↔ map calibration

```mermaid
sequenceDiagram
  participant U as User (controller)
  participant H as WebXR app
  participant A as anchor_bridge
  participant TF as TF (AMCL)
  U->>H: touch FRONT decal, trigger
  U->>H: touch REAR decal, trigger
  H->>A: /xr/compute_calibration(front, rear)
  A->>TF: map -> base_footprint
  A->>A: decals in map (robot YAML) vs headset points -> yaw, t, floor (2-point rigid fit)
  A-->>H: calibration + residual (redo if > 5 cm)
  H->>A: /xr/anchors (markers in the headset frame)
  A->>A: headset -> map, persist ~/.eyesteer/markers/<map>.yaml
  A-->>H: /markers (map frame, latched) -> rendered world-locked
```

## Topics, actions, services
See [INTERFACES.md](INTERFACES.md) for the full contract (types, rates, QoS, frames) at every boundary.

## Gated startup
`health_monitor` publishes `/system/health` (muse_lsl, bci, headset, frequency_table, lidar, tf, localization,
nav2, mcu, safety) and holds the `lock/health` twist_mux lock until every required gate is green; the dashboard
(`/dashboard.html`), the headset panel and the HUD show the same list. `make demo-sim` / `make demo-real`
(`tools/demo.py`) wait for the gates in dependency order with a specific hint for each red one.
