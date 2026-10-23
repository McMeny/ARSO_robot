# Eye as a Steering Wheel

An SSVEP-driven, Humanistic-Intelligence assistive robot and smart walker. The user looks at a **virtual,
world-locked flickering marker** in a mixed-reality headset (Meta Quest 3, passthrough WebXR). A Muse EEG headband
with an **Oz cup electrode** picks up the steady-state visual evoked potential, and the robot (or a walker guiding
its user) navigates to that marker. The BCI only ever selects **goals**; Nav2 and layered safety own all motion,
and the human can always observe and override (e-stop > dead-man grip > handle force > teleop > BCI).

Research system for a paper with Prof. Steve Mann (target venue: IEEE ARSO). Start with [FOR_STEVE.md](FOR_STEVE.md).

| doc | contents |
|---|---|
| [FOR_STEVE.md](FOR_STEVE.md) | one-page plain-language explainer with stills |
| [DEMO_DAY.md](DEMO_DAY.md) | step-by-step script for the meeting, troubleshooting, safety checklist |
| [DEPLOY.md](DEPLOY.md) | Jetson setup, flashing, bring-up order, calibration, troubleshooting |
| [ARCHITECTURE.md](ARCHITECTURE.md) | dataflow, FSM, mux priorities, sim/real swap, TF tree, calibration (mermaid) |
| [INTERFACES.md](INTERFACES.md) | the contract at every boundary (each with a test) |
| [DECISIONS.md](DECISIONS.md) | every choice made, with reasons, and the package versions confirmed |
| [outputs/metrics/summary.md](outputs/metrics/summary.md) | scenario metrics in both runtimes, golden-path latency budget |

## One codebase, three runtimes
| runtime | command | what it is |
|---|---|---|
| **C: standalone sim** | `python standalone/app/run_scenario.py supermarket_walker` | MuJoCo + `eyesteer_core` + built-in planner; renders the presentation videos |
| **A: ROS sim** | `make sim world=supermarket robot=walker gaze=headset eeg=synthetic` | the full deployable ROS 2 stack (Nav2, twist_mux, collision monitor, BCI over LSL, rosbridge, WebXR app) with MuJoCo as the robot |
| **A2: ROS HIL** | `make sim … backend=mujoco_hil` / `make demo-sim` | as A, but motion goes through the real `ros2_control` + `diffdrive_arduino` path into an MCU emulator running the firmware's own C core |
| **B: real robot** | `make demo-real robot=walker` | identical upper stack on the Jetson with the real MCU, LiDAR and Muse |
| **Desktop demo** (no headset, no Muse) | `make demo-desktop` (sim) / `make demo-real-desktop` (robot) | a browser's 2D map is the headset: click a marker or press its number to look at it; SIMULATED EEG through the real decoder; no on-screen flicker ([DEMO_DAY.md](DEMO_DAY.md#fallback-no-headset-no-muse-desktop-demo)) |

Decoding, intent, FSM, queue, mux rules, admittance and footprints live in the ROS-free package `eyesteer_core`.
The ROS nodes are thin wrappers around it, so the logic in the videos is the logic on the robot.

## Quick start
```bash
make install          # Python deps (+ PlatformIO, Playwright)
make generate         # config -> frequency table, URDF/xacro/MJCF, controllers, Nav2 params, firmware header, maps, decals
make test             # core + standalone scenarios + firmware host tests + headset unit tests
make render-all       # six videos + stills + metrics in outputs/
python standalone/app/interactive.py --world supermarket --robot walker   # MuJoCo viewer; keys 1-5 gaze, C cancel, Space e-stop …
```
ROS (Ubuntu 22.04 + Humble, or the Jetson):
```bash
bash deploy/build_deps_from_source.sh --all   # only if twist_mux / rosbridge / diffdrive_arduino are not installed
source deploy/env.sh && (cd ros2_ws && colcon build --symlink-install)
make sim world=field robot=field_robot        # then open http://localhost:8443 (https with certs: make certs)
make test-ros                                 # S1-S6 through the ROS stack
make test-integration                         # golden path (HIL) x5 per robot, failure injection, WebXR (Playwright)
```

## Configuration
`config/hardware.yaml` is the only file normally edited on site (board, pins, encoder CPR, LiDAR, headset refresh
rate, EEG tool and channel map). The measured robot dimensions live in `config/robot_field.yaml` and
`config/robot_walker.yaml`, and are the single source of truth for the URDF, MJCF, controller YAML, Nav2 footprint
and firmware constants. Values that are placeholders are marked "MEASURE ON THE REAL ROBOT".

## Repository layout
```
config/            hardware, robots, BCI, worlds, marker layouts, scenarios (+ generated/)
tools/             generators, frequency computation, BCI trade-off, motor_test, eeg_check, flash, demo, decals
eyesteer_core/     ROS-free core: bci/ fsm/ control/ world, xr_calibration
standalone/        Runtime C: sim/ nav/ viz/ app/
ros2_ws/src/       eyesteer_{interfaces,description,mujoco,bci,behavior,bringup}
firmware/          PlatformIO project; lib/eyesteer_fw (shared C core), test/, emulator/
webxr/             headset app (index.html, dashboard.html, js/, tests/)
deploy/            jetson_setup, udev, systemd, certs, preflight, docker, env.sh
tests/             pytest (core, contracts, scenarios), ros/ scripts, integration/ (HIL golden path, failures, Playwright)
outputs/           videos/ stills/ metrics/ logs/
```

## Safety and honesty
- Flicker runs only in the headset, behind a photosensitivity consent screen. Videos use a slow, labeled pulse.
- Synthetic or replayed EEG is always labeled **SIMULATED EEG** / **REPLAYED EEG**.
- Two watchdogs: the MCU stops the motors 200 ms after the last command; the ROS side zeroes stale `cmd_vel`.
- Motion stays locked until every health gate is green (`/dashboard.html`).

## Verifiable only on the real hardware
See the end of [DEPLOY.md](DEPLOY.md) and the final section below. Each item is one command with PASS/FAIL output.

| check | command (on the Jetson) |
|---|---|
| MCU protocol, encoders by hand, 200 ms watchdog | `bash deploy/preflight.sh --only mcu` (wheels off the ground) |
| motor polarity / encoder direction / CPR | `python3 tools/motor_test.py` / `--measure-cpr` |
| speed feed-forward constant (then `make generate` + `python3 tools/flash.py`) | `python3 tools/motor_test.py --measure-ff` |
| e-stop button wiring + latch | `bash deploy/preflight.sh --only mcu` (press and release the button when prompted) |
| LiDAR rate + frame, TF tree | `bash deploy/preflight.sh --only lidar` / `--only tf` (stack running) |
| Muse stream, channel map, 10 s spectrum | `python3 tools/eeg_check.py stream` |
| Oz contact + mapping (eyes-closed alpha) | `python3 tools/eeg_check.py alpha` |
| headset → eye → Muse → decoder chain | `python3 tools/eeg_check.py ssvep` (headset open, consent given) |
| headset refresh, frame timing, calibration residual, marker round trip | dashboard `headset` gate (`https://<jetson>:8443/dashboard.html`) |
| odometry calibration, PID, LiDAR pose | DEPLOY.md §4 steps 6–8 |
| full chain: synthetic gaze → real EEG (reduced, then full speed) | `make demo-real` (DEMO_DAY.md Part 2) |
