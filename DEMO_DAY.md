# DEMO DAY: script for the meeting with Steve

Total ≈ 60 min. Part 1 (simulation) ≈ 25 min, Part 2 (real robot) ≈ 35 min.
**Before anyone looks at a flickering marker: the photosensitivity screening question** ("any history of seizures or
epilepsy, or flashing lights making you unwell?"). If yes → synthetic gaze only, no flicker.

## Setup (30 min before)
- [ ] Jetson + Quest on the same Wi-Fi (or the Jetson hotspot). `ping eyesteer.local` from the laptop.
- [ ] Quest browser: `https://eyesteer.local:8443` loads (certificate accepted), and `https://eyesteer.local:9090` accepted once.
- [ ] Quest display refresh = `headset.refresh_hz` (90 Hz) in `config/hardware.yaml`.
- [ ] Muse charged, Oz cup electrode + gel ready, `muselsl` paired (`muselsl list`).
- [ ] Videos S1–S6 copied to the laptop (`outputs/videos/`), `FOR_STEVE.md` open.
- [ ] Robot: battery charged, e-stop tested (press → wheels stop), wheels off the ground for the first power-up.

## Part 1: simulation (same stack, same headset app)
1. On the Jetson (or the laptop):
   ```bash
   make sim world=supermarket robot=walker gaze=headset eeg=synthetic
   ```
   (gated version: `make demo-sim world=supermarket robot=walker`. It prints each health gate as it turns green.)
2. Quest browser → `https://<host>:8443` → read the consent screen together → **I understand**. The supermarket's
   markers appear (in sim they come from the default layout; show "Place markers" to explain how they would be placed
   on real shelves). The badge says **SIMULATED EEG (gaze-driven)**.
3. Steve aims at **Beans** for about 3 s. The CCA bars rise, dwell fills, the selection fires, and the simulated walker
   guides the simulated user to the beans. Show at the same time:
   - the MuJoCo viewer: the walker plus the user, and the shopper crossing;
   - RViz: the plan, the collision-monitor polygons, and the MR markers as translucent panels;
   - the headset overlay: state, queue and mux source.
4. Show the overrides live: `Space` in the MuJoCo viewer = e-stop (latched; `R` resets). `G` releases the grip:
   dead-man, full stop. `S` pulls the handles back: the walker slows. While driving, look at **Milk**: it is queued
   with badge 1. Look at **CANCEL** to clear everything.
5. Play the pre-rendered videos S1–S6 (`outputs/videos/`). They show the BCI decoding panel (Oz trace, ρ bars,
   threshold, dwell), and the no-control S3 shows that looking is not commanding.
6. If the Muse is on: stop, `make sim … eeg=lsl`. Run the per-user calibration in the headset (Run → Per-user EEG
   calibration: 10 s per marker, then 20 s looking at nothing). Then drive the sim robot with real SSVEP; the badge
   now says REAL EEG.

## Part 2: real robot
```bash
make flash                                  # firmware for hardware.yaml board/robot (skip if already flashed)
make preflight                              # MCU + watchdog, LiDAR, TF, web, EEG stream / alpha / SSVEP, sim suite
ros2 launch eyesteer_bringup slam.launch.py sim:=false robot:=walker      # map the room (or reuse ~/maps/demo.yaml)
ros2 run nav2_map_server map_saver_cli -f ~/maps/demo
make demo-real robot=walker                 # = real.launch.py + gated startup; motion locked until all green
```
1. RViz: "2D Pose Estimate" → the localization gate turns green.
2. Headset → **Calibrate**: with the robot stationary, touch FRONT decal → trigger, REAR decal → trigger; residual < 5 cm.
3. **Place markers** on the real targets (e.g. a chair "Beans", a door "Checkout", one CANCEL near the user).
4. **Synthetic-gaze test first** (EEG out of the loop): `eeg:=synthetic` → aim at a marker → the robot drives there.
5. **EEG**: `python3 tools/eeg_check.py alpha` (eyes open/closed, PASS), `tools/eeg_check.py ssvep` (each marker in
   turn, PASS), then the per-user calibration in the headset.
6. **Full run** at reduced speed (`nav2_real`: 0.3 m/s), then the field-robot WANDER demo or the walker demo,
   **without a person on the walker first**, then with Steve holding the handles (spotter beside him).

## Fallback: no headset, no Muse (desktop demo)
For a dead Quest battery, no Muse, an audience member who should not see flicker, or a quick show from a laptop.
The browser's 2D map stands in for the headset, and nothing flickers on screen.
- **Simulation:** `make demo-desktop world=supermarket robot=walker` (or `world=field robot=field_robot`), then open
  `https://localhost:8443`.
- **Real robot:** `make demo-real-desktop robot=walker` on the Jetson, then on a laptop open `https://eyesteer.local:8443`.
  - Set the robot's pose ("2D Pose Estimate" in RViz) so the localization gate turns green.
  - In the **Place markers** tab, click on the map where each target is.
- **Run tab:**
  - Click a marker, or press its number, to "look" at it. Hold for ~2.5 s and the decoder selects it.
  - `C` looks at CANCEL. `0` or `Esc` looks away. `Space` is an e-stop.
- **No ROS at all** (any laptop with Python): `python standalone/app/interactive.py --world supermarket --robot walker`
  opens a MuJoCo window. Keys `1`–`5` look at a marker, `C` looks at CANCEL, `0` looks away, arrows teleop, `Space`
  e-stop, `R` reset. The decoder and FSM are the same, but it uses
  a built-in planner instead of Nav2.
- **What stays real:** the decoder, intent rules, FSM, Nav2, collision monitor, health gates and every safety layer
  (watchdogs, e-stop, walker dead-man grip).
- **What does not:** the EEG is **SIMULATED** (labeled everywhere: banner, badge, logs). Say so out loud. This shows
  the control loop, not a neural result.

## Troubleshooting (fast)
| symptom | do this |
|---|---|
| no `/scan` | `ls -l /dev/lidar`; replug; check `lidar.model` baud (C1 = 460800) |
| TF errors | set the initial pose in RViz; `ros2 control list_controllers` must show diff_cont + joint_broad active |
| robot spins instead of driving | a wheel axis / motor polarity is flipped: `python3 tools/motor_test.py`, flip `invert_*` in hardware.yaml, `make generate flash` |
| odometry drift | re-check wheel radius / separation (DEPLOY.md §4 step 6) |
| rosbridge cert not accepted | open `https://<host>:9090` in the Quest browser once and accept |
| refresh-rate mismatch warning | set the Quest display to the configured rate (90 Hz) and reload |
| LSL stream not found | `muselsl stream --aux`; `tools/eeg_check.py stream`; `eeg.stream_host` if two streams |
| BAD SIGNAL | re-gel the Oz electrode; `tools/eeg_check.py alpha`; move away from chargers / mains |
| Nav2 goal rejected | marker too close to a wall or inside an obstacle → move it; relocalize |
| motion locked | open `https://<host>:8443/dashboard.html`; the red gate says why |

## Safety checklist (read aloud before Part 2)
- [ ] Physical e-stop tested today: pressed → wheels stop immediately; stays stopped until reset.
- [ ] Speed caps: real Nav2 params 0.3 m/s; walker ≤ 0.5 m/s hard limit in firmware/controller.
- [ ] A spotter stands next to the robot/walker with the e-stop in reach at all times.
- [ ] Photosensitivity screening done for everyone who will look at the headset.
- [ ] The walker has been driven through the whole route **without a person** first.
- [ ] Dead-man tested: releasing the grip stops the walker within 0.3 s.
- [ ] Teleop (keyboard) running and verified to override navigation.
