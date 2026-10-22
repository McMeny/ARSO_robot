# INTERFACES — the contract at every boundary

Every row below has a test. Static contract tests: `tests/test_contracts.py` (no ROS needed).
Live tests against the running stack: `tests/integration/` (sim, HIL, headless headset).

## 1. Muse → LSL
| item | contract |
|---|---|
| stream | name/type from `config/hardware.yaml → eeg.tools[eeg.source_tool]`; default **muselsl**: name `Muse`, type `EEG` |
| channels | 5, order **TP9, AF7, AF8, TP10, AUX(=Oz cup electrode)**; labels in `<desc><channels>`; `eeg.channel_map` maps names → indices (`Oz: 4`) |
| units / rate | microvolts, float32, nominal **256 Hz** |
| timestamps | LSL `local_clock()` of the producer; consumers apply `time_correction` (clock-sync + dejitter) |
| tool mapping | muselsl (`Muse`, AUX = "Right AUX"), Petal Metrics (`PetalStream_eeg`), Mind Monitor via an OSC→LSL bridge (`MindMonitor`), the user's Eye-as-a-Camera code (`Muse`) — select with `eeg.source_tool` |
| emulator | `eyesteer_bci/muse_emulator.py` publishes **exactly** this layout (synthetic or replayed) |
| tests | `test_contracts.py::test_muse_emulator_stream_matches_contract` (pylsl, real outlet) |

## 2. LSL → `ssvep_node`
| item | contract |
|---|---|
| resolve | `resolve_byprop("name", …)` + type check, timeout `eeg.resolve_timeout_s` (5 s); retried forever |
| inlet | `max_buflen=4 s`, `proc_clocksync | proc_dejitter` |
| loss | no samples for 2 s → status `BAD SIGNAL (stream lost)`, **no selections**, reconnect loop; recovers automatically |
| rate | measured from LSL timestamps over 3 s, published in `/bci/status.sample_rate`; health gate 256 ± 5 Hz |
| quality | Oz flat-line (< 0.5 µV sd), saturation (> 1500 µV), huge variance (> 300 µV sd), 60 Hz ratio > 50 → BAD SIGNAL |
| tests | `test_bci.py` (quality, decoder, intent), `tests/integration/test_failures.py::test_muse_stream_drop_and_flat` |

## 3. Headset ↔ rosbridge (WebXR app, `webxr/js/ros.js`)
Endpoint `wss://<jetson>:9090` (TLS, same cert as `https://<jetson>:8443`); plain `ws://` when no cert (dev). The app
reconnects every 2 s and re-subscribes; latched topics (`/markers`, `/xr/calibration`) resync state.

| topic / service | type | dir | rate / QoS |
|---|---|---|---|
| `/robot/status` | `eyesteer_interfaces/RobotStatus` | → headset | 10 Hz (throttled 50 ms) |
| `/markers` | `eyesteer_interfaces/MarkerArray` (frame `map`) | → headset | latched (transient local) |
| `/bci/status` | `eyesteer_interfaces/BciStatus` | → headset | 4 Hz |
| `/system/health` | `eyesteer_interfaces/SystemHealth` | → headset | 2 Hz |
| `/xr/calibration` | `eyesteer_interfaces/HeadsetCalibration` | → headset | latched |
| `/bci/calibration_cue` | `std_msgs/Int32` (marker id, −1 rest, −2 done) | → headset | events |
| `/bci/selection` | `eyesteer_interfaces/Selection` | → headset | events |
| `/amcl_pose`, `/sim/ground_truth` | `geometry_msgs/PoseWithCovarianceStamped`, `nav_msgs/Odometry` | → headset | ≤ 10 Hz |
| `/sim/people` | `geometry_msgs/PoseArray` (x, y, z = radius; sim only) | → headset | 5 Hz |
| `/eyesteer/latency` | `std_msgs/String` (JSON `{event, wall, dt_ms…}`) | → headset | events |
| `/eyesteer/status_ack` | `std_msgs/String` (displayed state) | headset → | on every displayed state change (latency on one clock) |
| `/eyesteer/ping` | `std_msgs/String` (page time) | headset → | 0.5 Hz; answered by a `pong` on `/eyesteer/latency` (clock offset) |
| `/xr/timing` | `eyesteer_interfaces/XrTiming` | headset → | 2 Hz (also the headset heartbeat: 3 s silence = disconnected) |
| `/xr/anchors` | `eyesteer_interfaces/MarkerArray` (frame `map` or `headset`) | headset → | on edit |
| `/bci/gaze` | `std_msgs/Int32` (aimed marker id, 0 = none) | headset → | 10 Hz, **only when EEG is SIMULATED** |
| `/estop` | `std_msgs/Bool` | headset → | event |
| `/xr/compute_calibration` | `eyesteer_interfaces/ComputeHeadsetCalibration` | service | on demand |
| `/bci/calibrate` | `eyesteer_interfaces/Calibrate` | service | on demand (≈ 90 s for real EEG) |

JSON shapes = the ROS message fields (rosbridge convention). Tests: `test_contracts.py::test_webxr_topics_match_interfaces`,
`tests/integration/test_webxr_preview.py`.

## 4. Headset anchors → `anchor_bridge` → `map`
| item | contract |
|---|---|
| headset frame | WebXR `local-floor`: x right, y up, z towards the viewer, metres |
| transform | ground plane (u, v) = (−z, −x); `[x,y]_map = R(yaw)[u,v] + [tx,ty]`, `z_map = y − floor_height` (`eyesteer_core/xr_calibration.py` ≡ `webxr/js/calib.js`) |
| calibration | two decal touches (front/rear, positions in `robot_*.yaml → calibration_decals`) + robot map pose (TF `map→base_footprint`) → `/xr/compute_calibration`; residual ≤ 0.05 m else redo |
| markers | `/markers` latched, frame `map`, frequency from the served table by slot; CANCEL slot = highest frequency |
| persistence | `~/.eyesteer/markers/<map>.yaml` (real robot), reloaded on startup; sim uses the default layout |
| tests | `test_contracts.py::test_calibration_python_js_parity`, `webxr/tests/unit.test.mjs`, integration golden path step 2–3 |

## 5. `bci.yaml` → headset + decoder (frequency table)
One table, `config/generated/frequencies.json`, computed from `headset.refresh_hz`. The decoder (`ssvep_node`), the
markers (`anchor_bridge`) and the headset (`/config/frequencies.json`) all carry its **hash**; any mismatch →
`frequency_table` health gate red. A display refresh ≠ table refresh → headset blocks run mode + `headset` gate red.
Tests: `test_frequencies.py`, `test_contracts.py::test_single_frequency_table`, `test_webxr_preview.py::test_refresh_mismatch…`.

## 6. `mode_manager` → Nav2
| item | contract |
|---|---|
| action | `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`), goal frame **`map`** |
| goal pose | `approach_distance` in front of the anchored point along `facing`, heading towards it; nudged to the nearest pose with ≥ 0.15 m footprint clearance on `/map` |
| tolerances | `goal_xy_tolerance` 0.30 m, `goal_yaw_tolerance_deg` 20° (robot YAML → Nav2 goal checker) |
| preempt / cancel | a new goal preempts (results of stale goals are ignored by sequence number); CANCEL / e-stop / LOST → `cancel_goal_async` |
| failures | `ABORTED` or rejection → retry ×3 → IDLE with reason; walker BT has no Spin recovery |
| tests | `test_fsm.py`, `test_contracts.py::test_nav2_params_match_robot_yaml`, scenarios in ROS sim |

## 7. Velocity chain: Nav2 / teleop / admittance → `twist_mux` → collision monitor → `/cmd_vel`
```
controller_server, behavior_server --cmd_vel_nav_raw--> safety (walker pull-back scale) --cmd_vel_nav--+
teleop ------------------------------------------------------------------------------cmd_vel_teleop----+--> twist_mux --cmd_vel_mux-->
admittance (walker, ASSIST only) -----------------------------------------------cmd_vel_admittance----+
   collision_monitor --cmd_vel_cm--> safety (rotation guard, 0.5 s staleness watchdog) --/cmd_vel--> backend
```
| priority | source | timeout |
|---|---|---|
| 255 | lock `/lock/estop` (latched until `/system/reset`) | – |
| 254 | lock `/lock/deadman` (walker grip released > 0.3 s) | – |
| 253 | lock `/lock/health` (gated startup + red runtime gates) | – |
| 100 | `cmd_vel_admittance` | 0.3 s |
| 90 | `cmd_vel_teleop` | 0.5 s |
| 10 | `cmd_vel_nav` | 0.5 s |

Generated from `eyesteer_core/control/mux_rules.py` (`params/generated/twist_mux.yaml`). Tests:
`test_control.py`, `test_contracts.py::test_velocity_chain_topics`.

## 8. `ros2_control` ↔ MCU (serial)
| item | contract |
|---|---|
| plugin | `diffdrive_arduino/DiffDriveArduinoHardware` (joshnewans, branch `humble`) |
| port | `mcu_bridge` owns `/dev/eyesteer_mcu` (real) or the emulator pty (HIL) and exposes `/tmp/eyesteer_mcu_ros` to diffdrive_arduino |
| line format | ASCII, request terminated `\r`, reply terminated `\r\n`, one reply per request, 57600 baud |
| commands | `e` → `"<l> <r>"` encoder totals; `m <l> <r>` → `OK`, target **counts per PID loop** at `PID_RATE_HZ` = `loop_rate` = 30; `u p:d:i:o` → `OK`; `\r` → empty line |
| extensions | `s` → `S <estop_latched> <estop_input> <watchdog_tripped> <moving> <ms_since_cmd>`; `h` → `H <fl_dN> <fr_dN> <grip>`; `x` → clear e-stop; `v` → version |
| watchdog | no `m`/`o` for **200 ms** → motors off (firmware), `e` polling does not feed it |
| e-stop | physical input → latched stop, cleared by `x` after release (`/system/reset`) |
| encoder CPR | `encoders.counts_per_rev` (hardware.yaml) in URDF param `enc_counts_per_rev` and firmware header |
| tests | `firmware/test/test_core.cpp`, `tests/test_mcu_emulator.py`, `test_contracts.py::test_hw_interface_params_match_firmware`, HIL golden path |

## 9. LiDAR → SLAM / AMCL / Nav2
`/scan` `sensor_msgs/LaserScan`, frame **`laser_frame`**, 10 Hz, 360 beams, range 0.15–12 m, `inf` = no return/masked;
published **reliable** (compatible with SensorData best-effort subscribers). Real: `sllidar_ros2 → /scan_raw → scan_mask → /scan`
(walker rear sector 135–225° masked). TF `base_link → laser_frame` from the URDF. Tests: smoke + health `lidar` gate.

## 10. TF tree
`map → odom → base_footprint → base_link → {laser_frame, imu_link, camera_link → camera_optical_frame, left/right_wheel_link, casters, handle_{left,right}_ft → grip_*}`

| edge | sim (`mujoco`) | HIL (`mujoco_hil`) | real |
|---|---|---|---|
| map → odom | AMCL (or slam_toolbox) | AMCL | AMCL |
| odom → base_footprint | `mujoco_bridge` (encoder odometry) | `diff_drive_controller` | `diff_drive_controller` (or the vendor base driver) |
| base_footprint → … | `robot_state_publisher` (URDF) + `/joint_states` from the bridge | rsp + `joint_state_broadcaster` | rsp + `joint_state_broadcaster` |

## 11. Clock
Sim and HIL: `/clock` from `mujoco_bridge` (real-time paced), **every** node `use_sim_time:=true`. Real robot:
system time, `use_sim_time:=false`. Never mixed (`test_contracts.py::test_clock_settings_in_launch_files`). The MCU
emulator in HIL uses the sim clock for its PID and watchdog, so its 200 ms is the same 200 ms as the stack's.
