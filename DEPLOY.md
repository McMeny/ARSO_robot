# DEPLOY — Jetson, firmware, bring-up, calibration, troubleshooting

Everything hardware-specific is in **`config/hardware.yaml`** (plus the measured dimensions in
`config/robot_field.yaml` / `config/robot_walker.yaml`). After any edit: `make generate`.

## 0. What runs where
| device | runs |
|---|---|
| Jetson Orin (JetPack 6 → Ubuntu 22.04 → ROS 2 **Humble**; 24.04 → Jazzy) | ROS 2 stack, rosbridge (wss :9090), HTTPS web server (:8443), optional Muse BLE streaming |
| MCU (Mega 2560 default; Uno / Teensy 4.1 / ESP32 supported) | `firmware/`: encoders, PID, motor driver, 200 ms watchdog, physical e-stop, walker load cells + grip |
| Meta Quest 3 | the WebXR app in the Quest browser (passthrough MR) |
| Muse 2/S + Oz cup electrode on the AUX input | streamed to LSL (`muselsl`) from the Jetson (BLE) or from a laptop/phone on the same network |

## 1. Jetson setup (once)
```bash
git clone <repo> ~/eyesteer && cd ~/eyesteer
bash deploy/jetson_setup.sh            # idempotent; --dry-run prints the plan
# log out / in once (dialout group)
```
It detects the Ubuntu version and picks the ROS distro, then installs:
- **ROS 2 and navigation:** ROS 2, Nav2, slam_toolbox, twist_mux, ros2_control(lers), rosbridge_suite, CycloneDDS.
- **Source builds:** `libserial-dev`, `sllidar_ros2` and `diffdrive_arduino` (branch `humble`).
- **Python packages:** `numpy<1.25` (Humble's scipy 1.8 needs it), pylsl, muselsl, pyserial, PlatformIO.

It then writes the udev rules, creates the certificates, installs the systemd units (they stay disabled) and builds
the workspace. From a laptop instead: `make deploy JETSON=user@eyesteer.local` (rsync + setup + build).

Docker alternative: `deploy/docker/Dockerfile` (`--build-arg BASE=dustynv/ros:humble-ros-base-l4t-r36.2.0` on the
Jetson) + `deploy/docker/compose.yaml` (host network, privileged for /dev and Bluetooth). Native is primary.

Network: Jetson and Quest on the same Wi-Fi (or the Jetson as a hotspot). `ROS_DOMAIN_ID=42`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. If DDS discovery over Wi-Fi is flaky, set a CycloneDDS peers list or use
Zenoh (Jazzy). The headset only talks websocket (rosbridge), so DDS never crosses Wi-Fi unless a laptop runs RViz.

## 2. USB devices (udev)
`deploy/udev/make_rules.py` writes `/etc/udev/rules.d/99-eyesteer.rules` from `hardware.yaml`:
`/dev/eyesteer_mcu` (mcu.usb_vid/pid) and `/dev/lidar` (lidar.usb_vid/pid). Find your IDs:
```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial' | head
```
Genuine Mega 2560: 2341:0042; CH340 clones: 1a86:7523; RPLIDAR (CP2102): 10c4:ea60. If two devices share a
VID/PID add `ATTRS{serial}=="…"` to the rule.

## 3. Firmware ("flash to the robot")
1. Wire the motor driver, encoders, e-stop and (walker) HX711 load cells + grip using the pins in
   `hardware.yaml → motor_driver.pins.<board>`. E-stop: a normally-closed button between the e-stop pin and GND
   (open = stop, so a cut wire also stops). Motor driver: PWM + DIR (Cytron MDD10A style). For an L298N use
   IN1 = DIR, IN2 = DIR through an inverter (one NPN stage), EN = PWM.
2. `make flash`. This builds for `mcu.board` (+ `_walker` if `robot: walker`) and uploads to `/dev/eyesteer_mcu`.
   `make firmware` builds every env (mega2560, uno, teensy41, esp32 and the walker variants).
3. The serial protocol (ROSArduinoBridge, as spoken by diffdrive_arduino, plus eyesteer extensions) is in
   INTERFACES.md §8.

## 4. Bring-up order (do not skip steps)
| # | step | command / check |
|---|---|---|
| 1 | **Wheels off the ground**, e-stop in reach | |
| 2 | Motor + encoder polarity | `python3 tools/motor_test.py --device /dev/eyesteer_mcu` → every line `OK`, watchdog `OK`. A wheel that runs backwards or counts down when going forward → flip `invert_*` / `invert_*_encoder` in hardware.yaml → `make generate flash` |
| 3 | Encoder counts per revolution | `python3 tools/motor_test.py --measure-cpr` (turn the wheel 10×) → `encoders.counts_per_rev` |
| 4 | Watchdog | `deploy/preflight.sh --only mcu` (motors must stop ≤ 200 ms after commands stop) |
| 5 | Teleop only | `ros2 launch eyesteer_bringup real.launch.py eeg:=synthetic require_headset:=false` (in another terminal) then `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop` |
| 6 | Odometry calibration | drive exactly 1 m straight: correct `wheels.radius` by (odom distance / true distance); rotate 360° in place: correct `wheels.separation` by (odom angle / 360°). Edit the robot YAML → `make generate` → rebuild |
| 7 | PID | defaults Kp 20 / Kd 12 / Ki 0 / Ko 50 (field), 30/10/0/50 (walker), ROSArduinoBridge convention. If the wheels oscillate lower Kp; if they lag raise Kp; add Ki (1–2) only for steady-state error under load |
| 8 | LiDAR pose | RViz (`rviz:=true`), fixed frame `odom`, face a flat wall: the scan line must be straight and at the measured distance; rotating in place must not smear the wall. Fix `lidar.x/y/yaw` in the robot YAML |
| 9 | Map the demo space | `ros2 launch eyesteer_bringup slam.launch.py sim:=false robot:=<robot>`, drive slowly with teleop, then `ros2 run nav2_map_server map_saver_cli -f ~/maps/demo`. The saved map's origin is where mapping started (that spot is (0, 0, 0)); later sessions give the robot's pose in this map with `start_pose:=x,y,yaw` or "2D Pose Estimate" (step 10) |
| 10 | Localization + Nav2 | `real.launch.py map:=$HOME/maps/demo.yaml start_pose:=x,y,yaw`, set "2D Pose Estimate" in RViz, send "Nav2 Goal"s |
| 11 | Mode manager + wander | field robot: WANDER starts once all gates are green; walker: ASSIST (push the handles) |
| 12 | Headset | open `https://<jetson>:8443` in the Quest browser, accept the certificate (also open `https://<jetson>:9090` once and accept), consent, calibrate, place markers (§5) |
| 13 | BCI with synthetic gaze | `real.launch.py eeg:=synthetic`: aim at a marker → the robot goes there (EEG out of the loop) |
| 14 | Real EEG | `eeg:=lsl`, `deploy/preflight.sh` (stream, alpha, SSVEP), per-user calibration in the headset (Run → "Per-user EEG calibration") |
| 15 | Full system | `make demo-real`: motion unlocks only when every gate is green; reduced speeds first (`nav2_real` uses 0.3 m/s) |

## 5. Headset ↔ map calibration and markers
- Print `deploy/calibration_decals.pdf` at 100 % (check the 100 mm scale bar). Stick FRONT and REAR on the top plate at
  the positions printed on the page (`calibration_decals` in the robot YAML).
- Robot localized and **stationary**. In the headset: Calibrate → touch the controller tip to FRONT, trigger; REAR,
  trigger. The residual must be < 5 cm (otherwise redo). The calibration lives for the session.
- Place markers: Place markers → aim at a real surface (hit-test reticle) → trigger. Labels come from the preset list
  (Beans, Bread, Milk, …); the slot/frequency is assigned automatically (CANCEL = the highest frequency). Markers are
  stored on the Jetson in `~/.eyesteer/markers/<map>.yaml` and reload on startup (also when the browser has no
  persistent anchors).

## 6. Muse + Oz electrode → LSL
- Oz cup electrode (Ag/AgCl, gel) on the AUX input of the Muse (as in the Eye-as-a-Camera setup), reference as wired
  for the Muse AUX.
- Stream: on the Jetson `muselsl stream --address <MAC> --aux` (BLE; `muselsl list` finds the MAC) or from a laptop
  on the same network (LSL crosses the LAN). Petal Metrics (`source_tool: petal`) also works; Mind Monitor needs an
  OSC→LSL bridge (`mind_monitor_bridge`). If you have the Eye-as-a-Camera acquisition code, drop it into
  `reference/eye_as_camera/` and set `source_tool: eye_as_camera`; only the stream name/channel map must match.
- Two Muse streams on one network (e.g. a laptop and the Jetson) → set `eeg.stream_host` to the producer's hostname.
- Checks: `python3 tools/eeg_check.py stream | alpha | ssvep` (each prints PASS/FAIL).

## 7. Preflight (before the robot moves)
```bash
bash deploy/preflight.sh            # ROS, MCU protocol + watchdog, LiDAR, TF, web/rosbridge, EEG stream/alpha/SSVEP, sim suite
bash deploy/preflight.sh --emulate  # rehearsal without hardware (MCU + Muse emulators)
```

## 8. systemd (optional autostart)
Installed but disabled. Start for a demo: `sudo systemctl start eyesteer-robot` (real.launch.py incl. rosbridge and the
web server). Logs: `journalctl -u eyesteer-robot -f`. Enable at boot: `sudo systemctl enable eyesteer-robot`.

## 9. Troubleshooting
| symptom | likely cause | fix |
|---|---|---|
| no `/scan` | `/dev/lidar` missing, wrong baud (C1 = 460800), motor not spinning | `ls -l /dev/lidar`, udev rule, `hardware.yaml lidar.model` |
| TF errors (`map`/`odom` missing) | AMCL not initialised / ros2_control not up | set the initial pose; check `ros2 control list_controllers` (diff_cont, joint_broad active) |
| robot spins instead of driving | one wheel's axis / motor polarity flipped | `tools/motor_test.py`; flip `invert_left/right` (motor) or `invert_*_encoder` |
| odometry drifts | wrong wheel radius/separation, wheel slip | §4 step 6; lower acceleration limits |
| rosbridge certificate not accepted | Quest refuses wss with an untrusted cert | open `https://<jetson>:9090` once in the Quest browser and accept; regenerate certs with the Jetson's IP (`deploy/certs/make_cert.sh <host> <ip>`) |
| refresh-rate mismatch warning | Quest display ≠ `headset.refresh_hz` | set the Quest refresh rate (Settings → Display) or change `refresh_hz` and `make generate`, reload the page |
| LSL stream not found | Muse not streaming / different name / firewall | `muselsl list/stream`, `tools/eeg_check.py stream`, `eeg.source_tool` |
| BAD SIGNAL | Oz electrode dry/off, AUX not mapped, line noise | re-gel Oz, check `eeg.channel_map`, `tools/eeg_check.py alpha`, keep away from chargers |
| Nav2 goal rejected / aborted | goal in an obstacle, robot not localized | marker too close to a wall (the goal is `approach_distance` in front of it); relocalize |
| motion locked | a red health gate | `https://<jetson>:8443/dashboard.html` shows which and why; `make demo-real` prints a hint per gate |
| ros2_control fails to load `diffdrive_arduino` | libserial not found | `export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH` (done by `deploy/env.sh`) |

## 10. Things only the real hardware can verify
See the final section of README.md for the list and the exact command for each check.
