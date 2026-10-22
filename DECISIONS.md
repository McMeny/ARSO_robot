# DECISIONS

One line per decision: **decision**: reason.

## Repository / environment
- **Repo root is `ARSO_robot/`** (the brief calls it `eye-steering/`): the workspace folder given was ARSO_robot; layout below it follows Section 6 exactly.
- **Dev environment = Windows 11 (Python 3.12, MuJoCo 3.4) + WSL Ubuntu 22.04 with ROS 2 Humble (MuJoCo 3.11, Python 3.10)**: Docker is unavailable; WSL gives a native Humble that matches JetPack 6 (Ubuntu 22.04) on the Jetson Orin.
- **MuJoCo `Renderer`: one per process on Windows**: a second Renderer in the same process renders black frames; scenario rendering spawns a process per video.

## Models (M1)
- **Frequency selection objective**: hard rules from D6 (R/n, 6–16 Hz, no 9.5–10.5 Hz, no 2nd/3rd-harmonic collisions ±0.15 Hz, ≥0.4 Hz spacing), then prefer most members in the 6.5–13 Hz band (strongest SSVEP), then max min-spacing. A pure max-min-spacing objective picks 6.0 Hz and does not reproduce the expected 90 Hz set; the preferred-band rule does, and is physiologically motivated.
- **72 Hz yields only 5 valid frequencies** (6↔12 and 7.2↔14.4 harmonic collisions): table is flagged `reduced: true` → 4 destinations + CANCEL. Recommend 90 or 120 Hz on the Quest 3.
- **CANCEL = highest frequency in the set**: deterministic rule shared by headset and decoder.
- **base_link on the drive axle, base_footprint on the floor**: standard diff-drive convention; walker's base_link is on the rear axle.
- **Wheel joint axes are +y on both wheels**: positive joint velocity on both wheels drives +x (tested in MJCF and the bridge).
- **MJCF casters use `priority="1"` and condim 1**: MuJoCo combines contact params by max; without priority the floor's friction makes casters drag (robot moved 15% of commanded distance).
- **Walker user is geom group 4, contype 0**: never collides physically or with LiDAR; collisions are measured with the user-inclusive footprint polygon instead.
- **Chase camera uses `mode="trackcom"` with a world-fixed south-west offset**: a body-fixed chase camera ends up behind walls.
- **Prebuilt sim maps are rasterized from the world geometry** (exact ground truth, 5 cm cells) and committed; `slam.launch.py` can rebuild them by mapping in sim.
- **diffdrive_arduino (joshnewans, branch `humble`) confirmed**: plugin `diffdrive_arduino/DiffDriveArduinoHardware`; params left_wheel_name, right_wheel_name, loop_rate, device, baud_rate, timeout_ms, enc_counts_per_rev, pid_p/d/i/o; commands `e\r` → "L R\r\n", `m L R\r` (counts per loop = rad/s ÷ rad_per_count ÷ loop_rate), `u p:d:i:o\r`. Needs libserial-dev.

## BCI (M2)
- **Default intent thresholds θ = 0.40, δ = 0.08, N = 3 (refractory 2 s)**: chosen from `outputs/metrics/bci_tradeoff.png` (synthetic, SNR 0.8 and 1.5): N ≥ 3 (anti-Midas-touch), ≤ 0.2 false activations/min at both SNRs, ≤ 3 s latency, then max accuracy → 97 % trials selected, 0.00 false/min, median latency 2.7 s. The brief's 0.35/4 gave 1.3 false/min at SNR 1.5.
- **A fixed θ cannot fit every user's SNR**: per-user calibration (Section 9) refits θ/δ/N from 10 s per marker + 20 s rest and is required (health gate) for real EEG.
- **Synthetic EEG SNR is defined as SSVEP fundamental amplitude = 2.5 µV × snr** over a ~6 µV pink background + alpha + 60 Hz + artifacts; CCA is at chance at snr 0 and ≥ 99 % per-window at snr 3 (tested).
- **Repeat selections of the attended marker are allowed after the 2 s refractory**: the FSM deduplicates them (current goal/queue), so continued looking is harmless.
- **Engagement gate default `soft`**: z > 1 lowers θ by 0.05, z < −1 raises it; `required` blocks selections below z_required.

## Control (M2)
- **Collision-monitor zones cover the front half of the footprint only** (shared function `monitor_polygon`, also written into the Nav2 collision_monitor params): a full ring deadlocks the robot beside walls/aisles and around the walker user.
- **Walker admittance publishes `cmd_vel_admittance` only in ASSIST (idle)**; in ENGAGED, handle forces act through a pull-back scale factor on the nav command (≥12 N back → slow, ≥30 N → stop) so light walking forces never preempt navigation, but pulling back always wins.
- **twist_mux gets a third lock, `lock/health`** (priority 253): the gated startup (Section 24.4) holds motion locked until all health gates are green.

## Standalone sim (M3)
- **Runtime C uses the ground-truth pose as its "AMCL" estimate**; encoder odometry is integrated separately and its drift is reported (map shows odometry vs ground truth).
- **Collision metric = footprint polygon (walker incl. user) vs world SDF + people circles** every frame (MuJoCo contacts are counted too); "collision" = clearance < 0 transition.
- **Simulated people are polite**: they pause rather than walk into the robot and give way (turn back) after 2 s; otherwise a person and a collision-monitor STOP deadlock forever, which says nothing about the robot.
- **Collision-monitor STOP zeroes rotation too** (as Nav2); a short Nav2-style back-up recovery + replan follows after 4 s.
- **Rotation guard for the walker**: in-place turns sweep the user behind the handles, which the front-only zones and the occluded rear LiDAR sector cannot see; a 2 s memory of LiDAR points is checked against the full footprint + 0.1 m before turning (Runtime C and ROS `safety` node). Found when the walker's user clipped a shopper in testing.
- **Walker planner margins**: lethal radius = inscribed + 0.15 m, planned footprint poses ≥ 0.15 m from obstacles (> stop margin 0.1 m), pure-pursuit lookahead 0.45 m (0.6 m front overhang cuts corners otherwise). Walker inflation 0.95 m centres it in 2.1 m aisles.
- **Supermarket layout**: checkout counter moved 0.4 m south and Produce/Checkout markers moved so the user-inclusive walker footprint fits every goal pose (verified by planning from four start points to every marker).
- **Videos: slow 0.5 Hz labeled pulse; `--true-flicker` opt-in with a warning** (30 fps cannot render 6–13 Hz faithfully anyway).
- **Marker labels hidden behind shelves are drawn faded and marked "(behind)"** (ray test); MR overlays in a real headset draw on top.

## Firmware (M4)
- **One portable C core (`firmware/lib/eyesteer_fw`) for the MCU, the host tests and the HIL emulator (ctypes)**: the emulator is not a port, it is the same code.
- **ROSArduinoBridge velocity-form PID, counts per loop at 30 Hz = diffdrive_arduino `loop_rate`**; acceleration limit on the setpoint.
- **'e' polling does not feed the watchdog**; only `m`/`o` motion commands do.
- **E-stop input normally-closed to GND (open = stop)**: a cut wire stops the robot; latched until `x` with the button released.
- **Walker sensors via a serial proxy (`mcu_bridge`)** instead of forking diffdrive_arduino: it owns the port, forwards diffdrive_arduino's lines verbatim through a pty and interleaves `s`/`h` queries (20 Hz). Cleaner than a custom hardware interface; also gives the MCU-link health signal.
- **Uno: field robot only** (walker needs more interrupt-capable pins). All 7 PlatformIO envs build (mega2560, uno, teensy41, esp32 + walker variants of mega/teensy/esp32).
- **HX711 read by a minimal bit-bang routine**, forces reported in decinewtons (AVR snprintf has no %f).

## ROS 2 (M5)
- **ROS 2 Humble (Ubuntu 22.04 / JetPack 6)**; Jazzy supported via `ROS_DISTRO`. Confirmed in WSL: nav2 (bringup, collision_monitor, RPP, navfn), slam_toolbox, ros2_control, xacro available as binaries; **twist_mux (branch humble), rosbridge_suite (branch humble), diffdrive_arduino (branch humble) + libserial built from source** without sudo by `deploy/build_deps_from_source.sh` (apt on the Jetson).
- **numpy pinned < 1.25 on Humble** (system scipy 1.8); numpy 2 breaks every scipy import. `cbor2` needed by rosbridge.
- **Synthetic/replayed EEG always goes through a real LSL stream (Muse emulator)** so `ssvep_node` has one code path; the honesty label comes from the launch argument (D18). Each launch gives its emulator a **unique LSL source_id** and the decoder pins it (a stray "Muse" stream on the network, e.g. a test on the same host, was decoded once in testing).
- **Real EEG: optional `eeg.stream_host`** pins the producer if two Muse streams are on the network.
- **LSL sample rate measured from sample timestamps** (wall-clock delivery jitter under CPU load made 256 ± 5 Hz flap).
- **`/scan` published RELIABLE** (compatible with SensorData best-effort subscribers and RViz).
- **`/amcl_pose` subscribed transient-local**: AMCL publishes it latched and only on motion; a late volatile subscriber never unlocked motion.
- **Localization-lost detection = scan-to-map consistency** (< 40 % of LiDAR hits near occupied map cells for 2 s) plus AMCL covariance: a kidnapped robot keeps a confident wrong AMCL pose.
- **Nav2 controller = Regulated Pure Pursuit, planner = NavFn (A*)**; walker BT = Nav2's replanning BT **without Spin** (spinning sweeps the user); sim vs real params differ in speed (0.45/0.4 vs 0.3 m/s) and inflation (+0.1 m).
- **Collision monitor (Humble params `max_points`, flat `points` list)** between twist_mux and the final watchdog; zones = front of the footprint (stop +0.1 m, slowdown +0.4 m → 30 %), narrow sides.
- **Prebuilt sim maps = rasterized world geometry**; `tests/integration/test_slam.py` maps the field with slam_toolbox in sim and checks it against them.
- **Headset link heartbeat = `/xr/timing` (2 Hz)**; 3 s silence = disconnected → field robot finishes its goal then HOLDs (no wander), walker PAUSEs.
- **Health gates**: startup requires all required gates; at runtime MCU/LiDAR/TF/Nav2/safety re-lock motion, while BCI/headset problems use the FSM fallbacks of 24.5 (a lock there would contradict "the current goal continues").
- **Latency instrumentation** on `/eyesteer/latency` (JSON events with wall time) so the headless headset client can measure the budget end to end.

## Headset app (M6)
- **WebXR + Three.js 0.160.0 + roslibjs 1.4.1, vendored** (the Jetson may be offline on demo day). Plain ES modules, no build step.
- **Desktop preview is a 2D top-down canvas** (works headless without WebGL; used by Playwright).
- **Square-wave flicker by frame count** `floor(frame / (n/2))` parity (odd n → 4/3 split), sinusoid optional (`?flicker=sinusoid`).
- **`?emulate_refresh=90` test hook**: a desktop browser runs at 60 Hz; the headless headset client reports the headset rate it emulates (and 72 Hz for the mismatch test). Flicker frames then follow the emulated clock.
- **Sim calibration default**: map origin placed relative to the user; real sessions use the two-decal calibration.
- **Anchors**: WebXR anchors used when present; the Jetson's `~/.eyesteer/markers/<map>.yaml` is the persistent store either way.
- **Node 20.18.1 (portable) and Playwright Chromium** used for tests on this dev machine.

## Deployment (M7)
- **Native install primary, Docker documented** (L4T base image on the Jetson).
- **systemd units installed disabled**; started explicitly for a demo.
- **Self-signed cert with SANs for hostname, .local, localhost and all local IPs**; rosbridge wss uses the same cert.
- **`jetson_setup.sh --dry-run`** verified on Ubuntu 22.04 (WSL); all shell scripts pass shellcheck.

## End-to-end integration (M7.5): what the HIL path exposed (and the fixes)
- **ros2_control is started by an event, not a timer** (`launch/hw_interface.py`): it launches only after `mcu_bridge` logs that the MCU link is up. A 4 s timer raced the bridge's start (slow Python start-up) and diffdrive_arduino died with `LibSerial::OpenFailed`. The same race would hit the real robot (Arduino reset wait), so `real.launch.py` uses the same gate.
- **The HIL MCU emulator is serviced from the physics loop, not a thread**: a second Python thread starved by the GIL answered diffdrive_arduino's `e` late; each miss blocks the ros2_control loop for the full 1 s read timeout, odometry stalls and AMCL loses the robot.
- **First-order motor lag (τ = 80 ms) between the emulated PWM and the MuJoCo wheel**: without motor/gearbox inertia, the PID's 30 Hz PWM steps hit MuJoCo's near-instant velocity actuator as torque pulses; the wheels skid and odometry heading drifts (robot went LOST). With the lag, wheel-speed ripple drops 5–10× and S1 passes in HIL.
- **Walker approach distance 1.0 m (was 0.8)**: at the goal the walker faces the shelf; turning around sweeps a 0.73 m radius about the rear axle, so the axle needs ≥ 0.83 m (sweep + 0.1 m guard). At 0.8 m the (correct) rotation guard blocked the turn and Nav2 aborted for lack of progress.
- **Walker progress-checker allowance 45 s** (field 20 s): the collision monitor's slowdown also scales rotation, so a 180° turn next to a shelf takes ~20 s; Nav2's progress checker only counts translation.
- **Sweep guard (generalized rotation guard)**: any turning motion is checked by predicting the footprint 0.3 s and 0.6 s ahead against remembered LiDAR points; a shopper behind the walker was clipped by the user zone while the walker arced into an aisle.
- **Supermarket shopper_1 crosses once (one-way path) in front of the walker, then waits by the wall**: the old ping-pong path made the shopper loiter head-on in the only aisle exit, which tested the shopper model, not the robot.
- **`scenario_player` applies the scenario's people schedule in ROS** (`/sim/people_cmd`), so ROS runs see the same crossings as Runtime C; per-scenario `ros:` overrides (duration, `skip_expect`) record where Nav2 legitimately differs (its costmap usually plans around the shopper before the reactive collision monitor acts).
- **Long ROS batches run inside one long-lived `wsl.exe`** (processes started from a WSL session die when it exits; PowerShell background jobs are capped at 600 s).
- **Encoder-count deadband compensation in `safety` (last step before `/cmd_vel`)**: diffdrive_arduino sends integer counts per PID loop; any wheel speed < 1 count/loop (walker: 0.14 rad/s at the wheel) is truncated to 0. A rotation scaled to 30 % by the collision monitor (0.048 rad/s) or RPP's first accel-limited step (0.05 rad/s) then never moves the robot; its measured speed stays 0 and the controller repeats the tiny command forever (found in HIL; would happen on the real robot). Non-zero wheel speeds are raised to 1.5 counts/loop.
- **RPP `max_angular_accel` 3.2 rad/s²**: RPP ramps from the *measured* speed; diff_drive_controller (which ramps from its previous command) still enforces the robot's real limits.
- **Walker approach distance 1.1 m**: the walker's front ends 0.5 m from the shelf, outside the 0.4 m slowdown zone; Nav2 Humble's collision monitor also scales in-place rotation, which made the turn-around at a goal take as long as the progress checker allows.
- **Soft engagement gate is one-sided (only raises θ when z < −1)**: lowering θ for an engaged user produced wrong selections at gaze onset (engagement rises while the window still holds a half-formed SSVEP): 2/150 wrong onsets with the two-sided gate vs 0/150 one-sided (offline, synthetic, SNR 1.0); one such false selection showed up in an HIL run.
- **Sweep-guard margin 0.05 m**: 0.1 m also caught static walls that Nav2's costmap already handles and vetoed legitimate turns into aisles; the guard is for what the costmap cannot see (a person behind the walker).
