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
