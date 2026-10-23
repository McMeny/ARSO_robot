# Scenario metrics

All EEG in these runs is **SIMULATED** (synthetic SSVEP through the same decoder). Runtime C = standalone MuJoCo + eyesteer_core + built-in planner; ROS sim = the deployable ROS 2 stack (Nav2, twist_mux, collision monitor, Muse LSL emulator, ...) against MuJoCo.

## Runtime C (standalone)

| ID | scenario | pass | selections (correct/false) | false/min (look-around) | mean latency s | visited (time-to-goal s) | path m | min clearance m | collisions | CM activations | mux share | odom drift m/m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | field_basic | PASS | 1/0 | – | 1.95 | C (22.75) | 16.56 | 0.145 | **0** | 1 | NAVIGATION 98%, NONE 2% | 0.13457 |
| S2 | field_queue | PASS | 3/0 | – | 2.86 | A (52.43), D (15.61) | 22.61 | 0.028 | **0** | 26 | NAVIGATION 97%, NONE 3% | 0.16621 |
| S3 | field_no_control | PASS | 0/0 | 0.0 | – | – | 22.95 | 0.151 | **0** | 0 | NAVIGATION 100% | 0.05046 |
| S4 | field_overrides | PASS | 2/0 | – | 2.69 | – | 13.61 | 0.476 | **0** | 0 | LOCK:ESTOP 16%, NAVIGATION 77%, TELEOP 7% | 0.04169 |
| S5 | supermarket_walker | PASS | 2/0 | – | 3.14 | Beans (33.22), Checkout (36.24) | 18.25 | 0.086 | **0** | 4 | LOCK:DEADMAN 2%, NAVIGATION 64%, NONE 34% | 0.01977 |
| S6 | supermarket_queue | PASS | 3/0 | – | 2.33 | Bread (36.21), Milk (20.57) | 15.47 | 0.203 | **0** | 4 | NAVIGATION 67%, NONE 33% | 0.0055 |

## ROS sim (mujoco backend)

| ID | scenario | pass | selections (correct/false) | false/min (look-around) | mean latency s | visited (time-to-goal s) | path m | min clearance m | collisions | CM activations | mux share | odom drift m/m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | field_basic | PASS | 1/0 | – | 1.83 | C (22.41) | 20.39 | 0.509 | **0** | 0 | NAVIGATION 97%, NONE 3% | – |
| S2 | field_queue | PASS | 2/0 | – | 2.19 | A (21.0), D (19.14) | 34.57 | 0.149 | **0** | 2 | NAVIGATION 97%, NONE 3% | – |
| S3 | field_no_control | PASS | 0/0 | 0.0 | – | – | 30.12 | 0.128 | **0** | 4 | NAVIGATION 100%, NONE 0% | – |
| S4 | field_overrides | PASS | 2/0 | – | 2.36 | – | 18.96 | 0.376 | **0** | 2 | LOCK:ESTOP 16%, NAVIGATION 77%, TELEOP 7% | – |
| S5 | supermarket_walker | PASS | 2/0 | – | 3.10 | Beans (29.91), Checkout (82.78) | 17.91 | 0.128 | **0** | 2 | LOCK:DEADMAN 1%, NAVIGATION 56%, NONE 43% | – |
| S6 | supermarket_queue | PASS | 2/0 | – | 2.25 | Bread (34.03), Milk (16.65) | 15.27 | 0.132 | **0** | 0 | NAVIGATION 40%, NONE 60% | – |

## ROS HIL (mujoco_hil backend)

| ID | scenario | pass | selections (correct/false) | false/min (look-around) | mean latency s | visited (time-to-goal s) | path m | min clearance m | collisions | CM activations | mux share | odom drift m/m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | field_basic | PASS | 1/0 | – | 2.36 | C (25.26) | 20.29 | 0.187 | **0** | 2 | NAVIGATION 97%, NONE 3% | – |
| S2 | field_queue | PASS | 2/0 | – | 2.51 | A (25.12), D (14.27) | 30.89 | 0.149 | **0** | 4 | NAVIGATION 97%, NONE 3% | – |
| S3 | field_no_control | PASS | 0/0 | 0.0 | – | – | 32.13 | 0.128 | **0** | 4 | NAVIGATION 100%, NONE 0% | – |
| S4 | field_overrides | PASS | 2/0 | – | 2.30 | – | 13.21 | 0.151 | **0** | 0 | LOCK:ESTOP 36%, LOCK:HEALTH 2%, NAVIGATION 55%, TELEOP 7% | – |
| S5 | supermarket_walker | PASS | 3/0 | – | 2.20 | Beans (31.87), Checkout (42.43) | 17.67 | 0.151 | **0** | 1 | LOCK:DEADMAN 1%, NAVIGATION 36%, NONE 62% | – |
| S6 | supermarket_queue | PASS | 2/0 | – | 2.24 | Bread (44.72), Milk (18.8) | 15.3 | 0.149 | **0** | 2 | NAVIGATION 49%, NONE 51% | – |

## BCI defaults (bci_tradeoff.png)

θ = 0.4, N = 3 → accuracy 0.95, false activations 0.00/min, median latency 2.58 s (synthetic, SNR 0.8; checked at SNR 1.5).

## Full-system golden path: field_robot

5/5 consecutive passes. Latency budget (median):

| segment | measured | target |
|---|---|---|
| gaze onset -> selection (s) | 2.265 | 1.25-3.0 |
| selection -> NavigateToPose accepted (ms) | 89.615 | 0-200 |
| goal accepted -> robot moving (ms) | 142.831 | 0-1000 |
| cmd_vel -> wheel response, HIL (ms) | 89.804 | 0-100 |
| robot state change -> visible in headset (ms) | 5.558 | 0-300 |

## Full-system golden path: walker

5/5 consecutive passes. Latency budget (median):

| segment | measured | target |
|---|---|---|
| gaze onset -> selection (s) | 2.578 | 1.25-3.0 |
| selection -> NavigateToPose accepted (ms) | 86.365 | 0-200 |
| goal accepted -> robot moving (ms) | 263.067 | 0-1000 |
| cmd_vel -> wheel response, HIL (ms) | 76.764 | 0-100 |
| robot state change -> visible in headset (ms) | 6.707 | 0-300 |

