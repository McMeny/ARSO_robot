# Scenario metrics

All EEG in these runs is **SIMULATED** (synthetic SSVEP through the same decoder). Runtime C = standalone MuJoCo + eyesteer_core + built-in planner; ROS sim = the deployable ROS 2 stack (Nav2, twist_mux, collision monitor, Muse LSL emulator, ...) against MuJoCo.

## Runtime C (standalone)

| ID | scenario | pass | selections (correct/false) | false/min (look-around) | mean latency s | visited (time-to-goal s) | path m | min clearance m | collisions | CM activations | mux share | odom drift m/m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | field_basic | PASS | 1/0 | – | 1.95 | C (22.75) | 16.56 | 0.145 | **0** | 1 | NAVIGATION 98%, NONE 2% | 0.13457 |
| S2 | field_queue | PASS | 3/0 | – | 2.86 | A (52.43), D (15.61) | 22.61 | 0.028 | **0** | 26 | NAVIGATION 97%, NONE 3% | 0.16621 |
| S3 | field_no_control | PASS | 0/0 | 0.0 | – | – | 22.95 | 0.151 | **0** | 0 | NAVIGATION 100% | 0.05046 |
| S4 | field_overrides | PASS | 2/0 | – | 2.69 | – | 13.61 | 0.476 | **0** | 0 | LOCK:ESTOP 16%, NAVIGATION 77%, TELEOP 7% | 0.04169 |
| S5 | supermarket_walker | PASS | 2/0 | – | 3.14 | Beans (29.34), Checkout (44.64) | 18.49 | 0.014 | **0** | 8 | LOCK:DEADMAN 2%, NAVIGATION 68%, NONE 30% | 0.01886 |
| S6 | supermarket_queue | PASS | 3/0 | – | 2.33 | Bread (33.49), Milk (18.19) | 15.34 | 0.156 | **0** | 3 | NAVIGATION 61%, NONE 39% | 0.00604 |

## ROS sim (mujoco backend)

| ID | scenario | pass | selections (correct/false) | false/min (look-around) | mean latency s | visited (time-to-goal s) | path m | min clearance m | collisions | CM activations | mux share | odom drift m/m |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 | field_basic | PASS | 1/0 | – | – | C (–) | 17.03 | 0.307 | **0** | 0 | NAVIGATION 97%, NONE 3% | – |
| S2 | field_queue | PASS | 2/0 | – | – | A (–), D (–) | 26.05 | 0.122 | **0** | 1 | NAVIGATION 96%, NONE 4% | – |
| S3 | field_no_control | PASS | 0/0 | 0.0 | – | – | 33.71 | 0.196 | **0** | 4 | NAVIGATION 100% | – |
| S4 | field_overrides | PASS | 2/0 | – | – | – | 18.93 | 0.091 | **0** | 6 | LOCK:ESTOP 16%, NAVIGATION 77%, NONE 0%, TELEOP 7% | – |
| S5 | supermarket_walker | FAIL | 3/0 | – | – | Beans (–) | 8.59 | 0.015 | **0** | 1 | LOCK:DEADMAN 2%, NAVIGATION 92%, NONE 7% | – |
| S6 | supermarket_queue | not run | | | | | | | | | | |

## BCI defaults (bci_tradeoff.png)

θ = 0.4, N = 3 → accuracy 0.97, false activations 0.00/min, median latency 2.69 s (synthetic, SNR 0.8; checked at SNR 1.5).

