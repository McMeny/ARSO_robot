# /robot/status: shown in the headset overlay, RViz and the HUD (HI observability)
builtin_interfaces/Time stamp
string robot             # field_robot | walker
string state             # IDLE ENGAGED ARRIVED PAUSED HOLD LOST ESTOP
string idle_mode         # WANDER | ASSIST
string display
int32 goal_id            # -1 = none
string goal_label
int32[] queue
string[] queue_labels
string mux_source        # NAVIGATION TELEOP ADMITTANCE LOCK:ESTOP LOCK:DEADMAN LOCK:HEALTH NONE
string safety            # collision monitor: OK SLOWDOWN STOP
bool estop
bool deadman_locked
bool motion_locked
string reason
string eeg_source        # SIMULATED | REPLAYED | REAL
