# What the tentacle planner is doing, for the headset and the logs.
std_msgs/Header header
uint8 grip_state          # GripState.STOP / WALK / STEER
float32 authority         # 0 = machine steers, 1 = user (head) steers
float32 desired_yaw       # rad, odom frame
bool turning_in_place
bool blocked              # no tentacle is clear
bool caution              # an unregistered (or no-EEG) hazard is close ahead: slower, wider berth
string hint               # e.g. "Turn left? Squeeze to steer" - empty if none
string notice             # e.g. "Path blocked. Squeeze to steer out" - empty if none
