# A virtual SSVEP marker, anchored in the MAP frame (position = anchored point on the real world)
int32 id
string label
string slot              # "1".."5" or "CANCEL"
float32 frequency
geometry_msgs/Point position
float32 facing           # yaw (rad) of the panel normal = approach direction
bool cancel
