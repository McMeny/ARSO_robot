# The walker's grip state - both handles, after per-user calibration.
# STOP: either hand released (no motion). WALK: both held. STEER: both squeezed firmly (head steers).
std_msgs/Header header
float32 left        # normalised 0..1 after calibration
float32 right
uint8 STOP=0
uint8 WALK=1
uint8 STEER=2
uint8 state
