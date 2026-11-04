# The fan of candidate arcs ("tentacles"), from hard left to hard right, and how far each is clear.
std_msgs/Header header
float32[] curvatures     # 1/m, + = left
float32[] free_lengths   # metres along each arc before it hits something (= length if clear)
float32 length           # how far each arc was checked (m)
float32 half_width       # half the corridor width checked around each arc (m)
float32 clearance        # free length straight ahead (m) - the risk measure for shared steering
