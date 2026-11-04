# The walker's short-term path for the floor display (map frame, on the floor).
std_msgs/Header header
geometry_msgs/Point[] centre
geometry_msgs/Point[] left_edge
geometry_msgs/Point[] right_edge
float32 extra_clearance    # metres added to each side for a missed hazard (0 = normal)
float32 machine_authority  # 0 = the user's head steers, 1 = the machine is fully in charge
