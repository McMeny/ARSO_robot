# headset (WebXR local-floor) -> map 2D rigid transform: p_map = R(yaw) * p_headset_xz + t
builtin_interfaces/Time stamp
float64 yaw
float64 tx
float64 ty
float64 floor_height     # headset floor y -> map z offset
float32 residual         # m; redo if > 0.05
string session
