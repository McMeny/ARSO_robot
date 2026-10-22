// Headset (WebXR local-floor) <-> robot map calibration. Mirror of eyesteer_core/xr_calibration.py.
// WebXR: x right, y up, z towards the viewer. Ground plane (u, v) = (-z, -x) is a right-handed
// (forward, left) pair, so headset -> map is a 2D rotation + translation; z_map = y_xr - floorHeight.
export function xrPlane(p) {
  return [-p[2], -p[0]];
}

export function decalInMap(pose, decal) {
  const [x, y, yaw] = pose;
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [x + c * decal[0] - s * decal[1], y + s * decal[0] + c * decal[1]];
}

export function headsetToMap(p, cal) {
  const [u, v] = xrPlane(p);
  const c = Math.cos(cal.yaw), s = Math.sin(cal.yaw);
  return [c * u - s * v + cal.tx, s * u + c * v + cal.ty, p[1] - cal.floorHeight];
}

export function mapToHeadset(p, cal) {
  const c = Math.cos(cal.yaw), s = Math.sin(cal.yaw);
  const dx = p[0] - cal.tx, dy = p[1] - cal.ty;
  const u = c * dx + s * dy, v = -s * dx + c * dy;
  return [-v, p[2] + cal.floorHeight, -u];
}

export function yawHeadsetToMap(yawXr, cal) {
  const a = yawXr + cal.yaw;
  return Math.atan2(Math.sin(a), Math.cos(a));
}

// Two-point co-location (front + rear decal touched with the controller).
export function solve(hFront, hRear, robotPose, decalFront, decalRear, plateHeight) {
  const hf = xrPlane(hFront), hr = xrPlane(hRear);
  const df = decalInMap(robotPose, decalFront), dr = decalInMap(robotPose, decalRear);
  let yaw = Math.atan2(df[1] - dr[1], df[0] - dr[0]) - Math.atan2(hf[1] - hr[1], hf[0] - hr[0]);
  yaw = Math.atan2(Math.sin(yaw), Math.cos(yaw));
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const hm = [(hf[0] + hr[0]) / 2, (hf[1] + hr[1]) / 2];
  const dm = [(df[0] + dr[0]) / 2, (df[1] + dr[1]) / 2];
  const cal = {
    yaw,
    tx: dm[0] - (c * hm[0] - s * hm[1]),
    ty: dm[1] - (s * hm[0] + c * hm[1]),
    floorHeight: (hFront[1] + hRear[1]) / 2 - plateHeight,
    residual: 0,
  };
  const lenH = Math.hypot(hf[0] - hr[0], hf[1] - hr[1]);
  const lenD = Math.hypot(df[0] - dr[0], df[1] - dr[1]);
  let err = 0;
  for (const [h, d] of [[hFront, df], [hRear, dr]]) {
    const m = headsetToMap(h, cal);
    err = Math.max(err, Math.hypot(m[0] - d[0], m[1] - d[1]));
  }
  cal.residual = Math.max(Math.abs(lenH - lenD), err, Math.abs(hFront[1] - hRear[1]));
  return cal;
}

// From the ROS HeadsetCalibration message (snake_case) to the JS object.
export function fromMsg(m) {
  return { yaw: m.yaw, tx: m.tx, ty: m.ty, floorHeight: m.floor_height, residual: m.residual };
}
