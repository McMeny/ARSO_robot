// Marker <-> ROS message serialization (rosbridge JSON shapes; see INTERFACES.md).
// eyesteer_interfaces/Marker: {id, label, slot (string), frequency, position {x,y,z}, facing, cancel}
export function markerToMsg(m) {
  return {
    id: m.id | 0,
    label: String(m.label),
    slot: String(m.slot),
    frequency: Number(m.frequency) || 0,
    position: { x: m.x, y: m.y, z: m.z },
    facing: m.facing || 0,
    cancel: Boolean(m.cancel || String(m.slot) === "CANCEL"),
  };
}

export function msgToMarker(msg) {
  return {
    id: msg.id,
    label: msg.label,
    slot: msg.slot,
    frequency: msg.frequency,
    x: msg.position.x,
    y: msg.position.y,
    z: msg.position.z,
    facing: msg.facing,
    cancel: msg.cancel,
  };
}

// eyesteer_interfaces/MarkerArray; frameId "headset" (WebXR local-floor) or "map"
export function markerArrayMsg(markers, frameId, tableHash) {
  return {
    header: { frame_id: frameId, stamp: { sec: 0, nanosec: 0 } },
    markers: markers.map(markerToMsg),
    table_hash: tableHash,
  };
}

// eyesteer_interfaces/XrTiming
export function xrTimingMsg(t) {
  return {
    refresh_hz: t.refreshHz,
    frame_ms_mean: t.frameMsMean,
    frame_ms_std: t.frameMsStd,
    dropped_pct: t.droppedPct,
    table_hash: t.tableHash,
    calibrated: Boolean(t.calibrated),
    calibration_residual: t.residual ?? 0,
    markers_placed: t.markersPlaced | 0,
    consent: Boolean(t.consent),
    mode: t.mode,
  };
}

// Assign the next free frequency slot (1..5, CANCEL reserved) for a newly placed marker.
export function nextSlot(markers, table, wantCancel = false) {
  if (wantCancel) return "CANCEL";
  const used = new Set(markers.map((m) => String(m.slot)));
  for (const s of table.slots) {
    if (s.cancel) continue;
    if (!used.has(String(s.slot))) return String(s.slot);
  }
  return null;
}

export function frequencyFor(slot, table) {
  const s = table.slots.find((x) => String(x.slot) === String(slot));
  return s ? s.frequency_exact : 0;
}
