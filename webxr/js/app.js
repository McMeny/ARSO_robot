// Headset app (Section 14): consent -> calibrate (headset <-> map) -> place markers -> run.
// WebXR immersive-ar (passthrough) on the Quest; desktop preview (2D map) everywhere else.
import { headsetToMap, mapToHeadset, solve, fromMsg } from "./calib.js";
import { FlickerScheduler, luminance } from "./flicker.js";
import { markerArrayMsg, msgToMarker, nextSlot, frequencyFor, xrTimingMsg } from "./messages.js";
import { RosLink, defaultUrl } from "./ros.js";
import { FrameTiming } from "./timing.js";

const SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"];
const $ = (id) => document.getElementById(id);

const S = {
  cfg: null, table: null, sched: null, ros: null,
  markers: [], status: null, bci: null, health: null, pose: null, poseIsTruth: false,
  cal: null, consent: false, mode: "run", xrSession: null, aimed: 0, cue: null,
  timing: new FrameTiming(), frame: 0, blocked: false, connected: false, placed: [], calPoints: [],
  selections: [], flickerMode: "square", latency: [], aimLog: [],
};

// ------------------------------------------------------------------ boot
async function boot() {
  [S.cfg, S.table] = await Promise.all([fetch("config/system.json").then((r) => r.json()),
                                        fetch("config/frequencies.json").then((r) => r.json())]);
  S.sched = new FlickerScheduler(S.table);
  const q = new URLSearchParams(location.search);
  if (q.get("flicker") === "sinusoid") S.flickerMode = "sinusoid";
  if (q.get("emulate_refresh")) {                 // TEST ONLY: headless headset client
    S.timing = new FrameTiming(240, parseFloat(q.get("emulate_refresh")));
    S.emulated = true;
  }
  const badge = $("eeg-badge");
  badge.textContent = S.cfg.eeg_label + (S.cfg.eeg === "synthetic" ? " (gaze-driven)" : "");
  badge.classList.toggle("real", S.cfg.eeg === "lsl");
  if (S.cfg.mode === "sim") {
    // sim: no physical co-location; put the map origin 2 m in front of the user
    S.cal = { yaw: Math.PI / 2, tx: 0, ty: 0, floorHeight: 0, residual: 0, simulated: true };
  }
  S.ros = new RosLink(defaultUrl(location, S.cfg.rosbridge_port), {
    onConnection: (ok) => { S.connected = ok; renderConn(); },
    onStatus: (m) => { S.status = m; renderStatus(); },
    onMarkers: (m) => { S.markers = m.markers.map(msgToMarker); renderMode(); },
    onBci: (m) => { S.bci = m; renderBci(); },
    onHealth: (m) => { S.health = m; renderHealth(); },
    onCalibration: (m) => { S.cal = fromMsg(m); renderMode(); },
    onCue: (c) => { S.cue = c >= -1 ? c : null; },
    onPose: (p, truth) => { if (truth || !S.poseIsTruth) { S.pose = p; S.poseIsTruth = !!truth; } },
    onSelection: (m) => { S.selections.push({ id: m.marker_id, label: m.label, t: Date.now() }); },
    onPeople: (p) => { S.people = p.map((q) => [q.position.x, q.position.y, q.position.z]); },
    onLatency: (e) => { S.latency.push({ ...e, rx: Date.now() / 1000 }); if (S.latency.length > 500) S.latency.shift(); },
  });
  $("consent-ok").onclick = () => { S.consent = true; $("consent").classList.add("hidden"); };
  $("warning-dismiss").onclick = () => $("warning").classList.add("hidden");
  $("warning-reload").onclick = () => location.reload();
  for (const b of document.querySelectorAll("#tabs button")) b.onclick = () => setMode(b.dataset.mode);
  $("preview").addEventListener("click", onPreviewClick);
  $("preview").addEventListener("mousemove", onPreviewMove);
  setupXrButton();
  setMode("run");
  setInterval(publishTiming, 500);
  setInterval(publishGaze, 100);
  requestAnimationFrame(previewLoop);
}

function setMode(m) {
  S.mode = m;
  for (const b of document.querySelectorAll("#tabs button")) b.classList.toggle("active", b.dataset.mode === m);
  renderMode();
}

// ------------------------------------------------------------------ ROS outputs
function publishTiming() {
  const st = S.timing.stats();
  S.ros.pub("/xr/timing", "eyesteer_interfaces/XrTiming", xrTimingMsg({
    ...st, tableHash: S.table.hash, calibrated: !!S.cal && !S.cal.simulated, residual: S.cal ? S.cal.residual : 0,
    markersPlaced: S.markers.length, consent: S.consent, mode: S.xrSession ? S.mode : "preview",
  }));
  $("timing").textContent = `${st.refreshHz.toFixed(0)} Hz · ${st.frameMsMean.toFixed(1)} ms · drop ${st.droppedPct.toFixed(1)}%`;
  // refresh mismatch -> frame-locked frequencies would be wrong: block run mode in the headset
  S.blocked = (!!S.xrSession || S.emulated) && S.sched.mismatch(st.refreshHz);
  if (S.blocked) {
    $("warning-text").textContent = `The display runs at ${st.refreshHz.toFixed(0)} Hz but the frequency table was computed for ` +
      `${S.table.refresh_hz} Hz. Set the headset to ${S.table.refresh_hz} Hz (or change headset.refresh_hz in hardware.yaml and run make generate).`;
    $("warning").classList.remove("hidden");
  }
}

// synthetic gaze: the marker the user aims at drives the SIMULATED EEG (sim demos only)
function publishGaze() {
  if (S.cfg.eeg !== "synthetic" || !S.consent) return;
  S.ros.pub("/bci/gaze", "std_msgs/Int32", { data: S.aimed | 0 });
}

// ------------------------------------------------------------------ desktop preview (2D map)
const view = { scale: 40, cx: 450, cy: 320 };
function fitView() {
  if (!S.markers.length) return;
  const xs = S.markers.map((m) => m.x), ys = S.markers.map((m) => m.y);
  if (S.pose) { xs.push(S.pose.position.x); ys.push(S.pose.position.y); }
  const c = $("preview");
  const w = Math.max(...xs) - Math.min(...xs) + 3, h = Math.max(...ys) - Math.min(...ys) + 3;
  view.scale = Math.min(c.width / w, c.height / h);
  view.mx = (Math.max(...xs) + Math.min(...xs)) / 2;
  view.my = (Math.max(...ys) + Math.min(...ys)) / 2;
}
function w2c(x, y) {
  const c = $("preview");
  return [c.width / 2 + (x - (view.mx || 0)) * view.scale, c.height / 2 - (y - (view.my || 0)) * view.scale];
}
function c2w(u, v) {
  const c = $("preview");
  return [(u - c.width / 2) / view.scale + (view.mx || 0), -(v - c.height / 2) / view.scale + (view.my || 0)];
}
function canvasXY(ev) {
  const c = $("preview"), r = c.getBoundingClientRect();
  return [(ev.clientX - r.left) * (c.width / r.width), (ev.clientY - r.top) * (c.height / r.height)];
}
function markerAt(u, v) {
  for (const m of S.markers) {
    const [a, b] = w2c(m.x, m.y);
    if (Math.abs(a - u) < 22 && Math.abs(b - v) < 22) return m;
  }
  return null;
}
function onPreviewMove(ev) {
  if (S.mode !== "run") return;
  const m = markerAt(...canvasXY(ev));
  $("preview").style.cursor = m ? "crosshair" : "default";
}
function onPreviewClick(ev) {
  const [u, v] = canvasXY(ev);
  if (S.mode === "run") {
    const m = markerAt(u, v);
    S.aimed = m ? m.id : 0;
  } else if (S.mode === "place") {
    const [x, y] = c2w(u, v);
    placeMarker({ x, y, z: 1.4 }, "map");
  }
}

function previewLoop(t) {
  S.timing.push(t);
  if (!S.xrSession) {
    S.frame = S.emulated ? Math.floor((t / 1000) * S.timing.emulateHz) : S.frame + 1;
    S.sched.frame = S.frame;
    drawPreview();
  }
  requestAnimationFrame(previewLoop);
}

function markerColor(m) {
  if (m.cancel) return "#8a8984";
  return SERIES[(parseInt(m.slot, 10) - 1) % SERIES.length] || "#8a8984";
}

function drawPreview() {
  const c = $("preview"), g = c.getContext("2d");
  const css = getComputedStyle(document.documentElement);
  g.fillStyle = css.getPropertyValue("--panel");
  g.fillRect(0, 0, c.width, c.height);
  if (!view.mx && S.markers.length) fitView();
  // grid (1 m)
  g.strokeStyle = css.getPropertyValue("--grid");
  g.lineWidth = 1;
  const [x0, y1] = c2w(0, 0), [x1, y0] = c2w(c.width, c.height);
  for (let x = Math.floor(x0); x <= x1; x++) { const [u] = w2c(x, 0); g.beginPath(); g.moveTo(u, 0); g.lineTo(u, c.height); g.stroke(); }
  for (let y = Math.floor(y0); y <= y1; y++) { const [, v] = w2c(0, y); g.beginPath(); g.moveTo(0, v); g.lineTo(c.width, v); g.stroke(); }
  const st = S.status;
  for (const m of S.markers) {
    const [u, v] = w2c(m.x, m.y);
    const n = S.sched.framesFor(m.slot);
    let lum = n ? luminance(S.frame, n, S.flickerMode, 0.25, 1.0) : 1;
    if (!S.consent) lum = 0.6;                              // no flicker before consent
    if (S.cue !== null && S.cue !== undefined && S.cue !== m.id) lum = 0.15;   // calibration: dim the others
    g.globalAlpha = lum;
    g.fillStyle = markerColor(m);
    g.fillRect(u - 16, v - 16, 32, 32);
    g.globalAlpha = 1;
    if (m.id === S.aimed) { g.strokeStyle = css.getPropertyValue("--ink"); g.lineWidth = 2; g.strokeRect(u - 21, v - 21, 42, 42); }
    if (st && st.goal_id === m.id) { g.strokeStyle = "#35d0ff"; g.lineWidth = 4; g.strokeRect(u - 25, v - 25, 50, 50); }
    const qi = st ? st.queue.indexOf(m.id) : -1;
    if (qi >= 0) {
      g.fillStyle = markerColor(m); g.beginPath(); g.arc(u + 22, v - 22, 11, 0, 2 * Math.PI); g.fill();
      g.fillStyle = "#fff"; g.font = "bold 13px system-ui"; g.textAlign = "center"; g.textBaseline = "middle"; g.fillText(String(qi + 1), u + 22, v - 22);
    }
    g.fillStyle = css.getPropertyValue("--ink"); g.font = "600 13px system-ui"; g.textAlign = "center"; g.textBaseline = "top";
    g.fillText(`${m.label} ${m.frequency.toFixed(2)} Hz`, u, v + 20);
  }
  if (S.pose) {
    const p = S.pose.position, q = S.pose.orientation;
    const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    const [u, v] = w2c(p.x, p.y);
    g.fillStyle = "#1baf7a"; g.beginPath(); g.arc(u, v, 9, 0, 2 * Math.PI); g.fill();
    g.strokeStyle = "#1baf7a"; g.lineWidth = 3; g.beginPath(); g.moveTo(u, v); g.lineTo(u + 20 * Math.cos(yaw), v - 20 * Math.sin(yaw)); g.stroke();
  }
  if (!S.consent) {
    g.fillStyle = css.getPropertyValue("--ink2"); g.font = "14px system-ui"; g.textAlign = "left"; g.textBaseline = "top";
    g.fillText("Flicker paused until the photosensitivity consent is accepted.", 12, 10);
  }
}

// ------------------------------------------------------------------ placement + calibration
function placeMarker(pos, frame, label, cancel = false, facing = null) {
  const list = S.markers.map((m) => ({ ...m }));
  const slot = nextSlot(list, S.table, cancel || label === "CANCEL");
  if (!slot) { alert("All frequency slots are used: delete a marker first."); return null; }
  const presets = S.cfg.presets;
  const lab = label || (slot === "CANCEL" ? "CANCEL" : presets[list.filter((m) => !m.cancel).length % presets.length]);
  let p = pos, f = facing ?? 0;
  if (frame === "headset") {
    if (!S.cal) { alert("Calibrate the headset to the robot first."); return null; }
    const mp = headsetToMap([pos.x, pos.y, pos.z], S.cal);
    p = { x: mp[0], y: mp[1], z: mp[2] };
  }
  const id = Math.max(0, ...list.map((m) => m.id)) + 1;
  list.push({ id, label: lab, slot, frequency: frequencyFor(slot, S.table), x: p.x, y: p.y, z: p.z, facing: f, cancel: slot === "CANCEL" });
  sendMarkers(list);
  return id;
}

function sendMarkers(list) {
  S.ros.pub("/xr/anchors", "eyesteer_interfaces/MarkerArray", markerArrayMsg(list, "map", S.table.hash));
  S.markers = list;
  renderMode();
}

async function computeCalibration(front, rear) {
  const P = (p) => ({ x: p[0], y: p[1], z: p[2] });
  const r = await S.ros.call("/xr/compute_calibration", "eyesteer_interfaces/ComputeHeadsetCalibration",
    { front: P(front), rear: P(rear), session: String(Date.now()) });
  if (r.calibration) S.cal = fromMsg(r.calibration);
  renderMode(r.message);
  return r;
}

// For the sim / tests: decal points as the headset would measure them, given the robot pose and a
// chosen (ground-truth) headset<->map transform.
function simulatedDecalTouch(truthCal) {
  const q = S.pose.orientation, p = S.pose.position;
  const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  const d = S.cfg.decals;
  const pt = (dx, dy) => [p.x + Math.cos(yaw) * dx - Math.sin(yaw) * dy, p.y + Math.sin(yaw) * dx + Math.cos(yaw) * dy, d.plate_height];
  return [mapToHeadset(pt(d.front.x, d.front.y), truthCal), mapToHeadset(pt(d.rear.x, d.rear.y), truthCal)];
}

// ------------------------------------------------------------------ panels
function renderConn() {
  const c = $("conn");
  c.textContent = S.connected ? "rosbridge: connected" : `rosbridge: reconnecting… (${S.ros ? S.ros.reconnects : 0})`;
  c.className = "pill " + (S.connected ? "good" : "bad");
}
function renderStatus() {
  const s = S.status;
  $("st-display").textContent = s.display;
  $("st-queue").textContent = "queue: " + (s.queue_labels.length ? s.queue_labels.map((l, i) => `${i + 1}:${l}`).join(", ") : "empty");
  $("st-mux").textContent = "mux: " + s.mux_source + (s.motion_locked ? " (motion locked by health gates)" : "");
  $("st-safety").textContent = "collision monitor: " + s.safety + (s.estop ? " · E-STOP" : "") + (s.deadman_locked ? " · DEAD-MAN" : "");
  $("st-reason").textContent = s.reason;
}
function renderBci() {
  const b = S.bci;
  $("bci-source").textContent = b.source + " EEG";
  $("bci-source").className = "badge" + (b.source === "REAL" ? " real" : "");
  const rows = b.labels.map((l, i) => {
    const r = b.rhos[i] || 0;
    return `<div class="row"><span>${l}</span><span class="track"><div class="bar" style="width:${Math.min(100, r * 100)}%"></div>` +
      `<div class="thr" style="left:${b.threshold * 100}%"></div></span><span>${r.toFixed(2)}</span></div>`;
  });
  $("bci-bars").innerHTML = rows.join("");
  $("bci-quality").textContent = `${b.quality} · ${b.sample_rate.toFixed(0)} Hz · dwell ${(b.dwell * 100).toFixed(0)}% · ` +
    (b.no_control ? "NO-CONTROL" : "attending") + (b.table_hash !== S.table.hash ? " · TABLE MISMATCH" : "");
}
function renderHealth() {
  const colors = ["var(--good)", "var(--warning)", "var(--critical)", "var(--muted)"];
  const icons = ["✓", "!", "✕", "…"];
  $("health-list").innerHTML = S.health.components.map((c) =>
    `<li><span class="dot" style="background:${colors[c.level]}" title="${icons[c.level]}"></span><span>${icons[c.level]} ${c.name}${c.required ? "" : " (opt)"}</span>` +
    `<span class="muted">${c.message}</span></li>`).join("") +
    `<li><span></span><strong>${S.health.motion_unlocked ? "motion unlocked" : "MOTION LOCKED"}</strong><span></span></li>`;
}
function renderMode(msg) {
  const body = $("mode-body"), title = $("mode-title");
  if (S.mode === "calibrate") {
    title.textContent = "Calibrate headset ↔ robot map";
    body.innerHTML = `<p class="muted">Robot localized and stationary. In MR: touch the controller to the FRONT decal, pull the trigger,
      then the REAR decal. Residual must be &lt; 5 cm.</p><p>${S.cal ? `residual ${(S.cal.residual * 100).toFixed(1)} cm` +
      (S.cal.simulated ? " (sim default)" : "") : "not calibrated"}</p><p class="muted">${msg || ""}</p>
      <button id="cal-sim">Simulate decal touch (sim)</button>`;
    $("cal-sim").onclick = async () => {
      if (!S.pose) return;
      const [f, r] = simulatedDecalTouch({ yaw: 0.4, tx: 1.0, ty: -0.5, floorHeight: 0.0 });
      await computeCalibration(f, r);
    };
  } else if (S.mode === "place") {
    title.textContent = "Place markers";
    body.innerHTML = `<p class="muted">MR: aim at a real surface and pull the trigger. Preview: click on the map.</p>
      <ul>${S.markers.map((m) => `<li>${m.label} · slot ${m.slot} · ${m.frequency.toFixed(2)} Hz <button data-del="${m.id}">delete</button></li>`).join("")}</ul>`;
    for (const b of body.querySelectorAll("button[data-del]"))
      b.onclick = () => sendMarkers(S.markers.filter((m) => m.id !== parseInt(b.dataset.del, 10)));
  } else {
    title.textContent = "Run";
    body.innerHTML = `<p class="muted">Look at a marker to send the robot there. CANCEL clears the goal and the queue.</p>
      <button id="estop" style="background:var(--critical);color:#fff">E-STOP</button>
      <button id="calib-bci">Per-user EEG calibration</button>`;
    $("estop").onclick = () => S.ros.pub("/estop", "std_msgs/Bool", { data: true });
    $("calib-bci").onclick = () => S.ros.call("/bci/calibrate", "eyesteer_interfaces/Calibrate", { user: "default", synthetic: S.cfg.eeg !== "lsl" })
      .then((r) => renderMode(r.message));
  }
}

// ------------------------------------------------------------------ WebXR (immersive-ar passthrough)
async function setupXrButton() {
  const btn = $("enter-xr");
  if (!navigator.xr) { btn.title = "WebXR not available (desktop preview)"; return; }
  const ok = await navigator.xr.isSessionSupported("immersive-ar").catch(() => false);
  btn.disabled = !ok;
  btn.onclick = () => (S.consent ? startXr() : alert("Accept the photosensitivity warning first."));
}

async function startXr() {
  const THREE = await import("../vendor/three.module.js");
  const session = await navigator.xr.requestSession("immersive-ar", {
    requiredFeatures: ["local-floor"],
    optionalFeatures: ["hit-test", "anchors", "dom-overlay"],
    domOverlay: { root: $("overlay-root") },
  });
  S.xrSession = session;
  if (session.updateTargetFrameRate && session.supportedFrameRates && [...session.supportedFrameRates].includes(S.table.refresh_hz)) {
    session.updateTargetFrameRate(S.table.refresh_hz).catch(() => {});
  }
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.xr.enabled = true;
  renderer.xr.setReferenceSpaceType("local-floor");
  document.body.appendChild(renderer.domElement);
  await renderer.xr.setSession(session);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera();
  const meshes = new Map();
  const size = S.cfg.marker_size;
  const ray = new THREE.Raycaster();
  const ctrl = renderer.xr.getController(0);
  scene.add(ctrl);
  const refSpace = renderer.xr.getReferenceSpace();
  let hitSource = null;
  if (session.requestHitTestSource) {
    session.requestReferenceSpace("viewer").then((vs) => session.requestHitTestSource({ space: vs })).then((s) => (hitSource = s)).catch(() => {});
  }
  const reticle = new THREE.Mesh(new THREE.RingGeometry(0.06, 0.08, 32).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  reticle.visible = false;
  scene.add(reticle);
  // head-locked state panel
  const panelCanvas = document.createElement("canvas");
  panelCanvas.width = 512; panelCanvas.height = 160;
  const panelTex = new THREE.CanvasTexture(panelCanvas);
  const panel = new THREE.Mesh(new THREE.PlaneGeometry(0.5, 0.156), new THREE.MeshBasicMaterial({ map: panelTex, transparent: true }));
  panel.position.set(0, -0.28, -1.0);
  camera.add(panel);
  scene.add(camera);

  function labelTexture(text, color) {
    const c = document.createElement("canvas"); c.width = 512; c.height = 96;
    const g = c.getContext("2d");
    g.fillStyle = "rgba(20,20,19,0.8)"; g.fillRect(0, 0, 512, 96);
    g.strokeStyle = color; g.lineWidth = 6; g.strokeRect(3, 3, 506, 90);
    g.fillStyle = "#fff"; g.font = "bold 44px system-ui"; g.textAlign = "center"; g.textBaseline = "middle"; g.fillText(text, 256, 50);
    return new THREE.CanvasTexture(c);
  }
  function syncMeshes() {
    for (const m of S.markers) {
      if (!S.cal) break;
      let o = meshes.get(m.id);
      if (!o) {
        const mat = new THREE.MeshBasicMaterial({ color: markerColor(m), transparent: true });
        const quad = new THREE.Mesh(new THREE.PlaneGeometry(size, size), mat);
        const lab = new THREE.Mesh(new THREE.PlaneGeometry(size * 1.4, size * 0.26), new THREE.MeshBasicMaterial({ map: labelTexture(m.label, markerColor(m)), transparent: true }));
        lab.position.y = size * 0.75;
        const ring = new THREE.Mesh(new THREE.RingGeometry(size * 0.75, size * 0.82, 48), new THREE.MeshBasicMaterial({ color: 0x35d0ff, transparent: true, opacity: 0 }));
        const grp = new THREE.Group();
        grp.add(quad, lab, ring);
        grp.userData = { id: m.id, quad, ring };
        quad.userData.id = m.id;
        scene.add(grp);
        o = grp;
        meshes.set(m.id, o);
      }
      const h = mapToHeadset([m.x, m.y, m.z], S.cal);
      o.position.set(h[0], h[1], h[2]);
    }
    for (const [id, o] of meshes) if (!S.markers.find((m) => m.id === id)) { scene.remove(o); meshes.delete(id); }
  }
  ctrl.addEventListener("select", () => {
    const pos = new THREE.Vector3().setFromMatrixPosition(ctrl.matrixWorld);
    if (S.mode === "calibrate") {
      S.calPoints.push([pos.x, pos.y, pos.z]);
      if (S.calPoints.length === 2) { computeCalibration(S.calPoints[0], S.calPoints[1]); S.calPoints = []; }
    } else if (S.mode === "place" && reticle.visible) {
      const p = new THREE.Vector3().setFromMatrixPosition(reticle.matrix);
      const cam = renderer.xr.getCamera();
      const yawXr = Math.atan2(-(cam.position.x - p.x), -(cam.position.z - p.z));
      const facingMap = S.cal ? Math.atan2(Math.sin(yawXr + S.cal.yaw), Math.cos(yawXr + S.cal.yaw)) : 0;
      placeMarker({ x: p.x, y: p.y + 0.3, z: p.z }, "headset", null, false, facingMap);
    }
  });
  session.addEventListener("end", () => { S.xrSession = null; renderer.setAnimationLoop(null); renderer.domElement.remove(); });

  renderer.setAnimationLoop((t, frame) => {
    S.timing.push(t);
    S.frame += 1;                     // the flicker clock: one tick per displayed XR frame
    S.sched.frame = S.frame;
    syncMeshes();
    const cam = renderer.xr.getCamera();
    const st = S.status;
    for (const [id, o] of meshes) {
      const m = S.markers.find((x) => x.id === id);
      o.lookAt(cam.position);         // billboard towards the user
      const n = S.sched.framesFor(m.slot);
      let lum = S.blocked || !S.consent ? 0.5 : (n ? luminance(S.frame, n, S.flickerMode) : 1);
      if (S.cue !== null && S.cue !== undefined && S.cue !== id) lum = 0.1;
      o.userData.quad.material.opacity = 0.15 + 0.85 * lum;
      o.userData.ring.material.opacity = st && st.goal_id === id ? 1 : (st && st.queue.includes(id) ? 0.5 : 0);
    }
    // aiming: controller ray, else head gaze
    ray.setFromCamera({ x: 0, y: 0 }, cam);
    if (ctrl.visible) { ray.ray.origin.setFromMatrixPosition(ctrl.matrixWorld); ray.ray.direction.set(0, 0, -1).applyQuaternion(ctrl.quaternion); }
    const hits = ray.intersectObjects([...meshes.values()].map((o) => o.userData.quad));
    S.aimed = hits.length ? hits[0].object.userData.id : 0;
    if (frame && hitSource && S.mode === "place") {
      const res = frame.getHitTestResults(hitSource);
      reticle.visible = res.length > 0;
      if (res.length) reticle.matrix.fromArray(res[0].getPose(refSpace).transform.matrix);
      reticle.matrixAutoUpdate = false;
    } else reticle.visible = false;
    if (S.frame % 15 === 0 && st) {
      const g = panelCanvas.getContext("2d");
      g.clearRect(0, 0, 512, 160); g.fillStyle = "rgba(20,20,19,0.75)"; g.fillRect(0, 0, 512, 160);
      g.fillStyle = "#fff"; g.font = "bold 36px system-ui"; g.fillText(st.display, 16, 48);
      g.font = "24px system-ui"; g.fillStyle = "#ddd";
      g.fillText(`queue: ${st.queue_labels.join(", ") || "–"}   ${st.mux_source}   ${st.safety}`, 16, 92);
      g.fillStyle = "#ff6b6b"; g.fillText(S.cfg.eeg_label, 16, 136);
      panelTex.needsUpdate = true;
    }
    renderer.render(scene, camera);
  });
}

// ------------------------------------------------------------------ test hooks (Playwright headless headset client)
window.eyesteer = {
  state: () => ({ blocked: S.blocked, connected: S.connected, consent: S.consent, mode: S.mode, markers: S.markers, status: S.status,
                  bci: S.bci, health: S.health, cal: S.cal, aimed: S.aimed, frame: S.frame, selections: S.selections,
                  tableHash: S.table && S.table.hash, refresh: S.timing.snappedHz(), latency: S.latency, aimLog: S.aimLog,
                  pose: S.pose, people: S.people || [] }),
  pub: (topic, type, msg) => S.ros.pub(topic, type, msg),
  placeHeadsetFrame: (list) => {     // markers given in map coordinates, sent as the headset would: in its own frame
    const hs = list.map((m) => { const h = mapToHeadset([m.x, m.y, m.z], S.cal); return { ...m, x: h[0], y: h[1], z: h[2],
      facing: Math.atan2(Math.sin(m.facing - S.cal.yaw), Math.cos(m.facing - S.cal.yaw)), frequency: frequencyFor(m.slot, S.table) }; });
    S.ros.pub("/xr/anchors", "eyesteer_interfaces/MarkerArray", markerArrayMsg(hs, "headset", S.table.hash));
  },
  disconnect: () => { S.ros.ros.close(); clearTimeout(S.ros.timer); S.ros.connect = () => {}; },
  consent: () => $("consent-ok").click(),
  setMode,
  aim: (id) => { S.aimed = id; S.aimLog.push({ id, t: Date.now() }); },
  placeMarkers: (list) => sendMarkers(list.map((m) => ({ ...m, frequency: frequencyFor(m.slot, S.table) }))),
  placeAt: (x, y, z, label, cancel) => placeMarker({ x, y, z }, "map", label, cancel),
  calibrateSim: (truth) => computeCalibration(...simulatedDecalTouch(truth)),
  isOnForSlot: (slot, frame) => luminance(frame, S.sched.framesFor(slot), "square") > 0.5,
};

boot().catch((e) => { document.body.insertAdjacentHTML("afterbegin", `<pre style="color:red">boot failed: ${e}</pre>`); });
