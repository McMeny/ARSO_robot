// The participant's view. In the Quest: tap Enter VR (WebXR, full VR). On a PC with no
// headset: drag the mouse to look around (desktop mode).
//
// 1:1 scale, real eye height (floor-level tracking, no camera height is hard-coded in VR).
// Two modes (vr_mode, from the launch file):
//   room_scale  the sim world is anchored to the real room: the user walks along with the sim
//               walker in a cleared space. Recentre puts the walker's handles at the user's hands.
//               Head within boundary_stop_margin of the Guardian edge, or further than
//               max_user_handle_distance from the handles -> STOP ("Room edge reached").
//   stationary  the user stands still; the view moves with the walker. The view is never turned
//               for the user - turning comes only from the user's own body (squeeze to steer
//               then pivots the walker to match). Optional comfort vignette while moving.
// The Quest controllers' grip triggers are the handles (sent ~50 Hz -> grip_state_node).
// Hold A or X for 1 s to recentre / reset. Head pose goes to ROS 20 times a second.
import * as THREE from '../vendor/three.module.js';
import { connect, rosToThree, threeToRos, rosHeading, makeScene, addWorld, makeWalker, placeWalker,
         HazardTags, Trajectory, TestPatch, TextPanel } from './common.js';

const DESKTOP_EYE_HEIGHT = 1.6;   // desktop mode only - in VR the real eye height is used
const HANDLE_X = -0.1;            // m, handles behind base_footprint (see makeWalker)
const UP = new THREE.Vector3(0, 1, 0);

const status = document.getElementById('status');
const button = document.getElementById('enter-vr');

const scene = makeScene();
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
document.body.appendChild(renderer.domElement);

const camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.05, 100);
const dolly = new THREE.Group();     // places the real (tracked) space in the sim world
dolly.add(camera);
scene.add(dolly);
const grips = [renderer.xr.getControllerGrip(0), renderer.xr.getControllerGrip(1)];
grips.forEach((g) => dolly.add(g));

const walker = makeWalker();
scene.add(walker);
const tags = new HazardTags(scene);
const trajectory = new Trajectory(scene);
const patch = new TestPatch(walker);
patch.group.position.copy(rosToThree(1.0, 0, 1.5));
const hint = new TextPanel(scene, { icon: true });
const notice = new TextPanel(scene, { icon: true, colour: '#ffcf70' });
const safetyText = new TextPanel(scene, { icon: false, colour: '#ff8080' });

// comfort vignette - darkens the edge of view while the walker moves (stationary mode)
const vignette = new THREE.Mesh(new THREE.RingGeometry(0.22, 1.5, 48),
  new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0, depthTest: false }));
vignette.position.z = -0.3;
vignette.renderOrder = 10;
camera.add(vignette);

let config = { vr_mode: 'room_scale', comfort_vignette: true, user_to_handle_offset: 0.4,
               boundary_stop_margin: 0.5, max_user_handle_distance: 0.8 };
let pose = { x: 0, y: 0, yaw: 0 };
let steering = {};
let inVR = false, connected = false, flickerOn = true, worldCount = 0;
let bounds = null;              // Guardian polygon [[x, z], ...] in the tracking space, if known
let anchor = null;              // set by recentre (see below)
let safety = { stop: false, reason: '' };

const link = connect({
  config: (m) => { config = { ...config, ...m }; showStatus(); },
  walker_pose: (m) => { pose = m; },
  hazards: (m) => tags.update(m.hazards),
  trajectory: (m) => trajectory.update(m),
  steering: (m) => { steering = m; },
  test_stimulus: (m) => patch.set(m),
  flicker_enabled: (m) => { tags.flickerEnabled = m.value; flickerOn = m.value; },
  command: (m) => { if (m.name === 'recentre') recentre(); },
}, (ok) => { connected = ok; showStatus(); });

addWorld(scene).then((n) => { worldCount = n; showStatus(); });

function showStatus(extra = '') {
  status.textContent = `${connected ? 'Connected' : 'Connecting…'} · ${worldCount} objects · ` +
    `${inVR ? `VR (${config.vr_mode})` : 'desktop (drag to look, hold W = hands on, + Space = squeeze)'}` +
    `${flickerOn ? '' : ' · flicker OFF'} ${extra}`;
}

// ---------- where things are ----------

function headLocal() { return camera.position.clone(); }
function headYawLocal() { return new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ').y; }

// the handles' midpoint in the sim (three.js world coordinates)
function simHandles() {
  return rosToThree(pose.x + HANDLE_X * Math.cos(pose.yaw), pose.y + HANDLE_X * Math.sin(pose.yaw), 0);
}

// the hands' midpoint in the tracking space: the controllers if both are tracked, else in front of the head
function handsLocal() {
  const tracked = grips.filter((g) => g.visible !== false && g.position.lengthSq() > 0);
  if (tracked.length === 2) return tracked[0].position.clone().add(tracked[1].position).multiplyScalar(0.5);
  const yaw = headYawLocal();
  return headLocal().add(new THREE.Vector3(-Math.sin(yaw), 0, -Math.cos(yaw)).multiplyScalar(config.user_to_handle_offset));
}

// ---------- recentre / reset ----------
// Aligns the sim walker with the user: walker forward = the user's current head direction,
// and the walker's handles at the user's hands. (In room_scale this also re-anchors the sim world
// to the room, so the user can turn round and carry on in the space they have.)

function recentre() {
  if (!inVR) return;
  const theta = pose.yaw - headYawLocal();          // ROS yaw = rotation about three's +y
  const hands = handsLocal();
  const head = headLocal();
  const reach = Math.hypot(hands.x - head.x, hands.z - head.z);
  anchor = { theta, hands, head, reach: Math.max(0.25, Math.min(reach, 0.7)) };
  if (config.vr_mode === 'room_scale') {
    dolly.rotation.set(0, theta, 0);
    const h = simHandles();
    const local = new THREE.Vector3(hands.x, 0, hands.z).applyAxisAngle(UP, theta);
    dolly.position.set(h.x - local.x, 0, h.z - local.z);
  }
}

function updateDolly() {
  if (!inVR) {                                      // desktop: stand behind the handles
    const s = rosToThree(pose.x - 0.5 * Math.cos(pose.yaw), pose.y - 0.5 * Math.sin(pose.yaw), 0);
    dolly.position.copy(s);
    dolly.rotation.set(0, pose.yaw, 0);
    return;
  }
  if (!anchor || config.vr_mode === 'room_scale') return;   // room scale: fixed after recentre
  // stationary: the view follows the walker's position, never its rotation
  dolly.rotation.set(0, anchor.theta, 0);
  const h = simHandles();
  const fwd = new THREE.Vector3(-Math.sin(pose.yaw), 0, -Math.cos(pose.yaw));
  const standAt = h.clone().sub(fwd.multiplyScalar(anchor.reach));
  const head0 = new THREE.Vector3(anchor.head.x, 0, anchor.head.z).applyAxisAngle(UP, anchor.theta);
  dolly.position.set(standAt.x - head0.x, 0, standAt.z - head0.z);
}

// ---------- safety (room scale) ----------

function distanceToBounds(x, z) {
  if (!bounds || bounds.length < 3) return Infinity;
  let inside = false, best = Infinity;
  for (let i = 0, j = bounds.length - 1; i < bounds.length; j = i++) {
    const [xi, zi] = bounds[i], [xj, zj] = bounds[j];
    if ((zi > z) !== (zj > z) && x < ((xj - xi) * (z - zi)) / (zj - zi) + xi) inside = !inside;
    const dx = xj - xi, dz = zj - zi;
    const t = Math.max(0, Math.min(1, ((x - xi) * dx + (z - zi) * dz) / (dx * dx + dz * dz || 1)));
    best = Math.min(best, Math.hypot(x - (xi + t * dx), z - (zi + t * dz)));
  }
  return inside ? best : -best;
}

function checkSafety() {
  let reason = '';
  if (inVR && anchor && config.vr_mode === 'room_scale') {
    const head = headLocal();
    if (distanceToBounds(head.x, head.z) < config.boundary_stop_margin) reason = 'Room edge reached';
    const world = camera.getWorldPosition(new THREE.Vector3());
    const h = simHandles();
    if (Math.hypot(world.x - h.x, world.z - h.z) > config.max_user_handle_distance) {
      reason = reason || 'Too far from the handles';
    }
  }
  const stop = reason !== '';
  if (stop !== safety.stop || reason !== safety.reason) {
    safety = { stop, reason };
    link.send({ type: 'safety', stop, reason });
  }
}

// ---------- grip ----------
// Quest: the controllers' squeeze (grip) buttons, analog 0..1 (xr-standard button 1).
// Desktop: hold W = both hands on (0.4), W + Space = squeeze firmly (0.9).

const keys = new Set();
let desktopGripUsed = false;
window.addEventListener('keydown', (e) => { keys.add(e.code); if (e.code === 'KeyW') desktopGripUsed = true; });
window.addEventListener('keyup', (e) => keys.delete(e.code));

let resetHeldSince = null;
function readGrip(t) {
  if (inVR) {
    const session = renderer.xr.getSession();
    const g = { left: 0, right: 0 };
    let resetPressed = false;
    for (const source of session ? session.inputSources : []) {
      const pad = source.gamepad;
      if (!pad || !(source.handedness in g)) continue;
      g[source.handedness] = pad.buttons[1] ? pad.buttons[1].value : 0;
      if (pad.buttons[4] && pad.buttons[4].pressed) resetPressed = true;   // A / X
    }
    if (resetPressed) {
      if (resetHeldSince === null) resetHeldSince = t;
      else if (t - resetHeldSince > 1.0) { recentre(); resetHeldSince = Infinity; }
    } else resetHeldSince = null;
    return g;
  }
  if (!desktopGripUsed) return null;
  const on = keys.has('KeyW') ? (keys.has('Space') ? 0.9 : 0.4) : 0;
  return { left: on, right: on };
}

// ---------- desktop mode: drag to look ----------

let lookYaw = 0, lookPitch = 0, dragging = false;
camera.position.set(0, DESKTOP_EYE_HEIGHT, 0);
renderer.domElement.addEventListener('pointerdown', () => { dragging = true; });
window.addEventListener('pointerup', () => { dragging = false; });
window.addEventListener('pointermove', (e) => {
  if (!dragging || inVR) return;
  lookYaw -= e.movementX * 0.004;
  lookPitch = Math.max(-1.2, Math.min(1.2, lookPitch - e.movementY * 0.004));
  camera.rotation.set(lookPitch, lookYaw, 0, 'YXZ');
});
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ---------- Enter VR ----------

async function setupVR() {
  if (!navigator.xr || !(await navigator.xr.isSessionSupported('immersive-vr'))) {
    button.textContent = location.protocol === 'https:' ? 'VR not available on this device' : 'VR needs https://';
    button.disabled = true;
    return;
  }
  button.onclick = async () => {
    const session = await navigator.xr.requestSession('immersive-vr',
      { optionalFeatures: ['local-floor', 'bounded-floor'] });
    // room scale: use the Guardian's floor space if the headset offers it, so its edge is known
    bounds = null;
    renderer.xr.setReferenceSpaceType('local-floor');
    if (config.vr_mode === 'room_scale') {
      try {
        const space = await session.requestReferenceSpace('bounded-floor');
        if (space.boundsGeometry && space.boundsGeometry.length >= 3) {
          bounds = Array.from(space.boundsGeometry).map((p) => [p.x, p.z]);
          renderer.xr.setReferenceSpaceType('bounded-floor');
        }
      } catch (e) { /* no Guardian bounds: the handle-distance check still applies */ }
    }
    await renderer.xr.setSession(session);
    if (session.supportedFrameRates && session.updateTargetFrameRate) {   // flicker is tuned for 90 Hz
      const rates = Array.from(session.supportedFrameRates);
      if (rates.includes(90)) session.updateTargetFrameRate(90).catch(() => {});
    }
    inVR = true;
    anchor = null;
    camera.position.set(0, 0, 0);
    button.style.display = 'none';
    setTimeout(recentre, 1000);     // once the headset pose has settled
    session.addEventListener('end', () => {
      inVR = false;
      anchor = null;
      button.style.display = '';
      camera.position.set(0, DESKTOP_EYE_HEIGHT, 0);
      camera.rotation.set(lookPitch, lookYaw, 0, 'YXZ');
      checkSafety();
      showStatus();
    });
    showStatus(bounds ? '· Guardian bounds known' : '');
  };
}
setupVR();

// ---------- the frame loop ----------

let lastTime = null, frames = 0, fpsTime = 0, fps = 0, lastHeadSend = 0, lastGripSend = 0, lastSafety = 0;
let lastPose = null, moveRate = 0;
renderer.setAnimationLoop((time) => {
  const t = time / 1000;
  const dt = lastTime === null ? 0 : Math.min(t - lastTime, 0.1);
  lastTime = t;

  placeWalker(walker, pose);
  updateDolly();
  tags.animate(t, dt);
  patch.animate(t);

  // text: hint low in view, the blocked notice a little higher, both kept off hazard tags
  const tagDirs = tags.visibleDirections(camera);
  hint.show(steering.notice ? '' : (steering.hint || ''), camera, -0.32, tagDirs, dt);
  notice.show(steering.notice || '', camera, -0.14, tagDirs, dt);
  safetyText.show(safety.reason, camera, 0.05, tagDirs, dt);

  // vignette while the walker moves (stationary mode only; in room scale the user moves for real)
  if (dt > 0 && lastPose) {
    const rate = Math.hypot(pose.x - lastPose.x, pose.y - lastPose.y) / dt;
    moveRate += (rate - moveRate) * Math.min(dt * 4, 1);
  }
  lastPose = { ...pose };
  const vignetteOn = inVR && config.vr_mode === 'stationary' && config.comfort_vignette;
  vignette.material.opacity = vignetteOn ? Math.min(0.55, moveRate * 1.4) : 0;
  vignette.visible = vignette.material.opacity > 0.02;

  renderer.render(scene, camera);

  if (t - lastGripSend > 0.02) {
    lastGripSend = t;
    const g = readGrip(t);
    if (g) link.send({ type: 'grip', left: g.left, right: g.right });
  }
  if (t - lastSafety > 0.1) { lastSafety = t; checkSafety(); }
  if (t - lastHeadSend > 0.05) {
    lastHeadSend = t;
    const p = threeToRos(camera.getWorldPosition(new THREE.Vector3()));
    const h = rosHeading(camera);
    link.send({ type: 'head_pose', x: p.x, y: p.y, z: p.z, yaw: h.yaw, pitch: h.pitch });
  }
  frames++;
  if (t - fpsTime > 2) {
    fps = frames / (t - fpsTime);
    frames = 0;
    fpsTime = t;
    link.send({ type: 'headset_status', mode: inVR ? `vr_${config.vr_mode}` : 'desktop', fps: Math.round(fps),
                guardian: !!bounds });
    showStatus(`· ${Math.round(fps)} fps${steering.grip ? ` · grip ${steering.grip}` : ''}`);
  }
});
