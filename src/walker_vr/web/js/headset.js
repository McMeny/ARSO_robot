// The participant's view. In the Quest: tap Enter VR (WebXR, full VR). On a PC with no
// headset: drag the mouse to look around (desktop mode).
//
// The user stands (virtually) behind the walker's handles, at a standing eye height of 1.6 m,
// and moves with the simulated walker. Their real head movement is added on top, and the
// head pose is sent to ROS 20 times a second for head-guided steering and the emulated EEG.
import * as THREE from '../vendor/three.module.js';
import { connect, rosToThree, threeToRos, rosHeading, makeScene, addWorld, makeWalker, placeWalker,
         HazardTags, Trajectory, TestPatch } from './common.js';

const EYE_HEIGHT = 1.6;      // m, a standing walker user
const USER_BEHIND = 0.5;     // m behind base_footprint (the walker's axle), i.e. just behind the handles

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
const dolly = new THREE.Group();     // carries the user along with the walker
dolly.add(camera);
scene.add(dolly);

const walker = makeWalker();
scene.add(walker);
const tags = new HazardTags(scene);
const trajectory = new Trajectory(scene);
const patch = new TestPatch(walker);
patch.group.position.copy(rosToThree(1.0, 0, 1.5));   // 1 m in front of the user, near eye height

// comfort vignette - darkens the edge of view while the walker turns
const vignette = new THREE.Mesh(new THREE.RingGeometry(0.22, 1.5, 48),
  new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0, depthTest: false }));
vignette.position.z = -0.3;
vignette.renderOrder = 10;
camera.add(vignette);

let pose = { x: 0, y: 0, yaw: 0 };
let lastYaw = 0, turnRate = 0;
let recentred = { yaw: 0, local: new THREE.Vector3(0, EYE_HEIGHT, 0) };   // head pose at the last recentre
let inVR = false;
let connected = false;
let flickerOn = true;
let worldCount = 0;

const link = connect({
  walker_pose: (m) => { pose = m; },
  hazards: (m) => tags.update(m.hazards),
  trajectory: (m) => trajectory.update(m),
  test_stimulus: (m) => patch.set(m),
  flicker_enabled: (m) => { tags.flickerEnabled = m.value; flickerOn = m.value; },
  command: (m) => { if (m.name === 'recentre') recentre(); },
}, (ok) => { connected = ok; showStatus(); });

addWorld(scene).then((n) => { worldCount = n; showStatus(); });

function showStatus(extra = '') {
  status.textContent = `${connected ? 'Connected' : 'Connecting…'} · ${worldCount} objects · ` +
    `${inVR ? 'VR' : 'desktop (drag to look)'}${flickerOn ? '' : ' · flicker OFF'} ${extra}`;
}

// ---------- recentre ----------
// Puts the handles in front of the participant, whatever way they're sitting: their current
// head direction becomes "walker forward", and their head is placed at standing eye height.

function recentre() {
  if (!inVR) return;
  const e = new THREE.Euler().setFromQuaternion(camera.quaternion, 'YXZ');
  recentred = { yaw: e.y, local: camera.position.clone() };
}

function updateDolly() {
  const s = rosToThree(pose.x - USER_BEHIND * Math.cos(pose.yaw), pose.y - USER_BEHIND * Math.sin(pose.yaw), EYE_HEIGHT);
  const yaw = pose.yaw - (inVR ? recentred.yaw : 0);
  dolly.rotation.y = yaw;
  const head = inVR ? recentred.local.clone().applyAxisAngle(new THREE.Vector3(0, 1, 0), yaw)
                    : new THREE.Vector3(0, EYE_HEIGHT, 0);
  dolly.position.copy(s.sub(head));
}

// ---------- desktop mode: drag to look ----------

let lookYaw = 0, lookPitch = 0, dragging = false;
camera.position.set(0, EYE_HEIGHT, 0);
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
    const session = await navigator.xr.requestSession('immersive-vr', { optionalFeatures: ['local-floor'] });
    await renderer.xr.setSession(session);
    // the flicker frequencies are chosen for 90 Hz
    if (session.supportedFrameRates && session.updateTargetFrameRate) {
      const rates = Array.from(session.supportedFrameRates);
      if (rates.includes(90)) session.updateTargetFrameRate(90).catch(() => {});
    }
    inVR = true;
    button.style.display = 'none';
    setTimeout(recentre, 1000);     // once the headset pose has settled
    session.addEventListener('end', () => {
      inVR = false;
      button.style.display = '';
      camera.position.set(0, EYE_HEIGHT, 0);
      camera.rotation.set(lookPitch, lookYaw, 0, 'YXZ');
      showStatus();
    });
    showStatus();
  };
}
setupVR();

// ---------- the frame loop ----------

let lastTime = null, frames = 0, fpsTime = 0, fps = 0, lastHeadSend = 0;
renderer.setAnimationLoop((time) => {
  const t = time / 1000;
  const dt = lastTime === null ? 0 : Math.min(t - lastTime, 0.1);
  lastTime = t;

  placeWalker(walker, pose);
  updateDolly();
  tags.animate(t, dt);
  patch.animate(t);

  // vignette while turning (rad/s), smoothed
  if (dt > 0) {
    const rate = Math.abs(Math.atan2(Math.sin(pose.yaw - lastYaw), Math.cos(pose.yaw - lastYaw))) / dt;
    turnRate += (rate - turnRate) * Math.min(dt * 4, 1);
  }
  lastYaw = pose.yaw;
  vignette.material.opacity = Math.min(0.65, turnRate * 1.2);
  vignette.visible = inVR && vignette.material.opacity > 0.02;

  renderer.render(scene, camera);

  // head pose to ROS, 20 Hz
  if (t - lastHeadSend > 0.05) {
    lastHeadSend = t;
    const p = threeToRos(camera.getWorldPosition(new THREE.Vector3()));
    const h = rosHeading(camera);
    link.send({ type: 'head_pose', x: p.x, y: p.y, z: p.z, yaw: h.yaw, pitch: h.pitch });
  }
  // frame rate, every 2 s - the flicker depends on it
  frames++;
  if (t - fpsTime > 2) {
    fps = frames / (t - fpsTime);
    frames = 0;
    fpsTime = t;
    link.send({ type: 'headset_status', mode: inVR ? 'vr' : 'desktop', fps: Math.round(fps) });
    showStatus(`· ${Math.round(fps)} fps`);
  }
});
