// The headset mirror: draws the same scene as the participant's headset - tags, flicker and
// floor trajectory included - from their head pose, so other people can watch on a PC.
// Before the headset connects it shows the view from behind the walker's handles.
import * as THREE from '../vendor/three.module.js';
import { connect, rosToThree, makeScene, addWorld, makeWalker, placeWalker,
         HazardTags, Trajectory, TestPatch } from './common.js';

const status = document.getElementById('status');
const scene = makeScene();
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const camera = new THREE.PerspectiveCamera(80, window.innerWidth / window.innerHeight, 0.05, 100);

const walker = makeWalker();
scene.add(walker);
const tags = new HazardTags(scene);
const trajectory = new Trajectory(scene);
const patch = new TestPatch(walker);
patch.group.position.copy(rosToThree(1.0, 0, 1.5));

let pose = { x: 0, y: 0, yaw: 0 };
let head = null, headTime = 0, connected = false, worldCount = 0, mode = '';

connect({
  walker_pose: (m) => { pose = m; },
  head_pose: (m) => { head = m; headTime = performance.now(); },
  headset_status: (m) => { mode = `headset ${m.mode}, ${m.fps} fps`; },
  hazards: (m) => tags.update(m.hazards),
  trajectory: (m) => trajectory.update(m),
  test_stimulus: (m) => patch.set(m),
  flicker_enabled: (m) => { tags.flickerEnabled = m.value; },
}, (ok) => { connected = ok; });
addWorld(scene).then((n) => { worldCount = n; });

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

let lastTime = null, lastStatus = 0;
renderer.setAnimationLoop((time) => {
  const t = time / 1000;
  const dt = lastTime === null ? 0 : Math.min(t - lastTime, 0.1);
  lastTime = t;
  placeWalker(walker, pose);
  tags.animate(t, dt);
  patch.animate(t);

  const live = head && performance.now() - headTime < 1000;
  if (live) {
    camera.position.copy(rosToThree(head.x, head.y, head.z));
    camera.rotation.set(-head.pitch, head.yaw, 0, 'YXZ');   // ROS yaw = three yaw; nose-down pitch = -x rotation
  } else {
    camera.position.copy(rosToThree(pose.x - 0.5 * Math.cos(pose.yaw), pose.y - 0.5 * Math.sin(pose.yaw), 1.6));
    camera.rotation.set(-0.25, pose.yaw, 0, 'YXZ');
  }
  renderer.render(scene, camera);

  if (t - lastStatus > 0.5) {
    lastStatus = t;
    status.textContent = `${connected ? 'Connected' : 'Connecting…'} · ${worldCount} objects · ` +
      (live ? `participant's view (${mode})` : 'no headset yet - view from behind the walker');
  }
});
