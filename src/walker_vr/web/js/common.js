// Shared by the headset and spectator pages: the connection to ROS, coordinate
// conversion, and the scene - the store, the walker, the hazard tags and the floor trajectory.
import * as THREE from '../vendor/three.module.js';

// ---------- connection ----------

export function connect(handlers, onState) {
  let ws = null;
  const open = () => {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onopen = () => onState && onState(true);
    ws.onclose = () => { onState && onState(false); setTimeout(open, 1500); };
    ws.onmessage = (e) => {
      const msg = JSON.parse(e.data);
      const h = handlers[msg.type];
      if (h) h(msg);
    };
  };
  open();
  return {
    send(obj) { if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj)); },
  };
}

// ---------- coordinates ----------
// ROS map frame: x forward, y left, z up.  three.js: x right, y up, z towards the viewer.
// A ROS yaw is the same angle as a rotation about three's +y.

export const rosToThree = (x, y, z = 0) => new THREE.Vector3(-y, z, -x);
export const threeToRos = (v) => ({ x: -v.z, y: -v.x, z: v.y });

// yaw (about ROS z) and pitch (nose down = +) of a three.js camera, in the ROS frame
export function rosHeading(object) {
  const f = new THREE.Vector3(0, 0, -1).applyQuaternion(object.getWorldQuaternion(new THREE.Quaternion()));
  const r = threeToRos(f);
  return { yaw: Math.atan2(r.y, r.x), pitch: -Math.atan2(r.z, Math.hypot(r.x, r.y)) };
}

// ---------- scene ----------

export function makeScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xbfd4e6);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x777777, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.2);
  sun.position.set(-3, 10, 4);
  scene.add(sun);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshLambertMaterial({ color: 0xd8d8d0 }));
  floor.rotation.x = -Math.PI / 2;
  scene.add(floor);
  const grid = new THREE.GridHelper(40, 40, 0xb0b0a8, 0xc4c4bc);
  grid.position.y = 0.002;
  scene.add(grid);
  return scene;
}

export async function addWorld(scene) {
  const res = await fetch('/world.json');
  const { objects } = await res.json();
  for (const o of objects) {
    const colour = new THREE.Color(...o.colour);
    const material = new THREE.MeshLambertMaterial({ color: colour });
    let geometry;
    if (o.kind === 'box') geometry = new THREE.BoxGeometry(o.size_y, o.height, o.size_x);
    else geometry = new THREE.CylinderGeometry(o.size_x / 2, o.size_x / 2, o.height, 24);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.copy(rosToThree(o.x, o.y, o.z));
    mesh.rotation.y = o.yaw;
    scene.add(mesh);
  }
  return objects.length;
}

// The walker, drawn in base_footprint: body from 0.04 m behind the axle to 0.46 m in front,
// 0.5 m wide, with two handles at the back for the user.
export function makeWalker() {
  const walker = new THREE.Group();
  const frame = new THREE.MeshLambertMaterial({ color: 0x3a4250 });
  const grip = new THREE.MeshLambertMaterial({ color: 0x1a1a1a });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.12, 0.5), frame);
  body.position.copy(rosToThree(0.21, 0, 0.18));
  walker.add(body);
  for (const side of [-1, 1]) {
    const post = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.75, 0.04), frame);
    post.position.copy(rosToThree(-0.02, side * 0.23, 0.6));
    walker.add(post);
    const handle = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.16, 12), grip);
    handle.rotation.x = Math.PI / 2;
    handle.position.copy(rosToThree(-0.1, side * 0.23, 0.95));
    walker.add(handle);
    const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.04, 16), grip);
    wheel.rotation.z = Math.PI / 2;
    wheel.position.copy(rosToThree(0, side * 0.22, 0.05));
    walker.add(wheel);
  }
  const bar = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.04, 0.04), frame);
  bar.position.copy(rosToThree(-0.02, 0, 0.95));
  walker.add(bar);
  return walker;
}

export function placeWalker(walker, pose) {
  walker.position.copy(rosToThree(pose.x, pose.y, 0));
  walker.rotation.y = pose.yaw;
}

// ---------- hazard tags ----------
// A see-through shell around each tagged object whose brightness flickers as a sine at the
// hazard's frequency: opacity = contrast * (0.5 + 0.5 sin(2 pi f t)). A missed hazard gets
// the stronger, orange cue. When a hazard is registered its tag fades out over a second.

export class HazardTags {
  constructor(scene) {
    this.scene = scene;
    this.tags = new Map();
    this.flickerEnabled = true;
  }

  update(hazards) {
    const seen = new Set();
    for (const h of hazards) {
      seen.add(h.id);
      let tag = this.tags.get(h.id);
      if (!tag) {
        const pad = 0.06;
        const geometry = h.kind === 'cylinder'
          ? new THREE.CylinderGeometry(h.size_x / 2 + pad, h.size_x / 2 + pad, h.height + pad, 24)
          : new THREE.BoxGeometry(h.size_y + 2 * pad, h.height + pad, h.size_x + 2 * pad);
        const material = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0, depthWrite: false });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.copy(rosToThree(h.x, h.y, h.z));
        mesh.rotation.y = h.yaw;
        mesh.renderOrder = 2;
        this.scene.add(mesh);
        tag = { mesh, fade: 0 };
        this.tags.set(h.id, tag);
      }
      if (tag.hazard && tag.hazard.frequency > 0 && h.state === 'registered') tag.fade = 1.0;  // just registered
      tag.hazard = h;
    }
    for (const [id, tag] of this.tags) {
      if (!seen.has(id)) { this.scene.remove(tag.mesh); this.tags.delete(id); }
    }
  }

  // t in seconds, from the display's frame clock
  animate(t, dt) {
    for (const tag of this.tags.values()) {
      const h = tag.hazard;
      const m = tag.mesh.material;
      if (this.flickerEnabled && h.frequency > 0) {
        m.color.set(h.state === 'missed' ? 0xffa040 : 0xffffff);
        m.opacity = h.contrast * (0.5 + 0.5 * Math.sin(2 * Math.PI * h.frequency * t));
      } else if (h.state === 'missed') {
        m.color.set(0xffa040);           // missed but not (or no longer) flickering: a steady orange cue
        m.opacity = 0.3;
      } else if (tag.fade > 0) {
        tag.fade = Math.max(0, tag.fade - dt);
        m.color.set(0xffffff);
        m.opacity = 0.25 * tag.fade;     // registered: fade out
      } else {
        m.opacity = 0;
      }
      tag.mesh.visible = m.opacity > 0.001;
    }
  }
}

// ---------- the floor trajectory ----------
// Flat ribbons on the floor (WebXR ignores line widths): a centre line and two edges marking
// the walker's footprint over the next couple of seconds. More solid as the machine takes over.

function ribbon(points, width) {
  const positions = [];
  for (let i = 0; i < points.length - 1; i++) {
    const a = rosToThree(points[i][0], points[i][1], 0.012);
    const b = rosToThree(points[i + 1][0], points[i + 1][1], 0.012);
    const d = new THREE.Vector3().subVectors(b, a);
    if (d.lengthSq() < 1e-8) continue;
    const n = new THREE.Vector3(-d.z, 0, d.x).normalize().multiplyScalar(width / 2);
    const p = [a.clone().add(n), a.clone().sub(n), b.clone().add(n), b.clone().sub(n)];
    positions.push(...p[0].toArray(), ...p[1].toArray(), ...p[2].toArray(),
                   ...p[2].toArray(), ...p[1].toArray(), ...p[3].toArray());
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  return g;
}

export class Trajectory {
  constructor(scene) {
    this.centre = new THREE.Mesh(new THREE.BufferGeometry(),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, side: THREE.DoubleSide, depthWrite: false }));
    this.edges = new THREE.Mesh(new THREE.BufferGeometry(),
      new THREE.MeshBasicMaterial({ color: 0x40e0ff, transparent: true, side: THREE.DoubleSide, depthWrite: false }));
    scene.add(this.centre, this.edges);
  }

  update(t) {
    if (!t.centre || t.centre.length < 2) return;
    this.centre.geometry.dispose();
    this.centre.geometry = ribbon(t.centre, 0.025);
    const left = ribbon(t.left, 0.04), right = ribbon(t.right, 0.04);
    const merged = new THREE.BufferGeometry();
    const a = left.getAttribute('position').array, b = right.getAttribute('position').array;
    const all = new Float32Array(a.length + b.length);
    all.set(a); all.set(b, a.length);
    merged.setAttribute('position', new THREE.BufferAttribute(all, 3));
    this.edges.geometry.dispose();
    this.edges.geometry = merged;
    const solid = 0.35 + 0.6 * (t.machine_authority ?? 1);   // more solid as the machine takes over
    this.edges.material.opacity = solid;
    this.centre.material.opacity = solid * 0.7;
    this.edges.material.color.set(t.extra_clearance > 0 ? 0xffa040 : 0x40e0ff);  // widened berth shows orange
  }
}

// ---------- the SSVEP-check test patch ----------

export class TestPatch {
  constructor(parent) {
    this.mesh = new THREE.Mesh(new THREE.PlaneGeometry(0.4, 0.4),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0 }));
    this.backing = new THREE.Mesh(new THREE.PlaneGeometry(0.5, 0.5), new THREE.MeshBasicMaterial({ color: 0x202020 }));
    this.group = new THREE.Group();
    this.group.add(this.backing, this.mesh);
    this.mesh.position.z = 0.001;
    this.group.visible = false;
    parent.add(this.group);
    this.freq = 0;
  }

  set(msg) {
    this.freq = msg.on ? msg.freq : 0;
    this.group.visible = msg.on;
  }

  animate(t) {
    if (this.freq > 0) this.mesh.material.opacity = 0.5 * (0.5 + 0.5 * Math.sin(2 * Math.PI * this.freq * t));
  }
}
