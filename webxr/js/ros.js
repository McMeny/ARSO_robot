// rosbridge connection (roslibjs) with auto-reconnect. Every topic/service the headset uses is listed
// here and in INTERFACES.md.
/* global ROSLIB */

export function defaultUrl(loc = window.location, port = 9090) {
  const q = new URLSearchParams(loc.search);
  if (q.get("rosbridge")) return q.get("rosbridge");
  const proto = loc.protocol === "https:" ? "wss" : "ws";
  return `${proto}://${loc.hostname || "localhost"}:${port}`;
}

export class RosLink {
  constructor(url, handlers = {}) {
    this.url = url;
    this.h = handlers;
    this.connected = false;
    this.ros = null;
    this.pubs = {};
    this.reconnects = 0;
    this.connect();
  }

  connect() {
    this.ros = new ROSLIB.Ros({ url: this.url });
    this.ros.on("connection", () => {
      this.connected = true;
      this.setup();
      this.h.onConnection && this.h.onConnection(true);
    });
    const down = () => {
      if (this.connected) this.h.onConnection && this.h.onConnection(false);
      this.connected = false;
      clearTimeout(this.timer);
      this.timer = setTimeout(() => {
        this.reconnects += 1;
        this.connect();
      }, 2000);
    };
    this.ros.on("close", down);
    this.ros.on("error", () => {});
  }

  sub(name, type, cb, opts = {}) {
    const t = new ROSLIB.Topic({ ros: this.ros, name, messageType: type, ...opts });
    t.subscribe(cb);
    return t;
  }

  setup() {
    const h = this.h;
    this.pubs = {};
    // state the headset shows (HI observability)
    this.sub("/robot/status", "eyesteer_interfaces/RobotStatus", (m) => h.onStatus && h.onStatus(m), { throttle_rate: 50 });
    this.sub("/markers", "eyesteer_interfaces/MarkerArray", (m) => h.onMarkers && h.onMarkers(m));
    this.sub("/bci/status", "eyesteer_interfaces/BciStatus", (m) => h.onBci && h.onBci(m), { throttle_rate: 100 });
    this.sub("/system/health", "eyesteer_interfaces/SystemHealth", (m) => h.onHealth && h.onHealth(m), { throttle_rate: 250 });
    this.sub("/xr/calibration", "eyesteer_interfaces/HeadsetCalibration", (m) => h.onCalibration && h.onCalibration(m));
    this.sub("/bci/calibration_cue", "std_msgs/Int32", (m) => h.onCue && h.onCue(m.data));
    this.sub("/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped", (m) => h.onPose && h.onPose(m.pose.pose), { throttle_rate: 100 });
    this.sub("/sim/ground_truth", "nav_msgs/Odometry", (m) => h.onPose && h.onPose(m.pose.pose, true), { throttle_rate: 100 });
    this.sub("/bci/selection", "eyesteer_interfaces/Selection", (m) => h.onSelection && h.onSelection(m));
    this.sub("/sim/people", "geometry_msgs/PoseArray", (m) => h.onPeople && h.onPeople(m.poses), { throttle_rate: 200 });
    this.sub("/eyesteer/latency", "std_msgs/String", (m) => h.onLatency && h.onLatency(JSON.parse(m.data)));
    // the occupancy map, only where it is drawn (desktop demo): not worth the bandwidth on the headset
    if (h.onMap) this.sub("/map", "nav_msgs/OccupancyGrid", (m) => h.onMap(m));
  }

  pub(name, type, msg) {
    if (!this.connected) return false;
    if (!this.pubs[name]) this.pubs[name] = new ROSLIB.Topic({ ros: this.ros, name, messageType: type });
    this.pubs[name].publish(msg);
    return true;
  }

  call(name, type, req) {
    return new Promise((resolve, reject) => {
      if (!this.connected) return reject(new Error("rosbridge not connected"));
      const s = new ROSLIB.Service({ ros: this.ros, name, serviceType: type });
      s.callService(req, resolve, reject);
    });
  }
}
