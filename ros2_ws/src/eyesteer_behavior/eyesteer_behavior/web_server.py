"""HTTPS static server for the WebXR app + the health dashboard, served from the Jetson (D16).

WebXR needs a secure context, so this serves over TLS with the cert from deploy/certs/make_cert.sh.
Endpoints besides the static files in webxr/:
  /config/frequencies.json   the single frequency table (same as the decoder; versioned by hash)
  /config/system.json        robot, refresh rate, rosbridge port, decal positions, marker presets
Usage: ros2 run eyesteer_behavior web_server --ros-args -p port:=8443   (or python3 -m ... without ROS)
"""
from __future__ import annotations

import argparse
import functools
import http.server
import json
import ssl
import sys
import threading
from pathlib import Path

from eyesteer_behavior import _root

from eyesteer_core import config

ROOT = _root.ROOT


def system_json(robot: str, eeg: str, mode: str, demo: bool = False) -> dict:
    hw = config.hardware()
    r = config.robot(robot)
    return {"robot": robot, "idle_mode": r["idle_mode"], "refresh_hz": hw["headset"]["refresh_hz"],
            "rosbridge_port": hw["headset"]["rosbridge_port"], "eeg": eeg, "mode": mode, "demo": demo,
            "eeg_label": {"synthetic": "SIMULATED EEG", "replay": "REPLAYED EEG"}.get(eeg, "REAL EEG"),
            "decals": r["calibration_decals"], "marker_size": 0.3,
            "presets": ["Beans", "Bread", "Milk", "Produce", "Checkout", "Door", "Window", "Table", "A", "B", "C", "D", "E"]}


class Handler(http.server.SimpleHTTPRequestHandler):
    extra: dict = {}

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def do_GET(self):
        if self.path.startswith("/config/"):
            name = self.path.split("?")[0][len("/config/"):]
            body = None
            if name == "frequencies.json":
                body = config.frequency_table()
            elif name == "system.json":
                body = self.extra
            if body is not None:
                data = json.dumps(body).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
        super().do_GET()

    def log_message(self, fmt, *args):
        pass


def serve(port: int, cert: str | None, key: str | None, robot: str, eeg: str, mode: str, block: bool = True,
          demo: bool = False):
    Handler.extra = system_json(robot, eeg, mode, demo)
    handler = functools.partial(Handler, directory=str(ROOT / "webxr"))
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
    scheme = "http"
    if cert and Path(cert).exists():
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
        scheme = "https"
    print(f"[web_server] {scheme}://0.0.0.0:{port}/  (webxr app, /dashboard.html, /config/*)", flush=True)
    if block:
        httpd.serve_forever()
    else:
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main(argv=None):
    certdir = ROOT / "deploy/certs"
    try:
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=argv)
        n = Node("web_server")
        port = n.declare_parameter("port", config.hardware()["headset"]["web_port"]).value
        cert = n.declare_parameter("cert", str(certdir / "eyesteer.pem")).value
        key = n.declare_parameter("key", str(certdir / "eyesteer.key")).value
        robot = n.declare_parameter("robot", "field_robot").value
        eeg = n.declare_parameter("eeg", "synthetic").value
        mode = n.declare_parameter("mode", "sim").value
        demo = n.declare_parameter("demo", False).value      # desktop demo: no headset, no on-screen flicker
        serve(port, cert, key, robot, eeg, mode, block=False, demo=demo)
        rclpy.spin(n)
    except ImportError:
        ap = argparse.ArgumentParser()
        ap.add_argument("--port", type=int, default=8443)
        ap.add_argument("--robot", default="field_robot")
        a = ap.parse_args(argv)
        serve(a.port, str(certdir / "eyesteer.pem"), str(certdir / "eyesteer.key"), a.robot, "synthetic", "sim")


if __name__ == "__main__":
    sys.exit(main())
