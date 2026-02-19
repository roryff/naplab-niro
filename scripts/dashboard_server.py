#!/usr/bin/env python3
"""
Car System Web Dashboard
Serves a live status dashboard at http://localhost:8080

Run:
    source install/setup.bash
    python3 src/car_control/scripts/dashboard_server.py

No extra dependencies - uses Python stdlib http.server + rclpy.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from car_control.msg import VehicleState, EsfStatus

import threading
import json
import time
import math
import pathlib
from http.server import HTTPServer, BaseHTTPRequestHandler

# HTML file sits next to this script (works both when run directly and after
# colcon install, since both files land in lib/<pkg>/)
_HTML_PATH = pathlib.Path(__file__).parent / "dashboard.html"

# ---------------------------------------------------------------------------
# Shared state (written by ROS callbacks, read by HTTP handler)
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()
_state = {
    # Topic health  (last recv wall time, message count for Hz)
    "topics": {
        "/vehicle/state":  {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/fix":       {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/velocity":  {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/odometry":  {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/esf_status":{"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
    },
    # Vehicle state (from /vehicle/state)
    "vehicle": {
        "v_ego_kmh": 0.0,
        "steering_deg": 0.0,
        "steering_torque": 0.0,
        "wheel_left_mps": 0.0,
        "wheel_right_mps": 0.0,
        "lat_active": False,
        "long_active": False,
        "actuators_accel": 0.0,
        "actuators_torque": 0.0,
        "car_output_accel": 0.0,
        "car_output_torque": 0.0,
        "timestamp_ns": 0,
    },
    # GNSS (from /gnss/fix + /gnss/velocity + /gnss/odometry)
    "gnss": {
        "latitude": 0.0,
        "longitude": 0.0,
        "altitude_m": 0.0,
        "fix_status": -1,
        "fix_label": "UNKNOWN",
        "h_acc_m": 0.0,
        "v_acc_m": 0.0,
        "vel_n_mps": 0.0,
        "vel_e_mps": 0.0,
        "vel_d_mps": 0.0,
        "ground_speed_mps": 0.0,
        "pos_x_m": 0.0,
        "pos_y_m": 0.0,
        "pos_z_m": 0.0,
        "heading_deg": 0.0,
    },
    # ESF calibration status (from /gnss/esf_status)
    "esf": {
        "fusion_mode": -1,
        "fusion_label": "UNKNOWN",
        "wt_label": "unknown",
        "mnt_alg_label": "unknown",
        "ins_label": "unknown",
        "imu_label": "unknown",
        "num_sens": 0,
        "sensors": [],
    },
    "uptime_s": 0.0,
}

_start_time_mono = time.monotonic()  # for uptime (monotonic, never jumps)

_HZ_WINDOW_SEC = 3.0  # rolling window for Hz estimation


def _touch_topic(name: str):
    wall = time.time()       # wall clock — JS uses Date.now()/1000, must match
    mono = time.monotonic()  # monotonic for Hz window (no jumps)
    t = _state["topics"][name]
    t["last_recv"] = wall
    t["count"] += 1
    win = t["_hz_window"]
    win.append(mono)
    # Trim to window
    cutoff = mono - _HZ_WINDOW_SEC
    while win and win[0] < cutoff:
        win.pop(0)
    if len(win) >= 2:
        span = win[-1] - win[0]
        t["hz"] = round((len(win) - 1) / span, 1) if span > 0 else 0.0
    else:
        t["hz"] = 0.0


def _fix_label(status: int) -> str:
    # gnss_node.cpp maps:  STATUS_NO_FIX(-1)=no fix, STATUS_FIX(0)=3D/2D fix,
    # STATUS_SBAS_FIX(1)=RTK float, STATUS_GBAS_FIX(2)=RTK fixed / dead-reck.
    if status == -1:
        return "NO FIX"
    elif status == 0:
        return "FIX"
    elif status == 1:
        return "RTK FLOAT"
    elif status == 2:
        return "RTK FIXED"
    else:
        return f"STATUS {status}"


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_node")
        self.create_subscription(VehicleState,  "/vehicle/state",    self._cb_vehicle,  10)
        self.create_subscription(NavSatFix,     "/gnss/fix",         self._cb_gnss_fix, 10)
        self.create_subscription(TwistStamped,  "/gnss/velocity",    self._cb_velocity, 10)
        self.create_subscription(Odometry,      "/gnss/odometry",    self._cb_odom,     10)
        self.create_subscription(EsfStatus,     "/gnss/esf_status",  self._cb_esf,      10)

    def _cb_vehicle(self, msg: VehicleState):
        with _state_lock:
            _touch_topic("/vehicle/state")
            v = _state["vehicle"]
            v["v_ego_kmh"]         = round(float(msg.v_ego), 2)
            v["steering_deg"]      = round(float(msg.steering_angle_deg), 2)
            v["steering_torque"]   = round(float(msg.steering_torque), 3)
            v["wheel_left_mps"]    = round(float(msg.rear_wheel_speed_left), 3)
            v["wheel_right_mps"]   = round(float(msg.rear_wheel_speed_right), 3)
            v["lat_active"]        = bool(msg.lat_active)
            v["long_active"]       = bool(msg.long_active)
            v["actuators_accel"]   = round(float(msg.actuators_accel), 3)
            v["actuators_torque"]  = round(float(msg.actuators_torque), 3)
            v["car_output_accel"]  = round(float(msg.car_output_accel), 3)
            v["car_output_torque"] = round(float(msg.car_output_torque), 3)
            v["timestamp_ns"]      = int(msg.timestamp)

    def _cb_gnss_fix(self, msg: NavSatFix):
        with _state_lock:
            _touch_topic("/gnss/fix")
            g = _state["gnss"]
            g["latitude"]   = round(msg.latitude,  7)
            g["longitude"]  = round(msg.longitude, 7)
            g["altitude_m"] = round(msg.altitude,  2)
            g["fix_status"] = int(msg.status.status)
            g["fix_label"]  = _fix_label(int(msg.status.status))
            cov = msg.position_covariance
            if msg.position_covariance_type >= 1:
                g["h_acc_m"] = round(math.sqrt(max(cov[0], 0.0)), 3)
                g["v_acc_m"] = round(math.sqrt(max(cov[8], 0.0)), 3)

    def _cb_velocity(self, msg: TwistStamped):
        with _state_lock:
            _touch_topic("/gnss/velocity")
            g = _state["gnss"]
            vn = msg.twist.linear.x
            ve = msg.twist.linear.y
            vd = -msg.twist.linear.z
            g["vel_n_mps"] = round(vn, 3)
            g["vel_e_mps"] = round(ve, 3)
            g["vel_d_mps"] = round(vd, 3)
            g["ground_speed_mps"] = round(math.sqrt(vn*vn + ve*ve), 3)

    def _cb_odom(self, msg: Odometry):
        with _state_lock:
            _touch_topic("/gnss/odometry")
            g = _state["gnss"]
            g["pos_x_m"] = round(msg.pose.pose.position.x, 2)
            g["pos_y_m"] = round(msg.pose.pose.position.y, 2)
            g["pos_z_m"] = round(msg.pose.pose.position.z, 2)
            # heading from quaternion (yaw in ENU)
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            g["heading_deg"] = round(math.degrees(yaw) % 360, 1)

    def _cb_esf(self, msg: EsfStatus):
        with _state_lock:
            _touch_topic("/gnss/esf_status")
            e = _state["esf"]
            e["fusion_mode"]   = int(msg.fusion_mode)
            e["fusion_label"]  = msg.fusion_label
            e["wt_label"]      = msg.wt_label
            e["mnt_alg_label"] = msg.mnt_alg_label
            e["ins_label"]     = msg.ins_label
            e["imu_label"]     = msg.imu_label
            e["num_sens"]      = int(msg.num_sens)
            e["sensors"] = [
                {
                    "type":      int(s.type),
                    "type_name": s.type_name,
                    "used":      bool(s.used),
                    "ready":     bool(s.ready),
                    "calib":     s.calib,
                    "time":      s.time_tag,
                    "freq":      int(s.freq),
                    "faults":    s.faults,
                }
                for s in msg.sensors
            ]


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
# HTML is served from dashboard.html (same directory as this script).
# Re-read on every request so you can edit it without restarting the server.


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # silence request logs

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            # Read from disk every request — edit dashboard.html and just refresh
            body = _HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/api/status":
            with _state_lock:
                # Build a clean serialisable copy (strip _hz_window)
                topics_clean = {}
                for name, info in _state["topics"].items():
                    topics_clean[name] = {
                        "last_recv": info["last_recv"],
                        "count":     info["count"],
                        "hz":        info["hz"],
                    }
                payload = {
                    "topics":   topics_clean,
                    "vehicle":  dict(_state["vehicle"]),
                    "gnss":     dict(_state["gnss"]),
                    "esf":      dict(_state["esf"]),
                    "uptime_s": round(time.monotonic() - _start_time_mono, 1),
                }
            body = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    PORT = 8080

    rclpy.init()
    node = DashboardNode()

    # HTTP server in background thread
    server = HTTPServer(("0.0.0.0", PORT), Handler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    print(f"[dashboard] Listening on http://localhost:{PORT}")
    print("[dashboard] Ctrl+C to stop")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
