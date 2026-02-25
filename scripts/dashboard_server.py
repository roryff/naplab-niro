#!/usr/bin/env python3
"""
Car System Web Dashboard  –  http://localhost:8080

Extends the original GNSS/ESF dashboard with full path-follower telemetry:
  • ENU map  (path waypoints + vehicle position/trail + cross-track error)
  • Steering gauge  (target front-axle angle vs measured)
  • Error sparklines  (lateral error, heading error, speed history)
  • Controller status card  (IDLE / FOLLOWING / STOPPING)

New ROS 2 topics consumed
--------------------------
  /cmd_vel                   geometry_msgs/Twist      – controller output
  /path_visualization        nav_msgs/Path            – ENU path waypoints
  /lateral_error             std_msgs/Float64         – cross-track error [m]
  /heading_error             std_msgs/Float64         – heading error [rad]
  /path_following_status     std_msgs/Bool            – active flag

HTTP endpoints
--------------
  GET /            →  dashboard.html
  GET /api/status  →  full JSON state (vehicle, gnss, esf, controller, topics, uptime)
  GET /api/path    →  {waypoints: [{x,y},...], length_m, updated_at}
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float64
from car_control.msg import VehicleState, EsfStatus

import threading
import json
import time
import math
import pathlib
from http.server import HTTPServer, BaseHTTPRequestHandler

_HTML_PATH = pathlib.Path(__file__).parent / "dashboard.html"

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()

_MAX_HISTORY = 300   # samples kept for sparklines

_state = {
    # ── Topic health ─────────────────────────────────────────────────────────
    "topics": {
        "/vehicle/state":          {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/fix":               {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/velocity":          {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/odometry":          {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/esf_status":        {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/cmd_vel":                {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/path_visualization":     {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/lateral_error":          {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/path_following_status":  {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
    },

    # ── Vehicle state ────────────────────────────────────────────────────────
    "vehicle": {
        "v_ego_kmh":         0.0,
        "steering_deg":      0.0,
        "steering_torque":   0.0,
        "wheel_left_mps":    0.0,
        "wheel_right_mps":   0.0,
        "lat_active":        False,
        "long_active":       False,
        "actuators_accel":   0.0,
        "actuators_torque":  0.0,
        "car_output_accel":  0.0,
        "car_output_torque": 0.0,
        "timestamp_ns":      0,
    },

    # ── GNSS ─────────────────────────────────────────────────────────────────
    "gnss": {
        "latitude":         0.0,
        "longitude":        0.0,
        "altitude_m":       0.0,
        "fix_status":       -1,
        "fix_label":        "UNKNOWN",
        "h_acc_m":          0.0,
        "v_acc_m":          0.0,
        "vel_n_mps":        0.0,
        "vel_e_mps":        0.0,
        "vel_d_mps":        0.0,
        "ground_speed_mps": 0.0,
        "pos_x_m":          0.0,
        "pos_y_m":          0.0,
        "pos_z_m":          0.0,
        "heading_deg":      0.0,
    },

    # ── ESF calibration ──────────────────────────────────────────────────────
    "esf": {
        "fusion_mode":   -1,
        "fusion_label":  "UNKNOWN",
        "wt_label":      "unknown",
        "mnt_alg_label": "unknown",
        "ins_label":     "unknown",
        "imu_label":     "unknown",
        "num_sens":      0,
        "sensors":       [],
    },

    # ── Path-follower controller ──────────────────────────────────────────────
    "controller": {
        "active":             False,
        "state_label":        "IDLE",
        "target_speed_mps":   0.0,
        "target_steer_deg":   0.0,     # front-axle angle from cmd_vel [deg]
        "lateral_error_m":    0.0,
        "heading_error_deg":  0.0,
        "lat_err_history":    [],      # [{t, v}, ...]
        "hdg_err_history":    [],
        "speed_history":      [],
        "steer_cmd_history":  [],
    },

    "uptime_s": 0.0,
}

# Path waypoints are large – served via a separate /api/path endpoint
_path_lock = threading.Lock()
_path_data  = {"waypoints": [], "length_m": 0.0, "updated_at": 0.0}

_start_time_mono = time.monotonic()
_HZ_WINDOW_SEC   = 3.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _touch_topic(name: str):
    wall = time.time()
    mono = time.monotonic()
    t = _state["topics"][name]
    t["last_recv"] = wall
    t["count"] += 1
    win = t["_hz_window"]
    win.append(mono)
    cutoff = mono - _HZ_WINDOW_SEC
    while win and win[0] < cutoff:
        win.pop(0)
    if len(win) >= 2:
        span = win[-1] - win[0]
        t["hz"] = round((len(win) - 1) / span, 1) if span > 0 else 0.0
    else:
        t["hz"] = 0.0


def _push_history(buf: list, val: float):
    """Append {t, v} sample; cap at _MAX_HISTORY."""
    buf.append({"t": round(time.monotonic() - _start_time_mono, 2), "v": round(val, 4)})
    if len(buf) > _MAX_HISTORY:
        del buf[0]


def _fix_label(status: int) -> str:
    if status == -1: return "NO FIX"
    if status == 0:  return "FIX"
    if status == 1:  return "RTK FLOAT"
    if status == 2:  return "RTK FIXED"
    return f"STATUS {status}"


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------
class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_node")

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Existing topics
        self.create_subscription(VehicleState, "/vehicle/state",   self._cb_vehicle,  10)
        self.create_subscription(NavSatFix,    "/gnss/fix",        self._cb_gnss_fix, 10)
        self.create_subscription(TwistStamped, "/gnss/velocity",   self._cb_velocity, 10)
        self.create_subscription(Odometry,     "/gnss/odometry",   self._cb_odom,     10)
        self.create_subscription(EsfStatus,    "/gnss/esf_status", self._cb_esf,      10)

        # Path-follower topics
        self.create_subscription(Twist,   "/cmd_vel",               self._cb_cmd_vel,   10)
        self.create_subscription(Path,    "/path_visualization",    self._cb_path,      latched_qos)
        self.create_subscription(Float64, "/lateral_error",         self._cb_lat_err,   10)
        self.create_subscription(Float64, "/heading_error",         self._cb_hdg_err,   10)
        self.create_subscription(Bool,    "/path_following_status", self._cb_pf_status, latched_qos)

    # ── Existing callbacks ───────────────────────────────────────────────────

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
            _push_history(_state["controller"]["speed_history"], float(msg.v_ego) / 3.6)

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
            vn, ve = msg.twist.linear.x, msg.twist.linear.y
            g["vel_n_mps"]        = round(vn, 3)
            g["vel_e_mps"]        = round(ve, 3)
            g["vel_d_mps"]        = round(-msg.twist.linear.z, 3)
            g["ground_speed_mps"] = round(math.sqrt(vn*vn + ve*ve), 3)

    def _cb_odom(self, msg: Odometry):
        with _state_lock:
            _touch_topic("/gnss/odometry")
            g = _state["gnss"]
            g["pos_x_m"] = round(msg.pose.pose.position.x, 2)
            g["pos_y_m"] = round(msg.pose.pose.position.y, 2)
            g["pos_z_m"] = round(msg.pose.pose.position.z, 2)
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
                {"type": int(s.type), "type_name": s.type_name,
                 "used": bool(s.used), "ready": bool(s.ready),
                 "calib": s.calib, "time": s.time_tag,
                 "freq": int(s.freq), "faults": s.faults}
                for s in msg.sensors
            ]

    # ── Path-follower callbacks ──────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        with _state_lock:
            _touch_topic("/cmd_vel")
            c = _state["controller"]
            c["target_speed_mps"] = round(float(msg.linear.x), 3)
            deg = math.degrees(float(msg.angular.z))
            c["target_steer_deg"] = round(deg, 3)
            _push_history(c["steer_cmd_history"], deg)

    def _cb_lat_err(self, msg: Float64):
        with _state_lock:
            _touch_topic("/lateral_error")
            c = _state["controller"]
            c["lateral_error_m"] = round(float(msg.data), 4)
            _push_history(c["lat_err_history"], float(msg.data))

    def _cb_hdg_err(self, msg: Float64):
        with _state_lock:
            c = _state["controller"]
            c["heading_error_deg"] = round(math.degrees(float(msg.data)), 3)
            _push_history(c["hdg_err_history"], math.degrees(float(msg.data)))

    def _cb_pf_status(self, msg: Bool):
        with _state_lock:
            _touch_topic("/path_following_status")
            c = _state["controller"]
            c["active"]      = bool(msg.data)
            c["state_label"] = "FOLLOWING" if msg.data else "IDLE"

    def _cb_path(self, msg: Path):
        with _state_lock:
            _touch_topic("/path_visualization")

        poses  = msg.poses
        n      = len(poses)
        stride = max(1, n // 500)
        wpts = [{"x": round(p.pose.position.x, 2), "y": round(p.pose.position.y, 2)}
                for p in poses[::stride]]
        if n > 0 and (n - 1) % stride != 0:
            last = poses[-1]
            wpts.append({"x": round(last.pose.position.x, 2),
                         "y": round(last.pose.position.y, 2)})

        length_m = sum(
            math.hypot(wpts[i]["x"] - wpts[i-1]["x"], wpts[i]["y"] - wpts[i-1]["y"])
            for i in range(1, len(wpts))
        )
        with _path_lock:
            _path_data["waypoints"] = wpts
            _path_data["length_m"]  = round(length_m, 1)
            _path_data["updated_at"] = time.time()


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = _HTML_PATH.read_bytes()
            self._respond(200, "text/html; charset=utf-8", body)

        elif self.path == "/api/status":
            with _state_lock:
                topics_clean = {
                    name: {"last_recv": info["last_recv"],
                           "count":     info["count"],
                           "hz":        info["hz"]}
                    for name, info in _state["topics"].items()
                }
                ctrl = dict(_state["controller"])
                ctrl["lat_err_history"]   = list(ctrl["lat_err_history"])
                ctrl["hdg_err_history"]   = list(ctrl["hdg_err_history"])
                ctrl["speed_history"]     = list(ctrl["speed_history"])
                ctrl["steer_cmd_history"] = list(ctrl["steer_cmd_history"])
                payload = {
                    "topics":     topics_clean,
                    "vehicle":    dict(_state["vehicle"]),
                    "gnss":       dict(_state["gnss"]),
                    "esf":        dict(_state["esf"]),
                    "controller": ctrl,
                    "uptime_s":   round(time.monotonic() - _start_time_mono, 1),
                }
            self._respond_json(payload)

        elif self.path == "/api/path":
            with _path_lock:
                payload = {"waypoints": list(_path_data["waypoints"]),
                           "length_m":  _path_data["length_m"],
                           "updated_at": _path_data["updated_at"]}
            self._respond_json(payload)

        else:
            self.send_response(404)
            self.end_headers()

    def _respond(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _respond_json(self, obj):
        self._respond(200, "application/json", json.dumps(obj).encode())


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    PORT = 8080
    rclpy.init()
    node = DashboardNode()

    server = HTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

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
