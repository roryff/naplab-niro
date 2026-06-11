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
  /cmd_vel                   car_control/DriveCommand  – controller output
  /path_visualization        nav_msgs/Path            – ENU path waypoints
  /lateral_mpc/status        car_control/MpcStatus    – CTE, heading, vref, desired steer
  /path_following_status     std_msgs/Bool            – active flag (latched)

HTTP endpoints
--------------
  GET /            →  dashboard.html
  GET /api/status  →  full JSON state (vehicle, gnss, esf, controller, topics, uptime)
  GET /api/path    →  {waypoints: [{x,y},...], length_m, updated_at}
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
from car_control.msg import DriveCommand
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from car_control.msg import VehicleState, EsfStatus, MpcStatus

import threading
import json
import time
import math
import pathlib
import csv
from http.server import HTTPServer, BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError
try:
    from ament_index_python.packages import get_package_share_directory as _get_pkg_share
except ImportError:
    _get_pkg_share = None

_HTML_PATH = pathlib.Path(__file__).parent / "dashboard.html"


# ---------------------------------------------------------------------------
# UTM zone 32N (EPSG:25832) conversion — mirrors geo_utils.hpp exactly
# ---------------------------------------------------------------------------
def _latlon_to_utm32(lat_deg: float, lon_deg: float):
    """Return (easting, northing) in metres for UTM zone 32N / EPSG:25832."""
    a   = 6378137.0
    f   = 1.0 / 298.257222101
    e2  = 2.0 * f - f * f
    ep2 = e2 / (1.0 - e2)
    k0  = 0.9996
    lon0 = 9.0 * math.pi / 180.0
    E0  = 500000.0

    phi  = math.radians(lat_deg)
    lam  = math.radians(lon_deg)
    dlam = lam - lon0

    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    tan_phi = math.tan(phi)

    N = a / math.sqrt(1.0 - e2 * sin_phi * sin_phi)
    T = tan_phi * tan_phi
    C = ep2 * cos_phi * cos_phi
    A = cos_phi * dlam

    e4 = e2 * e2;  e6 = e4 * e2
    M = a * (
        (1.0 - e2/4.0 - 3.0*e4/64.0  - 5.0*e6/256.0)  * phi
      - (3.0*e2/8.0  + 3.0*e4/32.0  + 45.0*e6/1024.0) * math.sin(2.0*phi)
      + (15.0*e4/256.0 + 45.0*e6/1024.0)               * math.sin(4.0*phi)
      - (35.0*e6/3072.0)                                * math.sin(6.0*phi))

    A2=A*A; A3=A2*A; A4=A2*A2; A5=A4*A; A6=A4*A2
    easting = k0*N*(A + (1.0-T+C)*A3/6.0
              + (5.0-18.0*T+T*T+72.0*C-58.0*ep2)*A5/120.0) + E0
    northing = k0*(M + N*tan_phi*(A2/2.0
               + (5.0-T+9.0*C+4.0*C*C)*A4/24.0
               + (61.0-58.0*T+T*T+600.0*C-330.0*ep2)*A6/720.0))
    return easting, northing

# ---------------------------------------------------------------------------
# WMTS tile proxy  (Norge i bilder satellite imagery)
# ---------------------------------------------------------------------------
_WMTS_BASE = (
    "https://tilecache.norgeibilder.no/arcgis/rest/services"
    "/Nibcache_UTM32_EUREF89_v2/MapServer/WMTS/tile/1.0.0"
    "/Nibcache_UTM32_EUREF89_v2/default/default028mm"
)
_WMTS_REFERER = "https://naplab"
_wmts_token   = "X2sctU5MKSsFC2rtPOI4U2gbNQRjZgLtPuoxgnQ_MbQGdh8h9ZjuMm9M_oDWQeun"

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
_state_lock = threading.Lock()

_MAX_HISTORY = 300   # samples kept for sparklines

_state = {
    # ── Topic health ─────────────────────────────────────────────────────────
    "topics": {
        "/vehicle/state":           {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/fix":                {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/velocity":           {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/odometry":           {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/esf_status":         {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/gyro":               {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/gnss/accel":              {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/lateral_mpc/status":      {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/cmd_vel":                 {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/path_visualization":      {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
        "/path_following_status":   {"last_recv": None, "count": 0, "hz": 0.0, "_hz_window": []},
    },

    # ── Vehicle state ────────────────────────────────────────────────────────
    "vehicle": {
        "v_ego_kmh":         0.0,
        "steering_deg":      0.0,
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
        "roll_deg":         0.0,
        "pitch_deg":        0.0,
        "gyro_x_rads":      0.0,
        "gyro_y_rads":      0.0,
        "gyro_z_rads":      0.0,
        "accel_x_ms2":      0.0,
        "accel_y_ms2":      0.0,
        "accel_z_ms2":      0.0,
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

    # ── Path-follower / MPC controller ────────────────────────────────────────
    "controller": {
        "active":              False,
        "state_label":         "IDLE",
        # path_follower_node output (path_follower/cmd_vel)
        "target_speed_mps":    0.0,
        "target_steer_deg":    0.0,    # desired front-axle angle [deg], positive = left
        # steering_mpc_node output (cmd_vel)
        "mpc_torque":          0.0,    # MPC torque command [-1, 1]
        "mpc_accel_cmd":       0.0,    # accel command from MPC speed P-ctrl [-1,1]
        # diagnostics
        "lateral_error_m":     0.0,
        "heading_error_deg":   0.0,
        "lat_err_history":     [],     # [{t, v}, ...]
        "hdg_err_history":     [],
        "speed_history":       [],
        "steer_cmd_history":   [],     # desired front-axle angle [deg]
        "torque_history":      [],     # MPC torque output
    },

    "uptime_s": 0.0,
    "path_info": {
        "loaded_name": None,
    },
}

# Path waypoints are large – served via a separate /api/path endpoint
_path_lock = threading.Lock()
_path_data  = {"waypoints": [], "length_m": 0.0, "updated_at": 0.0}

_start_time_mono = time.monotonic()
_HZ_WINDOW_SEC   = 3.0
_dashboard_node  = None   # set in main(), used by POST handler

# ---------------------------------------------------------------------------
# Paths directory helper
# ---------------------------------------------------------------------------
def _find_paths_dir() -> pathlib.Path:
    """Return the src/car_control/paths directory, creating it if needed."""
    ws_root = pathlib.Path(__file__).resolve().parents[4]
    src_pkg = ws_root / 'src' / 'car_control'
    if src_pkg.exists():
        d = src_pkg / 'paths'
        d.mkdir(parents=True, exist_ok=True)
        return d
    # fallback: walk up looking for package.xml
    p = pathlib.Path(__file__).resolve().parent
    while p != p.parent:
        if (p / 'package.xml').exists():
            d = p / 'paths'
            d.mkdir(parents=True, exist_ok=True)
            return d
        p = p.parent
    d = pathlib.Path(__file__).resolve().parent.parent / 'paths'
    d.mkdir(parents=True, exist_ok=True)
    return d


# ---------------------------------------------------------------------------
# Path recording state
# ---------------------------------------------------------------------------
_RECORD_HZ = 30.0  # sampling rate for waypoints — matches gnss/odometry rate

_rec_lock     = threading.Lock()
_rec_active   = False
_rec_file     = None
_rec_writer   = None
_rec_count    = 0
_rec_filename = None
_rec_thread   = None


def _recording_loop():
    """Background thread: samples gnss/odometry pos at _RECORD_HZ Hz, writes CSV."""
    global _rec_active, _rec_count
    interval = 1.0 / _RECORD_HZ
    while True:
        time.sleep(interval)
        with _rec_lock:
            if not _rec_active:
                break
        with _state_lock:
            x   = _state["gnss"]["pos_x_m"]
            y   = _state["gnss"]["pos_y_m"]
            fix = _state["gnss"]["fix_label"]
        if fix in ("NO FIX", "UNKNOWN"):
            continue
        with _rec_lock:
            if not _rec_active:
                break
            _rec_writer.writerow([f'{x:.4f}', f'{y:.4f}'])
            _rec_file.flush()
            _rec_count += 1


def _start_recording() -> dict:
    global _rec_active, _rec_file, _rec_writer, _rec_count, _rec_filename, _rec_thread
    with _rec_lock:
        if _rec_active:
            return {"ok": False, "error": "already recording"}
        # Always save recordings to the source tree paths directory so they
        # persist across rebuilds and are independent of the install tree layout.
        # When running from install/car_control/lib/car_control/, parents[4] is
        # the workspace root; src/car_control/paths is the target.
        _ws_root = pathlib.Path(__file__).resolve().parents[4]
        _src_pkg_root = _ws_root / 'src' / 'car_control'
        if not _src_pkg_root.exists():
            # Fallback: walk up from __file__ looking for package.xml
            _src_pkg_root = pathlib.Path(__file__).resolve().parent
            while _src_pkg_root != _src_pkg_root.parent:
                if (_src_pkg_root / 'package.xml').exists():
                    break
                _src_pkg_root = _src_pkg_root.parent
            else:
                _src_pkg_root = pathlib.Path(__file__).resolve().parent.parent
        paths_dir = _src_pkg_root / 'paths'
        paths_dir.mkdir(parents=True, exist_ok=True)
        ts       = time.strftime('%Y%m%d_%H%M%S')
        filename = paths_dir / f'recording_{ts}.csv'
        _rec_file   = open(filename, 'w', newline='')
        _rec_writer = csv.writer(_rec_file)
        _rec_writer.writerow(['x', 'y'])
        _rec_count  = 0
        _rec_filename = str(filename)
        _rec_active = True
        _rec_thread = threading.Thread(target=_recording_loop, daemon=True)
    _rec_thread.start()
    return {"ok": True, "filename": _rec_filename}


def _stop_recording() -> dict:
    global _rec_active, _rec_file, _rec_writer, _rec_filename, _rec_count
    with _rec_lock:
        if not _rec_active:
            return {"ok": False, "error": "not recording"}
        _rec_active = False
        count    = _rec_count
        filename = _rec_filename
        stem     = pathlib.Path(filename).stem if filename else ""
        if _rec_file:
            _rec_file.flush()
            _rec_file.close()
            _rec_file = None
    return {"ok": True, "filename": filename, "stem": stem, "waypoint_count": count}


def _rename_recording(new_name: str) -> dict:
    global _rec_filename
    import re, os
    with _rec_lock:
        src = _rec_filename
        if not src or not os.path.isfile(src):
            return {"ok": False, "error": "no recording to rename"}
        safe = re.sub(r'[^\w\-]', '_', new_name.strip()).strip('_')
        if not safe:
            return {"ok": False, "error": "invalid name"}
        dst = str(pathlib.Path(src).parent / f'{safe}.csv')
        try:
            os.rename(src, dst)
        except OSError as e:
            return {"ok": False, "error": str(e)}
        _rec_filename = None
    return {"ok": True, "filename": pathlib.Path(dst).name}


def _discard_recording() -> dict:
    global _rec_filename
    import os
    with _rec_lock:
        src = _rec_filename
        if not src:
            return {"ok": False, "error": "no recording to discard"}
        _rec_filename = None
        if os.path.isfile(src):
            try:
                os.remove(src)
            except OSError:
                pass
    return {"ok": True}


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

        # Map frame origin — must match gnss_node.yaml / lateral_mpc_node.yaml.
        # path_visualization waypoints are published in local (origin-subtracted) coords;
        # gnss/odometry is in absolute UTM32.  We add the origin back when storing
        # path waypoints so the dashboard overlay is consistent.
        self.declare_parameter("origin_lat", 63.4391)
        self.declare_parameter("origin_lon", 10.4128)
        lat = self.get_parameter("origin_lat").value
        lon = self.get_parameter("origin_lon").value
        self._map_origin_x, self._map_origin_y = _latlon_to_utm32(lat, lon)
        self.get_logger().info(
            f"Map origin: lat={lat} lon={lon} -> UTM32 E={self._map_origin_x:.2f} N={self._map_origin_y:.2f}")

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Existing topics
        self.create_subscription(VehicleState, "/vehicle/state",   self._cb_vehicle,  qos_profile_sensor_data)
        self.create_subscription(NavSatFix,    "/gnss/fix",        self._cb_gnss_fix, 10)
        self.create_subscription(TwistStamped, "/gnss/velocity",   self._cb_velocity, 10)
        self.create_subscription(Odometry,     "/gnss/odometry",   self._cb_odom,     10)
        self.create_subscription(EsfStatus,       "/gnss/esf_status", self._cb_esf,      10)
        self.create_subscription(Vector3Stamped,  "/gnss/gyro",       self._cb_gyro,     10)
        self.create_subscription(Vector3Stamped,  "/gnss/accel",      self._cb_accel,    10)

        # Path-follower topics
        # lateral_mpc/status: CTE [m], heading error [deg], vref [m/s], desired steer [deg].
        # Single diagnostic message; replaces the old lateral_error / heading_error /
        # path_follower/cmd_vel Float64+Twist topics (all subsets of MpcStatus).
        self.create_subscription(MpcStatus, "/lateral_mpc/status",    self._cb_mpc_status, 10)
        # cmd_vel: MPC torque [-1,1] + accel command [-1,1]
        self.create_subscription(DriveCommand, "/cmd_vel",            self._cb_cmd_vel,   10)
        self.create_subscription(Path,    "/path_visualization",     self._cb_path,      latched_qos)
        self.create_subscription(Bool,    "/path_following_status",  self._cb_pf_status, latched_qos)

        # Publisher – allows dashboard to start/stop path following
        self.enable_pub_ = self.create_publisher(Bool, "/enable_path_following", 1)
        # Publisher – allows dashboard to load a path at runtime
        self.load_path_pub_ = self.create_publisher(String, "/lateral_mpc/load_path", 1)

    # ── Existing callbacks ───────────────────────────────────────────────────

    def _cb_vehicle(self, msg: VehicleState):
        with _state_lock:
            _touch_topic("/vehicle/state")
            v = _state["vehicle"]
            v["v_ego_kmh"]         = round(float(msg.v_ego), 2)
            v["steering_deg"]      = round(float(msg.steering_angle_deg), 2)
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

    def _cb_gyro(self, msg: Vector3Stamped):
        with _state_lock:
            _touch_topic("/gnss/gyro")
            g = _state["gnss"]
            g["gyro_x_rads"] = round(msg.vector.x, 5)
            g["gyro_y_rads"] = round(msg.vector.y, 5)
            g["gyro_z_rads"] = round(msg.vector.z, 5)

    def _cb_accel(self, msg: Vector3Stamped):
        with _state_lock:
            _touch_topic("/gnss/accel")
            g = _state["gnss"]
            g["accel_x_ms2"] = round(msg.vector.x, 4)
            g["accel_y_ms2"] = round(msg.vector.y, 4)
            g["accel_z_ms2"] = round(msg.vector.z, 4)

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

    def _cb_mpc_status(self, msg: MpcStatus):
        """lateral_mpc/status: per-tick controller diagnostics (CTE, heading, vref, desired steer).
        Single source for what used to arrive on lateral_error / heading_error /
        path_follower/cmd_vel. desired_delta_deg and heading_error_deg are already in
        degrees, so no conversion is needed here."""
        with _state_lock:
            _touch_topic("/lateral_mpc/status")
            c = _state["controller"]
            c["target_speed_mps"]  = round(float(msg.vref_mps), 3)
            steer_deg = float(msg.desired_delta_deg)            # positive = left
            c["target_steer_deg"]  = round(steer_deg, 3)
            _push_history(c["steer_cmd_history"], steer_deg)
            c["lateral_error_m"]   = round(float(msg.cte_m), 4)
            _push_history(c["lat_err_history"], float(msg.cte_m))
            hdg_deg = float(msg.heading_error_deg)
            c["heading_error_deg"] = round(hdg_deg, 3)
            _push_history(c["hdg_err_history"], hdg_deg)

    def _cb_cmd_vel(self, msg: DriveCommand):
        """cmd_vel: torque [-1,1], accel cmd [-1,1]"""
        with _state_lock:
            _touch_topic("/cmd_vel")
            c = _state["controller"]
            c["mpc_torque"]    = round(float(msg.torque), 4)
            c["mpc_accel_cmd"] = round(float(msg.accel), 3)
            _push_history(c["torque_history"], float(msg.torque))

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
        ox, oy = self._map_origin_x, self._map_origin_y
        wpts = [{"x": round(p.pose.position.x + ox, 2), "y": round(p.pose.position.y + oy, 2)}
                for p in poses[::stride]]
        if n > 0 and (n - 1) % stride != 0:
            last = poses[-1]
            wpts.append({"x": round(last.pose.position.x + ox, 2),
                         "y": round(last.pose.position.y + oy, 2)})

        length_m = sum(
            math.hypot(wpts[i]["x"] - wpts[i-1]["x"], wpts[i]["y"] - wpts[i-1]["y"])
            for i in range(1, len(wpts))
        )
        with _path_lock:
            _path_data["waypoints"] = wpts
            _path_data["length_m"]  = round(length_m, 1)
            _path_data["updated_at"] = time.time()


# ---------------------------------------------------------------------------
# HTTP server (threaded so parallel tile requests don't block each other)
# ---------------------------------------------------------------------------
class _QuietThreadedServer(ThreadingHTTPServer):
    """ThreadingHTTPServer that silently drops BrokenPipe from cancelled tile fetches."""
    def handle_error(self, request, client_address):
        import sys
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_OPTIONS(self):
        """Handle CORS pre-flight."""
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

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
                ctrl["torque_history"]    = list(ctrl["torque_history"])
                payload = {
                    "topics":     topics_clean,
                    "vehicle":    dict(_state["vehicle"]),
                    "gnss":       dict(_state["gnss"]),
                    "esf":        dict(_state["esf"]),
                    "controller": ctrl,
                    "path_info":  dict(_state["path_info"]),
                    "uptime_s":   round(time.monotonic() - _start_time_mono, 1),
                }
            with _rec_lock:
                payload["recording"] = {
                    "active":        _rec_active,
                    "filename":      pathlib.Path(_rec_filename).name if _rec_filename else None,
                    "waypoint_count": _rec_count,
                }
            self._respond_json(payload)

        elif self.path == "/api/paths":
            paths_dir = _find_paths_dir()
            files = sorted(paths_dir.glob("*.csv"))
            with _state_lock:
                loaded = _state["path_info"]["loaded_name"]
            entries = [
                {
                    "name": f.name,
                    "size_kb": round(f.stat().st_size / 1024, 1),
                    "active": (f.name == loaded),
                }
                for f in files
            ]
            self._respond_json({"paths": entries, "dir": str(paths_dir)})

        elif self.path == "/api/path":
            with _path_lock:
                payload = {"waypoints": list(_path_data["waypoints"]),
                           "length_m":  _path_data["length_m"],
                           "updated_at": _path_data["updated_at"]}
            self._respond_json(payload)

        elif self.path.startswith("/tiles/"):
            if not _wmts_token:
                body = b"WMTS token not configured (create mapview/WMTS_TOKEN.txt)"
                self._respond(503, "text/plain", body)
                return
            rel = self.path[len("/tiles/"):].split("?")[0].strip("/")
            if rel.endswith(".png"):
                rel = rel[:-4]
            parts = rel.split("/")
            if len(parts) != 3:
                self.send_response(400); self.end_headers(); return
            z, y, x = parts
            wmts_url = f"{_WMTS_BASE}/{z}/{y}/{x}?token={_wmts_token}"
            req = Request(wmts_url, headers={
                "Referer": _WMTS_REFERER,
                "User-Agent": "car-dashboard/1.0",
            })
            try:
                with urlopen(req, timeout=20) as resp:
                    data = resp.read()
                    ct = resp.headers.get("Content-Type", "image/png")
                    self.send_response(resp.status)
                    self.send_header("Content-Type", ct)
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "public, max-age=86400")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except HTTPError as exc:
                self.send_error(exc.code, f"WMTS upstream: {exc.reason}")
            except (URLError, Exception) as exc:
                self.send_error(502, str(exc))

        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == "/api/load_path":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(body)
                name = str(data.get("name", "")).strip()
            except (json.JSONDecodeError, KeyError):
                self._respond(400, "application/json", b'{"error":"bad json"}')
                return
            if not name:
                self._respond(400, "application/json", b'{"error":"empty name"}')
                return
            # Resolve to the paths directory so only files inside it can be loaded
            paths_dir = _find_paths_dir()
            csv_path = (paths_dir / name).resolve()
            if not str(csv_path).startswith(str(paths_dir.resolve())):
                self._respond(400, "application/json", b'{"error":"invalid path"}')
                return
            if not csv_path.exists():
                self._respond_json({"ok": False, "error": f"{name} not found"})
                return
            msg = String()
            msg.data = str(csv_path)
            _dashboard_node.load_path_pub_.publish(msg)
            with _state_lock:
                _state["path_info"]["loaded_name"] = name
            _dashboard_node.get_logger().info(f"Loading path: {name}")
            self._respond_json({"ok": True, "path": str(csv_path)})

        elif self.path == "/api/unload_path":
            msg = String()
            msg.data = ""  # empty string signals the node to clear the path
            _dashboard_node.load_path_pub_.publish(msg)
            with _state_lock:
                _state["path_info"]["loaded_name"] = None
            _dashboard_node.get_logger().info("Path unloaded")
            self._respond_json({"ok": True})

        elif self.path == "/api/enable_path_following":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(body)
                enable = bool(data.get("enable", True))
            except (json.JSONDecodeError, KeyError):
                self._respond(400, "application/json", b'{"error":"bad json"}')
                return
            msg = Bool()
            msg.data = enable
            _dashboard_node.enable_pub_.publish(msg)
            self._respond_json({"ok": True, "enable": enable})

        elif self.path == "/api/start_recording":
            self._respond_json(_start_recording())

        elif self.path == "/api/stop_recording":
            self._respond_json(_stop_recording())

        elif self.path == "/api/rename_recording":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(body)
                name = str(data.get("name", ""))
            except (json.JSONDecodeError, KeyError):
                self._respond(400, "application/json", b'{"error":"bad json"}')
                return
            self._respond_json(_rename_recording(name))

        elif self.path == "/api/discard_recording":
            self._respond_json(_discard_recording())

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
    global _dashboard_node
    PORT = 5000
    rclpy.init()
    node = DashboardNode()
    _dashboard_node = node

    server = _QuietThreadedServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    import socket as _socket
    _hostname = _socket.gethostname()
    try:
        _ip = _socket.gethostbyname(_hostname)
    except Exception:
        _ip = "localhost"
    node.get_logger().info(f"Dashboard running at http://{_hostname}:{PORT}  |  http://{_ip}:{PORT}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
