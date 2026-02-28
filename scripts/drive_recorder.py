#!/usr/bin/env python3
"""
drive_recorder – record a manual drive to a CSV file for later replay.

Subscribes to gnss/pose (geometry_msgs/PoseStamped) and writes the
ENU x, y position to a CSV file at the specified rate.

Usage
-----
  # Start recording (saves to ~/drive_<timestamp>.csv by default)
  ros2 run car_control drive_recorder.py

  # Specify output file
  ros2 run car_control drive_recorder.py --ros-args -p output_file:=/tmp/my_drive.csv

  # Change recording rate (default 5 Hz – enough for 4 m/s baseline)
  ros2 run car_control drive_recorder.py --ros-args -p record_hz:=10.0

Replay
------
  ros2 launch car_control car_control_all.launch.py \\
      run_path_follower:=true \\
      path_csv_file:=/tmp/my_drive.csv
"""

import os
import csv
import time
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class DriveRecorder(Node):
    def __init__(self):
        super().__init__('drive_recorder')

        self.declare_parameter('output_file', '')
        self.declare_parameter('record_hz', 5.0)
        self.declare_parameter('min_dist_m', 0.5)   # skip if moved less than this

        output_file = self.get_parameter('output_file').get_parameter_value().string_value
        if not output_file:
            ts = time.strftime('%Y%m%d_%H%M%S')
            output_file = os.path.expanduser(f'~/drive_{ts}.csv')

        record_hz = self.get_parameter('record_hz').get_parameter_value().double_value
        self._min_dist = self.get_parameter('min_dist_m').get_parameter_value().double_value

        self._output_file = os.path.abspath(output_file)
        self._file = open(self._output_file, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(['x', 'y'])

        self._last_x = None
        self._last_y = None
        self._count = 0
        self._record_interval = 1.0 / record_hz
        self._last_record_time = 0.0

        self._sub = self.create_subscription(
            PoseStamped,
            'gnss/pose',
            self._pose_callback,
            10
        )

        self.get_logger().info(f'DriveRecorder started – recording to: {self._output_file}')
        self.get_logger().info(
            f'Rate: {record_hz:.1f} Hz  |  Min distance filter: {self._min_dist:.2f} m'
        )
        self.get_logger().info('Drive the car!  Ctrl+C to stop and save.')

    def _pose_callback(self, msg: PoseStamped):
        now = self.get_clock().now().nanoseconds * 1e-9

        if now - self._last_record_time < self._record_interval:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        # Distance filter to avoid duplicate waypoints when stationary
        if self._last_x is not None:
            dx = x - self._last_x
            dy = y - self._last_y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < self._min_dist:
                return

        self._writer.writerow([f'{x:.4f}', f'{y:.4f}'])
        self._last_x = x
        self._last_y = y
        self._last_record_time = now
        self._count += 1

        if self._count % 20 == 0:
            self.get_logger().info(
                f'Recorded {self._count} waypoints  (last: x={x:.2f}, y={y:.2f})'
            )

    def shutdown(self):
        self._file.flush()
        self._file.close()
        self.get_logger().info(
            f'Recording stopped – {self._count} waypoints saved to {self._output_file}'
        )


def main():
    rclpy.init()
    node = DriveRecorder()

    def _sigint(sig, frame):
        node.shutdown()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
