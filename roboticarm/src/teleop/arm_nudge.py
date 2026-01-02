#!/usr/bin/env python3
import argparse
import math

import os
import sys

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Prefer installed st3215; fall back to bundled source if overlay not sourced
try:
    from st3215 import ST3215
except ImportError:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate = os.path.abspath(os.path.join(script_dir, "..", "st3215_bridge"))
    if candidate not in sys.path and os.path.exists(os.path.join(candidate, "st3215")):
        sys.path.insert(0, candidate)
    from st3215 import ST3215

class ArmNudge(Node):
    def __init__(self):
        super().__init__("arm_nudge")
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        # Waypoints to cycle through (joint1..joint4)
        self.waypoints = [
            [0.0, 0.6, -0.8, 0.3],
            [0.4, 0.2, -0.6, 0.1],
            [-0.4, 0.5, -0.5, 0.2],
            [0.0, 0.3, -0.9, 0.4],
        ]
        self.index = 0

        self.timer = self.create_timer(2.0, self.publish_next)

    def publish_next(self):
        msg = JointTrajectory()
        msg.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        p = JointTrajectoryPoint()
        p.positions = self.waypoints[self.index]
        p.time_from_start.sec = 2
        msg.points.append(p)

        self.pub.publish(msg)
        self.get_logger().info(f"Sent waypoint {self.index + 1}/{len(self.waypoints)}: {p.positions}")

        # Advance index cyclically
        self.index = (self.index + 1) % len(self.waypoints)

def main():
    parser = argparse.ArgumentParser(description="Nudge joints or check servo mid positions")
    parser.add_argument("--mode", choices=["nudge", "check"], default="check", help="Operation mode")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Servo port")
    parser.add_argument("--mid_tol_deg", type=float, default=5.0, help="Tolerance around mid for OK status")
    args = parser.parse_args()

    if args.mode == "check":
        servo_ids = [1, 2, 3, 4, 6]
        mid_angles = {sid: ((85 + 345) / 2.0 if sid == 6 else 180.0) for sid in servo_ids}
        drv = ST3215(args.port)
        print(f"Reading positions on {args.port}")
        for sid in servo_ids:
            pos = drv.ReadPosition(sid)
            if pos is None:
                print(f"Servo {sid}: read failed")
                continue
            angle_deg = pos * (360.0 / 4095.0)
            mid = mid_angles[sid]
            diff = angle_deg - mid
            status = "OK" if abs(diff) < args.mid_tol_deg else "OFF"
            print(f"Servo {sid}: angle={angle_deg:7.2f} deg, mid={mid:7.2f} deg, delta={diff:7.2f} deg {status}")
        drv.portHandler.closePort()
        return

    rclpy.init()
    rclpy.spin(ArmNudge())

if __name__ == "__main__":
    main()