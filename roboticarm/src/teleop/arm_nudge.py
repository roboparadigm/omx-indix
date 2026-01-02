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
    def __init__(self, sim_gripper: bool = False):
        super().__init__("arm_nudge")
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        # Waypoints (joint1..joint6): ready -> wrist down + open -> wrist up + close
        # Within target limits: j1 +/-45deg, j2 (-90..+45), j3 +/-45, j4 +/-90.
        # joint5 is wrist roll placeholder (kept at 0).
        # For hardware: joint6 is revolute servo6; closed=5.81 rad (~333 deg), open=3.49 rad (~200 deg).
        # For sim: joint6 is prismatic; closed=0.0 m, open=0.019 m.
        closed = 0.0 if sim_gripper else 5.81
        open_ = 0.019 if sim_gripper else 3.49
        if sim_gripper:
            # Sim: both prismatic fingers commanded with same sign (axis handles direction).
            self.waypoints = [
                [0.0, 0.35, -0.35, 0.35, 0.0, closed, closed],   # ready, closed
                [0.0, -1.0, 0.6, 0.2, 0.0, open_, open_],        # wrist down, open
                [0.0, 0.2, -0.5, 0.6, 0.0, closed, closed],      # wrist up, close
            ]
            self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_right_joint"]
        else:
            self.waypoints = [
                [0.0, 0.35, -0.35, 0.35, 0.0, closed],    # ready, closed
                [0.0, -1.0, 0.6, 0.2, 0.0, open_],        # wrist down, open
                [0.0, 0.2, -0.5, 0.6, 0.0, closed],       # wrist up, close
            ]
            self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        self.index = 0
        self.timer = self.create_timer(2.0, self.publish_next)

    def publish_next(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        p = JointTrajectoryPoint()
        p.positions = self.waypoints[self.index]
        p.time_from_start.sec = int(self.duration_sec)
        p.time_from_start.nanosec = int((self.duration_sec - int(self.duration_sec)) * 1e9)
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
    parser.add_argument("--duration_sec", type=float, default=6.0, help="time_from_start for each nudge waypoint (seconds)")
    parser.add_argument("--sim_gripper", action="store_true", help="Use prismatic gripper values for sim (joint6 in meters)")
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
    node = ArmNudge(sim_gripper=args.sim_gripper)
    node.duration_sec = args.duration_sec
    rclpy.spin(node)

if __name__ == "__main__":
    main()