#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty
from typing import Dict, List

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# Joint order expected by the bridge
JOINT_NAMES: List[str] = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]

# Limits (degrees)
DEFAULT_MIN_ANGLE = 0.0
DEFAULT_MAX_ANGLE = 359.0
SERVO_LIMITS = {6: (85.0, 345.0)}  # servo 6 narrower window

STEP_DEG = 1.0
BIG_STEP_DEG = 5.0
POSE_TIME_SEC = 5.0  # slower duration for home/ready moves

# Stored poses (radians)
HOME_POSE_RAD = {
    "joint1": 3.141,
    "joint2": 3.132,
    "joint3": 3.132,
    "joint4": 3.142,
    "joint5": 0.0,
    "joint6": 5.81,  # neutral/closed (~333 deg) within 85-345 deg limit
}

READY_POSE_RAD = {
    "joint1": 3.139,
    "joint2": 3.317,
    "joint3": 3.567,
    "joint4": 2.097,
    "joint5": 0.0,
    "joint6": 5.81,  # same neutral/closed value
}


def clamp_angle(sid: int, angle_deg: float) -> float:
    min_a, max_a = SERVO_LIMITS.get(sid, (DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE))
    return max(min_a, min(max_a, angle_deg))


def get_key(timeout: float = 0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class MultiTeleop(Node):
    def __init__(self):
        super().__init__("multi_teleop")
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.sub = self.create_subscription(JointState, "joint_states", self.on_joint_state, 10)

        # Populate from joint_states once available; fall back to mid if none ever arrive
        self.angle_deg: Dict[int, float] = {}
        self.last_js: Dict[str, float] = {}
        self.initialized_from_js = False

        # Key map: numbers increase, letters decrease
        self.key_map = {
            "1": (1, +STEP_DEG), "q": (1, -STEP_DEG),
            "2": (2, +STEP_DEG), "w": (2, -STEP_DEG),
            "3": (3, +STEP_DEG), "e": (3, -STEP_DEG),
            "4": (4, +STEP_DEG), "r": (4, -STEP_DEG),
            "5": (5, +STEP_DEG), "t": (5, -STEP_DEG),
            "6": (6, +STEP_DEG), "y": (6, -STEP_DEG),
            "z": (1, -BIG_STEP_DEG), "x": (1, +BIG_STEP_DEG),
            "c": (0, 0.0),  # capture current joint_states
            "h": (-1, 0.0),  # go to home pose
            "g": (-2, 0.0),  # go to ready pose
        }

        self.print_help()
        self.timer = self.create_timer(0.05, self.on_key)

    def print_help(self):
        print("ROS2 multi-servo teleop (publishes JointTrajectory)")
        print("Keys: 1/q 2/w 3/e 4/r 5/t 6/y (inc/dec) | z/x big step for joint1 | c capture current pose | h home | g ready | s stop timer | ESC quit")

    def on_joint_state(self, msg: JointState):
        self.last_js = {name: pos for name, pos in zip(msg.name, msg.position)}
        if not self.initialized_from_js:
            # Set baseline to current measured pose and report it once
            for idx, jname in enumerate(JOINT_NAMES, start=1):
                rad = self.last_js.get(jname)
                if rad is None:
                    continue
                self.angle_deg[idx] = math.degrees(rad)
            line = ", ".join(f"{jn}:{self.last_js.get(jn, float('nan')):.3f} rad" for jn in JOINT_NAMES)
            self.get_logger().info(f"Initial positions: {line}")
            self.initialized_from_js = True

    def on_key(self):
        key = get_key()
        if key is None:
            return

        if key == "\x1b":
            self.get_logger().info("ESC pressed, exiting")
            rclpy.shutdown()
            return

        if key == "s":
            self.get_logger().info("Stop command ignored (bridge manages torque); no action taken")
            return

        if key not in self.key_map:
            return

        sid, delta = self.key_map[key]
        if sid == -1:  # home
            self.apply_pose(HOME_POSE_RAD, label="home")
            return
        if sid == -2:  # ready
            self.apply_pose(READY_POSE_RAD, label="ready")
            return
        if sid == 0:  # capture current pose
            if not self.last_js:
                self.get_logger().warn("No joint_states received yet; cannot capture")
                return
            for idx, jname in enumerate(JOINT_NAMES, start=1):
                rad = self.last_js.get(jname)
                if rad is None:
                    continue
                self.angle_deg[idx] = math.degrees(rad)
            self.get_logger().info("Captured current joint_states as new baseline")
            # Print positions in radians
            line = ", ".join(f"{jn}:{self.last_js.get(jn, float('nan')):.3f} rad" for jn in JOINT_NAMES)
            self.get_logger().info(f"Current positions: {line}")
            return

        if not self.angle_deg and not self.initialized_from_js:
            self.get_logger().warn("No baseline yet; waiting for joint_states. Press 'c' after joint_states are available.")
            return

        prev = self.angle_deg.get(sid, 180.0)
        new_angle = clamp_angle(sid, prev + delta)
        if new_angle == prev:
            self.get_logger().warn(f"Servo {sid} at limit, ignoring")
            return

        self.angle_deg[sid] = new_angle
        self.publish_positions()
        self.get_logger().info(f"Servo {sid} -> {new_angle:.2f} deg")

    def apply_pose(self, pose_rad: Dict[str, float], label: str):
        for idx, jname in enumerate(JOINT_NAMES, start=1):
            rad = pose_rad.get(jname)
            if rad is None:
                continue
            self.angle_deg[idx] = math.degrees(rad)
        self.publish_positions(duration_sec=POSE_TIME_SEC)
        line = ", ".join(f"{jn}:{pose_rad.get(jn, float('nan')):.3f} rad" for jn in JOINT_NAMES)
        self.get_logger().info(f"Applied {label} pose -> {line}")

    def publish_positions(self, duration_sec: float = 1.0):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        # Fill positions in joint order
        positions = []
        for idx, jname in enumerate(JOINT_NAMES, start=1):
            ang = self.angle_deg.get(idx, 180.0)
            positions.append(math.radians(ang))
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        msg.points.append(point)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
