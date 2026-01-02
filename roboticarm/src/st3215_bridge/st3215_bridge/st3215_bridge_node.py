#!/usr/bin/env python3
import math
import time
import serial
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# Import the bundled driver package (installed as top-level st3215)
from st3215 import ST3215


TICKS_PER_REV = 4095.0
DEG_PER_TICK = 360.0 / TICKS_PER_REV
RAD_PER_TICK = (2.0 * math.pi) / TICKS_PER_REV

# Per-servo angle limits in degrees
DEFAULT_MIN_ANGLE = 0.0
DEFAULT_MAX_ANGLE = 359.0
SERVO_LIMITS = {
    6: (85.0, 345.0),  # servo 6 narrower range
}


def clamp_angle(sid: int, angle_deg: float) -> float:
    min_a, max_a = SERVO_LIMITS.get(sid, (DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE))
    return max(min_a, min(max_a, angle_deg))


def ticks_to_rad(ticks: int) -> float:
    return ticks * RAD_PER_TICK


def rad_to_ticks(rad: float) -> int:
    # Normalize into [0, 2pi) then to ticks and clamp
    revs = (rad % (2.0 * math.pi)) / (2.0 * math.pi)
    ticks = int(round(revs * TICKS_PER_REV))
    return max(0, min(int(TICKS_PER_REV), ticks))


class ST3215Bridge(Node):
    def __init__(self):
        super().__init__('st3215_bridge')

        # Parameters
        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('torque_on_start', True)
        self.declare_parameter('torque_off_on_exit', True)
        self.declare_parameter('servo_speed_steps_per_s', 400.0)
        self.declare_parameter('servo_acc_steps_per_s2', 20.0)

        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        self.servo_ids: List[int] = list(self.get_parameter('servo_ids').get_parameter_value().integer_array_value)
        self.joint_names: List[str] = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)
        publish_rate = float(self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        torque_on_start = bool(self.get_parameter('torque_on_start').get_parameter_value().bool_value)
        self.torque_off_on_exit = bool(self.get_parameter('torque_off_on_exit').get_parameter_value().bool_value)
        self.servo_speed = int(self.get_parameter('servo_speed_steps_per_s').get_parameter_value().double_value)
        self.servo_acc = int(self.get_parameter('servo_acc_steps_per_s2').get_parameter_value().double_value)

        if len(self.servo_ids) != len(self.joint_names):
            self.get_logger().error('servo_ids and joint_names must have the same length')
            raise RuntimeError('Invalid mapping lengths')

        # Connect to hardware
        self.get_logger().info(f'Connecting to ST3215 on {port_name}...')
        self.driver = ST3215(port_name)

        # Enable torque and build maps
        self.angle_deg: Dict[int, float] = {}
        if torque_on_start:
            for sid in self.servo_ids:
                try:
                    self.driver.StartServo(sid)
                except Exception as exc:  # pragma: no cover - hardware path
                    self.get_logger().warn(f'Failed to start servo {sid}: {exc}')

        for sid in self.servo_ids:
            pos = self.driver.ReadPosition(sid)
            if pos is None:
                pos = int(TICKS_PER_REV / 2)
            angle = clamp_angle(sid, pos * DEG_PER_TICK)
            self.angle_deg[sid] = angle

        # Publishers/subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.trajectory_sub = self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory', self.on_trajectory, 10)

        # Timer for joint_states
        period = 1.0 / publish_rate if publish_rate > 0 else 0.1
        self.timer = self.create_timer(period, self.publish_joint_states)

        self.get_logger().info('st3215_bridge ready: publishing joint_states and listening to /arm_controller/joint_trajectory')

    def publish_joint_states(self):
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.name = self.joint_names

        positions = []
        for sid in self.servo_ids:
            try:
                pos = self.driver.ReadPosition(sid)
            except Exception as exc:  # pragma: no cover - hardware/serial path
                self.get_logger().warn(f'Serial read failed for servo {sid}: {exc}')
                pos = None
            if pos is None:
                # If read fails, reuse last angle
                angle_deg = self.angle_deg.get(sid, 0.0)
            else:
                angle_deg = pos * DEG_PER_TICK
                self.angle_deg[sid] = angle_deg
            positions.append(angle_deg * math.pi / 180.0)

        msg.position = positions
        self.joint_state_pub.publish(msg)

    def on_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            return

        # Use the last point in the trajectory as goal
        point = msg.points[-1]
        name_to_idx = {name: i for i, name in enumerate(msg.joint_names)}

        for sid, jname in zip(self.servo_ids, self.joint_names):
            if jname not in name_to_idx:
                continue
            idx = name_to_idx[jname]
            if idx >= len(point.positions):
                continue
            target_rad = point.positions[idx]
            target_deg = math.degrees(target_rad)
            target_deg = clamp_angle(sid, target_deg)
            ticks = rad_to_ticks(math.radians(target_deg))
            try:
                self.driver.MoveTo(sid, ticks, speed=self.servo_speed, acc=self.servo_acc)
                self.angle_deg[sid] = target_deg
            except Exception as exc:  # pragma: no cover - hardware path
                self.get_logger().warn(f'Failed to command servo {sid}: {exc}')

    def safe_stop(self):
        try:
            if self.torque_off_on_exit:
                for sid in self.servo_ids:
                    try:
                        self.driver.StopServo(sid)
                    except Exception:
                        pass
            try:
                self.driver.portHandler.closePort()
            except Exception:
                pass
        except Exception:
            pass

    def destroy_node(self):  # type: ignore[override]
        self.safe_stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ST3215Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupt received: stopping and torque off')
        node.safe_stop()
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


if __name__ == '__main__':
    main()
