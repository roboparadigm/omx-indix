#!/usr/bin/env python3

import math
import select
import sys
import termios
import tty
import time

from st3215 import ST3215


SERVO_PORT = "/dev/ttyACM0"
SERVO_ID = 5

# ST3215: 0–4095 → 0–360°
TICKS_PER_REV = 4095.0

MIN_ANGLE = 0.0
MAX_ANGLE = 359.0  # clamp here instead of wrapping at 360


def ticks_to_deg(ticks: int) -> float:
    return ticks * 360.0 / TICKS_PER_REV


def deg_to_ticks(deg: float) -> int:
    # normalize and clip to [0, 4095]
    ticks = int(round(deg * TICKS_PER_REV / 360.0))
    return max(0, min(4095, ticks))


def clamp_angle(angle: float) -> float:
    """Clamp angle to [MIN_ANGLE, MAX_ANGLE]."""
    return max(MIN_ANGLE, min(MAX_ANGLE, angle))


def get_key(timeout=0.05):
    """Non-blocking single key read with timeout."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    print(f"Connecting to ST3215 on {SERVO_PORT}...")
    servo = ST3215(SERVO_PORT)

    # Enable torque
    servo.StartServo(SERVO_ID)

    # Try to read current position; if fail, start at mid (2048)
    pos = servo.ReadPosition(SERVO_ID)
    if pos is None:
        pos = 2048  # mid-point
    angle = clamp_angle(ticks_to_deg(pos))

    print("\nST3215 teleop ready.")
    print("Controls:")
    print("  a : -2°")
    print("  d : +2°")
    print("  z : -10°")
    print("  x : +10°")
    print("  s : stop (torque off)")
    print("  ESC : quit\n")

    try:
        while True:
            key = get_key()

            if key is None:
                continue

            # ESC
            if key == "\x1b":
                print("ESC pressed, quitting...")
                break

            prev_angle = angle

            if key == "a":
                angle -= 2.0
            elif key == "d":
                angle += 2.0
            elif key == "z":
                angle -= 10.0
            elif key == "x":
                angle += 10.0
            elif key == "s":
                print("Stopping servo (torque off)")
                servo.StopServo(SERVO_ID)
                continue  # don't send MoveTo
            else:
                continue

            # clamp angle into [0, 359]
            angle = clamp_angle(angle)

            # Optional: tell you when a move was clamped
            if angle == prev_angle:
                print(f"Limit reached at {angle:.2f}°, ignoring further move.")
                continue

            ticks = deg_to_ticks(angle)

            print(f"Target: {angle:6.2f}°  ({ticks} ticks)")
            servo.MoveTo(SERVO_ID, ticks)

            # small delay to avoid spamming bus
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nCtrl+C detected, exiting...")

    # Cleanup
    try:
        servo.StopServo(SERVO_ID)
    except Exception:
        pass

    try:
        servo.portHandler.closePort()
    except Exception:
        pass

    print("Done.")


if __name__ == "__main__":
    main()
