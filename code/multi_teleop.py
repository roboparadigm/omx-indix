#!/usr/bin/env python3

import select
import sys
import termios
import tty
import time

from st3215 import ST3215


SERVO_PORT = "/dev/ttyACM0"
SERVOS = [1, 2, 3, 4, 5, 6]

# ST3215: 0–4095 → 0–360°
TICKS_PER_REV = 4095.0  # keep consistent with your single-servo version
DEFAULT_TICKS = 2048    # mid-point

STEP_DEG = 1.0          # step per keypress
BIG_STEP_DEG = 10.0     # optional bigger step (you can remove if not needed)

# Default limits for all servos
DEFAULT_MIN_ANGLE = 0.0
DEFAULT_MAX_ANGLE = 359.0  # clamp here instead of wrapping at 360

# Per-servo angle limits (only servo 6 is restricted)
SERVO_LIMITS = {
    6: (85.0, 345.0),  # servo 6 only: must stay between 85° and 345°
}


def ticks_to_deg(ticks: int) -> float:
    return ticks * 360.0 / TICKS_PER_REV


def deg_to_ticks(deg: float) -> int:
    ticks = int(round(deg * TICKS_PER_REV / 360.0))
    return max(0, min(4095, ticks))


def clamp_angle_for_servo(sid: int, angle: float) -> float:
    """Clamp angle to the allowed range for a given servo."""
    min_a, max_a = SERVO_LIMITS.get(sid, (DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE))
    return max(min_a, min(max_a, angle))


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

    # Enable torque on all servos and initialize angle map
    angle_map = {}
    print("Enabling torque and reading initial positions...")
    for sid in SERVOS:
        servo.StartServo(sid)
        pos = servo.ReadPosition(sid)
        if pos is None:
            pos = DEFAULT_TICKS

        angle = clamp_angle_for_servo(sid, ticks_to_deg(pos))
        angle_map[sid] = angle
        print(f"  Servo {sid}: start at {angle:6.2f}° ({pos} ticks)")

    print("\nST3215 multi-servo teleop ready.")
    print("Controls (numbers = +, letters = -):")
    print("  Servo 1:  1 / q")
    print("  Servo 2:  2 / w")
    print("  Servo 3:  3 / e")
    print("  Servo 4:  4 / r")
    print("  Servo 5:  5 / t")
    print("  Servo 6:  6 / y")
    print("Other:")
    print("  s   : stop (torque off) ALL servos")
    print("  ESC : quit\n")

    # Mapping keys → (servo_id, delta_angle)
    key_map = {
        "1": (1, +STEP_DEG), "q": (1, -STEP_DEG),
        "2": (2, +STEP_DEG), "w": (2, -STEP_DEG),
        "3": (3, +STEP_DEG), "e": (3, -STEP_DEG),
        "4": (4, +STEP_DEG), "r": (4, -STEP_DEG),
        "5": (5, +STEP_DEG), "t": (5, -STEP_DEG),
        "6": (6, +STEP_DEG), "y": (6, -STEP_DEG),

        # Optional: bigger steps (remove if you don't want these)
        "z": (1, -BIG_STEP_DEG), "x": (1, +BIG_STEP_DEG),
    }

    try:
        while True:
            key = get_key()
            if key is None:
                continue

            # ESC
            if key == "\x1b":
                print("ESC pressed, quitting...")
                break

            # Stop all servos
            if key == "s":
                print("Stopping ALL servos (torque off)")
                for sid in SERVOS:
                    try:
                        servo.StopServo(sid)
                    except Exception:
                        pass
                continue

            if key not in key_map:
                continue

            sid, delta = key_map[key]

            prev_angle = angle_map.get(sid, 0.0)
            new_angle = clamp_angle_for_servo(sid, prev_angle + delta)

            # If clamped and no change, ignore further movement
            if new_angle == prev_angle:
                min_a, max_a = SERVO_LIMITS.get(sid, (DEFAULT_MIN_ANGLE, DEFAULT_MAX_ANGLE))
                print(f"[Servo {sid}] Limit reached ({min_a:.2f}°–{max_a:.2f}°). Current: {new_angle:.2f}°. Ignoring.")
                continue

            angle_map[sid] = new_angle
            ticks = deg_to_ticks(new_angle)

            print(f"[Servo {sid}] Target: {new_angle:6.2f}°  ({ticks} ticks)")
            servo.MoveTo(sid, ticks)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nCtrl+C detected, exiting...")

    # Cleanup
    try:
        for sid in SERVOS:
            try:
                servo.StopServo(sid)
            except Exception:
                pass
    except Exception:
        pass

    try:
        servo.portHandler.closePort()
    except Exception:
        pass

    print("Done.")


if __name__ == "__main__":
    main()
