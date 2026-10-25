"""
Publishes the arm's joint angles as /joint_states so RViz shows the arm.

Real arm:  reads the angles the ESP32 prints over USB serial.
Demo mode: no hardware. The arm holds still at the home pose and moves only
           when you press keys (same end-effector control as the firmware).

Servo angle 90 deg is treated as 0 rad (straight up).

Usage:
    python arm_serial_bridge.py --port COM3        # real arm
    python arm_serial_bridge.py --demo             # no hardware, keyboard control

Demo keys (click this window first):
    W / S   forward / back        A / D   left / right
    R / F   up / down             H       home          Q   quit
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import msvcrt  # Windows console keyboard
except ImportError:
    msvcrt = None

JOINTS = ["base_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]

# Arm dimensions in metres: keep identical to arm_joystick.ino and arm.urdf
Z_SHOULDER, L1, L2, L3 = 0.08, 0.16, 0.14, 0.06
MIN_ANGLE, MAX_ANGLE = 10.0, 170.0            # servo limits, degrees
HOME = (0.22, 0.00, 0.12)                     # x, y, z in metres
X_RANGE, Y_RANGE, Z_RANGE = (0.05, 0.32), (-0.25, 0.25), (0.03, 0.20)
KEY_STEP = 0.005                              # metres per key press


def solve_ik(x, y, z):
    """Same maths as the firmware. Returns [base, shoulder, elbow, wrist] as
    '90 + degrees', or None if the target is out of reach."""
    if x < 0.02:
        return None
    yaw = math.atan2(y, x)
    r = math.hypot(x, y)
    wrist_lim = math.radians((MAX_ANGLE - MIN_ANGLE) / 2.0)
    theta = math.pi                            # tool pointing straight down
    q1 = q2 = q3 = 0.0
    for _ in range(10):
        wr = r - L3 * math.sin(theta)
        wz = z - L3 * math.cos(theta) - Z_SHOULDER
        c = (wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if c < -1.0 or c > 1.0:
            return None
        q2 = math.acos(c)
        q1 = math.atan2(wr, wz) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        q3 = max(-wrist_lim, min(wrist_lim, theta - q1 - q2))
        theta = q1 + q2 + q3
    out = [90.0 + math.degrees(v) for v in (yaw, q1, q2, q3)]
    if any(a < MIN_ANGLE or a > MAX_ANGLE for a in out[:3]):
        return None
    return out


class ArmBridge(Node):
    def __init__(self, port, baud, demo):
        super().__init__("arm_serial_bridge")
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.demo = demo
        self.ser = None
        self.target = list(HOME)
        self.angles = solve_ik(*HOME) or [90.0, 90.0, 90.0, 90.0]

        if not demo:
            import serial  # pip install pyserial
            self.ser = serial.Serial(port, baud, timeout=0)
            self.buf = b""
            self.get_logger().info(f"Reading {port} at {baud} baud")
        else:
            if msvcrt is None:
                self.get_logger().warn("Keyboard control needs Windows; arm will hold still.")
            self.get_logger().info(
                "Demo mode: arm holds still. Keys: W/S forward/back, A/D left/right, "
                "R/F up/down, H home, Q quit (click this window first)")

        self.create_timer(0.02, self.tick)

    def read_serial(self):
        self.buf += self.ser.read(1024)
        *lines, self.buf = self.buf.split(b"\n")
        for line in lines:  # keep the newest valid line
            parts = line.decode(errors="ignore").strip().split(",")
            if len(parts) == 5 and parts[0] == "A":
                try:
                    self.angles = [float(p) for p in parts[1:]]
                except ValueError:
                    pass

    def read_keys(self):
        if msvcrt is None:
            return
        dx = dy = dz = 0.0
        while msvcrt.kbhit():
            k = msvcrt.getwch().lower()
            if k == "w": dx += KEY_STEP
            elif k == "s": dx -= KEY_STEP
            elif k == "a": dy += KEY_STEP      # left = +Y
            elif k == "d": dy -= KEY_STEP
            elif k == "r": dz += KEY_STEP
            elif k == "f": dz -= KEY_STEP
            elif k == "h": self.target = list(HOME); dx = dy = dz = 0.0
            elif k == "q": raise KeyboardInterrupt
        nx = min(max(self.target[0] + dx, X_RANGE[0]), X_RANGE[1])
        ny = min(max(self.target[1] + dy, Y_RANGE[0]), Y_RANGE[1])
        nz = min(max(self.target[2] + dz, Z_RANGE[0]), Z_RANGE[1])
        sol = solve_ik(nx, ny, nz)
        if sol is not None:
            self.target = [nx, ny, nz]
            self.angles = sol
        elif (dx, dy, dz) != (0.0, 0.0, 0.0):
            self.get_logger().info("Out of reach, arm stays put")

    def tick(self):
        if self.demo:
            self.read_keys()
        else:
            self.read_serial()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [math.radians(a - 90.0) for a in self.angles]
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = ArmBridge(args.port, args.baud, args.demo)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
