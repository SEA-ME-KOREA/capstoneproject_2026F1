#!/usr/bin/env python3

import threading

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
from rclpy.node import Node


FORWARD_SPEED_MPS = 0.1
BACKWARD_SPEED_MPS = -0.1
TURN_ANGULAR_RADPS = 0.5
TURN_ASSIST_LINEAR_MPS = 0.05
PUBLISH_PERIOD_S = 0.05

CMD_VEL_TOPIC = "/limo2/cmd_vel"

ARROW_KEYS = (
    keyboard.Key.up,
    keyboard.Key.down,
    keyboard.Key.left,
    keyboard.Key.right,
)


class Limo2ArrowTeleop(Node):
    def __init__(self) -> None:
        super().__init__("limo2_arrow_teleop")
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.lock = threading.Lock()
        self.held: set = set()
        self.create_timer(PUBLISH_PERIOD_S, self.publish_loop)

        self.get_logger().info(
            f"limo2_arrow_teleop ready -> publishing Twist on {CMD_VEL_TOPIC}"
        )
        self.get_logger().info(
            f"  Up: forward {FORWARD_SPEED_MPS:+.2f} m/s   "
            f"Down: backward {BACKWARD_SPEED_MPS:+.2f} m/s"
        )
        self.get_logger().info(
            f"  Left/Right: angular {TURN_ANGULAR_RADPS:+.2f} rad/s "
            f"with linear assist {TURN_ASSIST_LINEAR_MPS:+.2f} m/s"
        )
        self.get_logger().info(
            "  Release a key -> immediate stop signal.  ESC to quit."
        )

    def compute_twist(self) -> Twist:
        with self.lock:
            up = keyboard.Key.up in self.held
            down = keyboard.Key.down in self.held
            left = keyboard.Key.left in self.held
            right = keyboard.Key.right in self.held

        linear = 0.0
        angular = 0.0
        if up:
            linear += FORWARD_SPEED_MPS
        if down:
            linear += BACKWARD_SPEED_MPS
        if left:
            angular += TURN_ANGULAR_RADPS
            linear += TURN_ASSIST_LINEAR_MPS
        if right:
            angular -= TURN_ANGULAR_RADPS
            linear += TURN_ASSIST_LINEAR_MPS

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        return msg

    def publish_loop(self) -> None:
        self.cmd_pub.publish(self.compute_twist())

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def on_press(self, key) -> None:
        if key in ARROW_KEYS:
            with self.lock:
                self.held.add(key)

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.publish_zero()
            self.get_logger().info("ESC pressed; shutting down.")
            if rclpy.ok():
                rclpy.shutdown()
            return False

        if key in ARROW_KEYS:
            with self.lock:
                self.held.discard(key)
                still_held = bool(self.held)
            if still_held:
                self.cmd_pub.publish(self.compute_twist())
            else:
                self.publish_zero()
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Limo2ArrowTeleop()
    listener = keyboard.Listener(
        on_press=node.on_press,
        on_release=node.on_release,
    )
    listener.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero()
        except Exception:
            pass
        listener.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
