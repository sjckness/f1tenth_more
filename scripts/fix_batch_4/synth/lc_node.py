"""Minimal rclpy lifecycle node: logs CONFIGURED / ACTIVATED (Humble and Jazzy rclpy)."""
import rclpy
from rclpy.lifecycle import Node, TransitionCallbackReturn


class LC(Node):
    def __init__(self):
        super().__init__('syn_lifecycle')

    def on_configure(self, state):
        self.get_logger().info('SYN CONFIGURED')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('SYN ACTIVATED')
        return TransitionCallbackReturn.SUCCESS


rclpy.init()
n = LC()
try:
    rclpy.spin(n)
except BaseException:
    pass
