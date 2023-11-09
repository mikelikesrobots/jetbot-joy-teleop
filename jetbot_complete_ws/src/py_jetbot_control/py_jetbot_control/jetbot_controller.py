from rclpy.node import Node
from geometry_msgs.msg import Twist
try:
    from py_jetbot_control.motor_controller import MotorController
except ImportError as e:
    print("Using fake controller! Import error: ", e)
    from py_jetbot_control.fake_controller import MotorController


class JetbotController(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.on_cmd_vel_msg, 10
        )
        self._controller = MotorController()
        self.get_logger().info("JetbotController started")

    def on_cmd_vel_msg(self, msg: Twist):
        self.get_logger().info(f"Got msg: {msg}")

        # Linear x gives forward and backward
        linear_left = linear_right = msg.linear.x / 2
        # Angular z is -1.0 for a right turn, 1.0 for a left turn
        angular_right = msg.angular.z / 4
        angular_left = -angular_right

        left = linear_left + angular_left
        right = linear_right + angular_right
        self.get_logger().info(f"left: {left}, right: {right}")

        # Send speeds to motors
        self._controller.set_motors(left, right)
