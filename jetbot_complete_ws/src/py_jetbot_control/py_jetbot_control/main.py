import rclpy

from py_jetbot_control.jetbot_controller import JetbotController


def main(args=None):
    rclpy.init(args=args)
    jetbot_controller = JetbotController()
    rclpy.spin(jetbot_controller)
    jetbot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
