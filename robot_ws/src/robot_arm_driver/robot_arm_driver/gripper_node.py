import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robot_arm_interface.airbot_wrapper import AirbotWrapper


class ArmGripperNode(Node):
    def __init__(self):
        super().__init__('arm_gripper_node')

        self.arm = AirbotWrapper(url="localhost", port=50001)
        self.arm.connect()

        self.sub = self.create_subscription(
            String,
            '/robot_arm/gripper_cmd',
            self.cmd_callback,
            10
        )

        self.get_logger().info('ArmGripperNode started, waiting for /robot_arm/gripper_cmd...')

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received gripper command: {cmd}')

        try:
            if cmd == 'open':
                self.arm.open_gripper()
                self.get_logger().info('Gripper opened.')
            elif cmd == 'close':
                self.arm.close_gripper()
                self.get_logger().info('Gripper closed.')
            else:
                self.get_logger().error(f'Unknown gripper command: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Failed to execute gripper command: {e}')

    def destroy_node(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
