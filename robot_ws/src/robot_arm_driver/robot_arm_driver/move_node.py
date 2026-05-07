import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from robot_arm_interface.airbot_wrapper import AirbotWrapper


class ArmMoveNode(Node):
    def __init__(self):
        super().__init__('arm_move_node')

        self.arm = AirbotWrapper(url="localhost", port=50001)
        self.arm.connect()

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/robot_arm/target_joint',
            self.target_callback,
            10
        )

        self.get_logger().info('ArmMoveNode started, waiting for /robot_arm/target_joint...')

    def target_callback(self, msg):
        target = list(msg.data)

        if len(target) != 6:
            self.get_logger().error(f'Invalid joint target length: {len(target)} (expected 6)')
            return

        self.get_logger().info(f'Received joint target: {target}')

        try:
            self.arm.move_joints(target)
            self.get_logger().info('Move command executed.')
        except Exception as e:
            self.get_logger().error(f'Failed to move joints: {e}')

    def destroy_node(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
