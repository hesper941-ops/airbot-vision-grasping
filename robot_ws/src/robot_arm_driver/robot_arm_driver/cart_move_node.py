import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from robot_arm_interface.airbot_wrapper import AirbotWrapper


class ArmCartesianMoveNode(Node):
    """Execution layer node for cartesian step commands.

    This node is the only executor for `/robot_arm/cart_target` messages.
    It validates the incoming frame and forwards the command to the hardware wrapper.
    """

    def __init__(self):
        super().__init__('arm_cartesian_move_node')

        self.arm = AirbotWrapper(url='localhost', port=50001)
        self.arm.connect()

        self.sub = self.create_subscription(
            PointStamped,
            '/robot_arm/cart_target',
            self.target_callback,
            10
        )

        self.get_logger().info('ArmCartesianMoveNode started, waiting for /robot_arm/cart_target...')

    def target_callback(self, msg: PointStamped):
        """Validate and execute a single cartesian step command."""
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != 'base_link':
            self.get_logger().error(f'Invalid frame_id: {frame_id}, expected base_link')
            return

        target = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        self.get_logger().info(
            f'Received cartesian step target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
        )

        try:
            # The wrapper performs the final software safety gate before SDK control.
            self.arm.move_to_cart_target_with_current_orientation(target)
            self.get_logger().info('Cartesian step command sent to arm.')
        except Exception as e:
            self.get_logger().error(f'Failed to execute cartesian target: {e}')

    def destroy_node(self):
        """Disconnect the wrapper cleanly on shutdown."""
        try:
            self.arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmCartesianMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
