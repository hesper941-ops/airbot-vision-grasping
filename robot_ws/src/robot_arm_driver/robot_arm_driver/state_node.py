import rclpy
from rclpy.node import Node

from robot_arm_interface.airbot_wrapper import AirbotWrapper
from robot_msgs.msg import ArmJointState


class ArmStateNode(Node):
    def __init__(self):
        super().__init__('arm_state_node')

        self.arm = AirbotWrapper(url="localhost", port=50001)
        self.arm.connect()

        self.state_pub = self.create_publisher(ArmJointState, '/robot_arm/joint_state', 10)
        self.timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info('ArmStateNode started.')

    def publish_state(self):
        try:
            msg = ArmJointState()
            msg.state = str(self.arm.get_state())
            msg.joint_pos = list(self.arm.get_joint_pos())
            msg.joint_vel = list(self.arm.get_joint_vel())

            end_pose = self.arm.get_end_pose()
            msg.end_pose = list(end_pose[0]) + list(end_pose[1])

            self.state_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Failed to publish arm state: {e}')

    def destroy_node(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
