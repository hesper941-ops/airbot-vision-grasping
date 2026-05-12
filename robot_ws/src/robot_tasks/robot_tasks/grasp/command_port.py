"""统一的机械臂命令发布端口。

所有发往机械臂执行器的命令都通过此端口。任务层各模块不直接持有 publisher。
"""

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray, String


class ArmCommandPort:
    """封装抓取任务使用的五个机械臂 ROS2 publisher。

    端口在 ROS Node 初始化时创建一次，然后传递给所有需要控制机械臂的组件
    （抓取序列、恢复、搜索等）。
    """

    def __init__(self, node, base_frame: str = "base_link"):
        self._node = node
        self._base_frame = base_frame

        self._joint_pub = node.create_publisher(
            Float64MultiArray, "/robot_arm/target_joint", 10)
        self._cart_pub = node.create_publisher(
            PointStamped, "/robot_arm/cart_target", 10)
        self._gripper_pub = node.create_publisher(
            String, "/robot_arm/gripper_cmd", 10)
        self._speed_pub = node.create_publisher(
            String, "/robot_arm/speed_profile", 10)
        self._reset_pub = node.create_publisher(
            String, "/robot_arm/reset_executor", 10)

    # -- Joint target --------------------------------------------------------

    def publish_joint_target(self, joint_pos, reason: str = ""):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_pos]
        self._joint_pub.publish(msg)
        if reason:
            self._node.get_logger().info(
                f"Published joint target, reason={reason}")

    # -- Cartesian target ----------------------------------------------------

    def publish_cart_target(self, xyz, reason: str = ""):
        msg = PointStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        self._cart_pub.publish(msg)
        if reason:
            self._node.get_logger().info(
                f"Published cart target, reason={reason}")

    # -- Gripper -------------------------------------------------------------

    def publish_gripper(self, command: str, reason: str = ""):
        msg = String()
        msg.data = command
        self._gripper_pub.publish(msg)
        if reason:
            self._node.get_logger().info(
                f"Published gripper command={command}, reason={reason}")

    # -- Speed profile -------------------------------------------------------

    def publish_speed_profile(self, profile: str, reason: str = ""):
        msg = String()
        msg.data = profile
        self._speed_pub.publish(msg)
        if reason:
            self._node.get_logger().info(
                f"Published speed_profile={profile}, reason={reason}")

    # -- Reset / clear-error -------------------------------------------------

    def publish_reset(self, command: str, reason: str = ""):
        msg = String()
        msg.data = command
        self._reset_pub.publish(msg)
        if reason:
            self._node.get_logger().warn(
                f"Published reset command={command}, reason={reason}")
