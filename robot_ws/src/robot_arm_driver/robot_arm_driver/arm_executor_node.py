import math
import queue
import threading

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from robot_arm_interface.airbot_wrapper import AirbotWrapper
from robot_msgs.msg import ArmJointState


class ArmExecutorNode(Node):
    """唯一硬件 owner，启动时自动走到初始工作位姿，然后进入命令监听循环。

    All real robot commands should pass through this node. Keeping one SDK
    connection avoids command races between separate ROS nodes.
    """

    def __init__(self):
        super().__init__('arm_executor_node')

        # ---- 初始化参数 ----
        self.declare_parameter('do_init', True)
        self.declare_parameter('init_joint_pos_deg', [0.0, -45.0, 120.0, -90.0, 90.0, 90.0])

        # ---- 连接机械臂（慢速启动，初始化后再切快速） ----
        self.arm = AirbotWrapper(url='localhost', port=50001)
        self.arm.connect(speed_profile='slow')

        # 启动时自动走到初始工作位姿（慢速，可配置关闭）
        if self.get_parameter('do_init').value:
            self._move_to_init_pose()

        # 初始位姿完成后切换到快速，供后续抓取动作使用
        self.arm.set_speed_profile('default')
        self.get_logger().info('初始化完成，已切换到默认速度')

        self.sdk_lock = threading.Lock()
        self.last_state_msg = None
        self.last_pose_msg = None

        # Keep command execution off the ROS callback thread so state publishing
        # can stay responsive while a motion command is running inside the SDK.
        self.command_queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._command_worker, daemon=True)
        self.worker.start()

        # 两套状态 topic：
        # - ArmJointState：给任务节点（grasp_task_*）做到位判断
        # - PoseStamped：给 camera_to_base_transform 做手眼矩阵运算（需要四元数）
        self.state_pub = self.create_publisher(ArmJointState, '/robot_arm/joint_state', 10)
        self.end_pose_pub = self.create_publisher(PoseStamped, '/robot_arm/end_pose', 10)
        self.state_timer = self.create_timer(0.1, self.publish_state)

        self.joint_sub = self.create_subscription(
            Float64MultiArray,
            '/robot_arm/target_joint',
            self.joint_target_callback,
            10
        )
        self.cart_sub = self.create_subscription(
            PointStamped,
            '/robot_arm/cart_target',
            self.cart_target_callback,
            10
        )
        self.gripper_sub = self.create_subscription(
            String,
            '/robot_arm/gripper_cmd',
            self.gripper_callback,
            10
        )
        self.speed_sub = self.create_subscription(
            String,
            '/robot_arm/speed_profile',
            self.speed_callback,
            10
        )

        self.get_logger().info('ArmExecutorNode 启动完毕（唯一硬件 owner）。')
        self.get_logger().info('发布: /robot_arm/joint_state (ArmJointState) + /robot_arm/end_pose (PoseStamped)')
        self.get_logger().info('监听: /robot_arm/target_joint, /robot_arm/cart_target, /robot_arm/gripper_cmd, /robot_arm/speed_profile')

    def _deg2rad(self, deg: float) -> float:
        return deg * math.pi / 180.0

    def _move_to_init_pose(self):
        """启动时同步执行：走到初始工作位姿。

        这个操作在 worker 线程启动前同步完成，确保后续命令是在
        机械臂已就位的基础上执行。
        """
        deg = self.get_parameter('init_joint_pos_deg').value
        rad = [self._deg2rad(v) for v in deg]
        self.get_logger().info(
            f'机械臂初始化: 目标关节角(deg)={deg}')
        try:
            self.arm.get_state()  # 确认连接正常
            self.arm.move_joints(rad)
            self.get_logger().info('机械臂已到达初始工作位姿')
        except Exception as e:
            self.get_logger().error(f'初始化失败: {e}，继续启动...')

    def joint_target_callback(self, msg):
        """Validate and enqueue a joint-space command."""
        target = list(msg.data)
        if len(target) != 6:
            self.get_logger().error(f'Invalid joint target length: {len(target)} (expected 6)')
            return
        self._enqueue_command('joint', target)

    def cart_target_callback(self, msg: PointStamped):
        """Validate and enqueue a single Cartesian step command."""
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != 'base_link':
            self.get_logger().error(f'Invalid frame_id: {frame_id}, expected base_link')
            return

        target = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        self._enqueue_command('cart', target)

    def gripper_callback(self, msg):
        """Validate and enqueue a gripper command."""
        command = msg.data.strip().lower()
        if command not in ['open', 'close']:
            self.get_logger().error(f'Unknown gripper command: {command}')
            return
        self._enqueue_command('gripper', command)

    def speed_callback(self, msg):
        """运行时切换速度档（slow / default / fast）。"""
        profile = msg.data.strip().lower()
        if profile not in ('slow', 'default', 'fast'):
            self.get_logger().error(f'未知速度档: {profile}')
            return
        with self.sdk_lock:
            self.arm.set_speed_profile(profile)
        self.get_logger().info(f'速度档切换为: {profile}')

    def publish_state(self):
        """发布 ArmJointState + PoseStamped，给任务层和坐标变换用。"""
        if not self.sdk_lock.acquire(blocking=False):
            # SDK 正忙（在执行命令），发缓存副本
            if self.last_state_msg is not None:
                busy_msg = self._copy_state_msg(self.last_state_msg)
                busy_msg.state = 'BUSY'
                self.state_pub.publish(busy_msg)
            if self.last_pose_msg is not None:
                self.end_pose_pub.publish(self.last_pose_msg)
            return

        try:
            # ---- ArmJointState ----
            msg = ArmJointState()
            msg.state = str(self.arm.get_state())
            msg.joint_pos = list(self.arm.get_joint_pos())
            msg.joint_vel = list(self.arm.get_joint_vel())

            end_pose = self.arm.get_end_pose()
            position = list(end_pose[0])
            quaternion = list(end_pose[1])
            msg.end_pose = position + quaternion

            self.last_state_msg = self._copy_state_msg(msg)
            self.state_pub.publish(msg)

            # ---- PoseStamped（给 camera_to_base_transform） ----
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])
            pose_msg.pose.orientation.x = float(quaternion[0])
            pose_msg.pose.orientation.y = float(quaternion[1])
            pose_msg.pose.orientation.z = float(quaternion[2])
            pose_msg.pose.orientation.w = float(quaternion[3])
            self.last_pose_msg = pose_msg
            self.end_pose_pub.publish(pose_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish arm state: {e}')
        finally:
            self.sdk_lock.release()

    def _enqueue_command(self, command_type, payload):
        """Accept a one-command buffer and reject bursts while the arm catches up."""
        try:
            self.command_queue.put_nowait((command_type, payload))
            self.get_logger().info(f'Queued {command_type} command: {payload}')
        except queue.Full:
            self.get_logger().warning(
                f'Arm is busy, rejecting {command_type} command: {payload}'
            )

    def _command_worker(self):
        """Run blocking SDK commands sequentially in one background worker."""
        while not self.stop_event.is_set():
            try:
                command_type, payload = self.command_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                with self.sdk_lock:
                    if command_type == 'joint':
                        self.arm.move_joints(payload)
                        self.get_logger().info('Joint command executed.')
                    elif command_type == 'cart':
                        self.arm.move_to_cart_target_with_current_orientation(payload)
                        self.get_logger().info('Cartesian command executed.')
                    elif command_type == 'gripper':
                        if payload == 'open':
                            self.arm.open_gripper()
                        else:
                            self.arm.close_gripper()
                        self.get_logger().info(f'Gripper command executed: {payload}')
                    else:
                        self.get_logger().error(f'Unsupported command type: {command_type}')
            except Exception as e:
                self.get_logger().error(f'Failed to execute {command_type} command: {e}')
            finally:
                self.command_queue.task_done()

    def _copy_state_msg(self, msg):
        """Copy state messages so cached BUSY publishes cannot mutate old data."""
        copied = ArmJointState()
        copied.state = msg.state
        copied.joint_pos = list(msg.joint_pos)
        copied.joint_vel = list(msg.joint_vel)
        copied.end_pose = list(msg.end_pose)
        return copied

    def destroy_node(self):
        """Stop the worker and disconnect the SDK cleanly on shutdown."""
        self.stop_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
        if self.sdk_lock.acquire(timeout=1.0):
            try:
                self.arm.disconnect()
            except Exception:
                pass
            finally:
                self.sdk_lock.release()
        else:
            self.get_logger().warning('SDK command still running, skip disconnect on shutdown.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
