import queue
import threading

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from robot_arm_interface.airbot_wrapper import AirbotWrapper
from robot_msgs.msg import ArmJointState


class ArmExecutorNode(Node):
    """Single hardware owner for AIRBOT arm execution and state publishing.

    All real robot commands should pass through this node. Keeping one SDK
    connection avoids command races between separate ROS nodes.
    """

    def __init__(self):
        super().__init__('arm_executor_node')

        self.arm = AirbotWrapper(url='localhost', port=50001)
        self.arm.connect(speed_profile='slow')

        self.sdk_lock = threading.Lock()
        self.last_state_msg = None

        # Keep command execution off the ROS callback thread so state publishing
        # can stay responsive while a motion command is running inside the SDK.
        self.command_queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self._command_worker, daemon=True)
        self.worker.start()

        self.state_pub = self.create_publisher(ArmJointState, '/robot_arm/joint_state', 10)
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

        self.get_logger().info('ArmExecutorNode started as the single hardware owner.')
        self.get_logger().info('Publishing: /robot_arm/joint_state')
        self.get_logger().info('Listening: /robot_arm/target_joint, /robot_arm/cart_target, /robot_arm/gripper_cmd')

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

    def publish_state(self):
        """Publish the latest arm state for task-level feedback loops."""
        if not self.sdk_lock.acquire(blocking=False):
            if self.last_state_msg is not None:
                busy_msg = self._copy_state_msg(self.last_state_msg)
                busy_msg.state = 'BUSY'
                self.state_pub.publish(busy_msg)
            return

        try:
            msg = ArmJointState()
            msg.state = str(self.arm.get_state())
            msg.joint_pos = list(self.arm.get_joint_pos())
            msg.joint_vel = list(self.arm.get_joint_vel())

            end_pose = self.arm.get_end_pose()
            msg.end_pose = list(end_pose[0]) + list(end_pose[1])

            self.last_state_msg = self._copy_state_msg(msg)
            self.state_pub.publish(msg)
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
