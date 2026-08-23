"""Bridge between ROS 2 and the KUKA controller's EKI interface.

Provides the ``kuka_joint`` service (move to a joint target) and continuously
publishes ``/joint_states`` from the controller's reported axis values.

Units: EKI speaks degrees, ROS speaks radians. This node is the boundary.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from kuka_control_box_srvs.srv import KukaJoint
from kuka_eki.eki import EkiMotionClient, EkiStateClient
from kuka_eki.krl import Axis

DEFAULT_JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
    "end_effector_joint",
]


class KukaEkiJointServer(Node):
    def __init__(self):
        super().__init__('kuka_eki_joint_server')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('robot_ip', '172.31.1.147')
        self.declare_parameter('motion_port', 54600)
        self.declare_parameter('state_port', 54602)
        self.declare_parameter('state_poll_period', 0.02)
        self.declare_parameter('joint_state_publish_period', 0.1)
        self.declare_parameter('motion_check_period', 0.1)
        self.declare_parameter('motion_tolerance_deg', 0.1)
        self.declare_parameter('max_velocity_scaling', 0.5)
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)

        robot_ip = self.get_parameter('robot_ip').value
        motion_port = self.get_parameter('motion_port').value
        state_port = self.get_parameter('state_port').value
        self.motion_tolerance_deg = float(self.get_parameter('motion_tolerance_deg').value)
        self.max_velocity_scaling = float(self.get_parameter('max_velocity_scaling').value)
        self.joint_names = list(self.get_parameter('joint_names').value)

        # ------------------------------------------------------------------
        # State (instance attributes, not module globals)
        # ------------------------------------------------------------------
        self.joint_positions_deg = [0.0] * len(self.joint_names)
        self.target_axis_deg = None

        # ------------------------------------------------------------------
        # Controller connection
        # ------------------------------------------------------------------
        self.get_logger().info(
            f"Connecting to KUKA controller at {robot_ip} "
            f"(motion:{motion_port}, state:{state_port})"
        )
        self.eki_motion_client = EkiMotionClient(robot_ip, motion_port)
        self.eki_state_client = EkiStateClient(robot_ip, state_port)
        try:
            self.eki_motion_client.connect()
            self.eki_state_client.connect()
        except OSError as exc:
            self.get_logger().error(
                f"Could not reach the controller at {robot_ip}: {exc}. "
                f"Check 'robot_ip' in your params file and that the EKI "
                f"program is running on the controller."
            )
            raise RuntimeError(f"KUKA controller connection failed: {exc}") from exc

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self.joint_service = self.create_service(
            KukaJoint, 'kuka_joint', self.handle_joint_service
        )
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.create_timer(
            float(self.get_parameter('state_poll_period').value), self.read_robot_state
        )
        self.create_timer(
            float(self.get_parameter('joint_state_publish_period').value),
            self.publish_joint_states,
        )
        self.create_timer(
            float(self.get_parameter('motion_check_period').value),
            self.check_motion_completion,
        )

        self.get_logger().info("kuka_joint service ready.")

    # ----------------------------------------------------------------------
    def read_robot_state(self):
        """Poll the controller and cache the current axis values [deg]."""
        try:
            state = self.eki_state_client.state()
        except Exception as exc:  # noqa: BLE001 - keep the timer alive
            self.get_logger().error(f"Failed to read controller state: {exc}")
            return

        if state is None or state.axis is None:
            return

        axes = [
            float(state.axis.a1), float(state.axis.a2), float(state.axis.a3),
            float(state.axis.a4), float(state.axis.a5), float(state.axis.a6),
        ]
        # An all-zero reading means the frame was empty or malformed, not that
        # the robot is actually at the zero pose.
        if np.linalg.norm(axes) <= 0.001:
            return

        self.joint_positions_deg[:6] = axes
        if len(self.joint_positions_deg) > 6:
            self.joint_positions_deg[6] = 0.0  # tool joint is fixed

    # ----------------------------------------------------------------------
    def handle_joint_service(self, request, response):
        """Send a joint-space PTP target to the controller."""
        try:
            target = Axis(request.a1, request.a2, request.a3,
                          request.a4, request.a5, request.a6)
            self.eki_motion_client.ptp(target, self.max_velocity_scaling)
            self.target_axis_deg = np.array(
                [request.a1, request.a2, request.a3,
                 request.a4, request.a5, request.a6], dtype=float
            )
            response.success = True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"kuka_joint request failed: {exc}")
            response.success = False

        response.a1, response.a2, response.a3 = self.joint_positions_deg[:3]
        response.a4, response.a5, response.a6 = self.joint_positions_deg[3:6]
        return response

    # ----------------------------------------------------------------------
    def check_motion_completion(self):
        if self.target_axis_deg is None:
            return
        current = np.array(self.joint_positions_deg[:6], dtype=float)
        if np.linalg.norm(current - self.target_axis_deg) < self.motion_tolerance_deg:
            self.get_logger().info("Target joint configuration reached.")
            self.target_axis_deg = None

    # ----------------------------------------------------------------------
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [np.deg2rad(pos) for pos in self.joint_positions_deg]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KukaEkiJointServer()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if isinstance(exc, RuntimeError):
            print(f"[kuka_eki_joint_server] {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
