"""ROS2 adapter for deterministic semantic navigation."""

import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from semantic_nav_orchestrator.core import (
    ApproachPosePlanner,
    GridMap,
    Pose2D,
    SemanticResolver,
)


class SemanticNavNode(Node):
    """Translate deterministic text commands into safe Nav2 pose goals."""

    def __init__(self):
        super().__init__('semantic_nav')
        package_dir = Path(get_package_share_directory(
            'semantic_nav_orchestrator'))
        self.declare_parameter(
            'locations_file',
            str(package_dir / 'config' / 'named_locations.yaml'))
        self.declare_parameter(
            'objects_file',
            str(package_dir / 'config' / 'object_map.yaml'))
        self.declare_parameter('approach_distance', 0.8)
        self.declare_parameter('clearance', 0.25)
        self.declare_parameter('candidate_count', 16)
        self.resolver = SemanticResolver.from_files(
            self.get_parameter('locations_file').value,
            self.get_parameter('objects_file').value,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.path_client = ActionClient(
            self, ComputePathToPose, '/compute_path_to_pose')
        self.grid_map = None
        self.busy = False
        self.nav_goal_handle = None
        self.operation_id = 0
        self.path_candidates = []
        self.robot_pose = None
        self.path_target = ''
        self.status_publisher = self.create_publisher(
            String, '/semantic_nav/status', 10)
        self.create_subscription(
            String, '/semantic_nav/command', self.command_callback, 10)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.publish_status('ready')

    def publish_status(self, status: str):
        self.get_logger().info(status)
        self.status_publisher.publish(String(data=status))

    def command_callback(self, message: String):
        command = message.data.strip()
        if command in ('取消', 'cancel', '停止'):
            self.cancel_navigation()
            return
        if self.busy:
            self.publish_status('rejected: navigation task already active')
            return
        resolution = self.resolver.resolve(command)
        if not resolution.accepted:
            self.publish_status(f'rejected: {resolution.reason}')
            return
        if resolution.kind == 'location':
            self.operation_id += 1
            self.send_navigation_goal(resolution.pose, 'named location')
        else:
            self.plan_object_approach(resolution.semantic_object)

    def map_callback(self, message: OccupancyGrid):
        self.grid_map = GridMap(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            data=tuple(message.data),
        )

    def plan_object_approach(self, semantic_object):
        if self.grid_map is None:
            self.publish_status('rejected: map is not available')
            return
        self.robot_pose = self.current_robot_pose()
        if self.robot_pose is None:
            return
        planner = ApproachPosePlanner(
            self.grid_map,
            approach_distance=float(
                self.get_parameter('approach_distance').value),
            clearance=float(self.get_parameter('clearance').value),
            candidate_count=int(self.get_parameter('candidate_count').value),
        )
        self.path_candidates = planner.candidates(
            semantic_object, self.robot_pose)
        if not self.path_candidates:
            self.publish_status('rejected: no clear approach pose')
            return
        if not self.path_client.server_is_ready():
            self.publish_status('rejected: Nav2 path planner is not available')
            return
        self.busy = True
        self.operation_id += 1
        self.path_target = f'object {semantic_object.object_id}'
        self.check_next_path(self.operation_id)

    def check_next_path(self, operation_id):
        if operation_id != self.operation_id:
            return
        if not self.path_candidates:
            self.busy = False
            self.publish_status('rejected: no safe approach pose')
            return
        candidate = self.path_candidates.pop(0)
        goal = ComputePathToPose.Goal()
        goal.start = self.to_pose_stamped(self.robot_pose)
        goal.goal = self.to_pose_stamped(candidate)
        goal.use_start = True
        future = self.path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response, pose=candidate, task_id=operation_id:
            self.path_goal_response(response, pose, task_id))

    def path_goal_response(self, future, candidate, operation_id):
        if operation_id != self.operation_id:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.check_next_path(operation_id)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, pose=candidate, task_id=operation_id:
            self.path_result(result, pose, task_id))

    def path_result(self, future, candidate, operation_id):
        if operation_id != self.operation_id:
            return
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            self.busy = False
            self.send_navigation_goal(candidate, self.path_target)
        else:
            self.check_next_path(operation_id)

    def send_navigation_goal(self, pose: Pose2D, target: str):
        if not self.nav_client.server_is_ready():
            self.busy = False
            self.publish_status('rejected: Nav2 navigator is not available')
            return
        self.busy = True
        goal = NavigateToPose.Goal()
        goal.pose = self.to_pose_stamped(pose)
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response, task_id=self.operation_id:
            self.navigation_goal_response(response, pose, target, task_id))

    def navigation_goal_response(self, future, pose: Pose2D, target: str,
                                 operation_id: int):
        goal_handle = future.result()
        if operation_id != self.operation_id:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        self.nav_goal_handle = goal_handle
        if not goal_handle.accepted:
            self.busy = False
            self.publish_status('rejected: Nav2 rejected navigation goal')
            return
        self.publish_status(
            f'accepted: {target} at ({pose.x:.2f}, {pose.y:.2f}, '
            f'{pose.yaw:.2f})')
        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, task_id=operation_id:
            self.navigation_result(result, task_id))

    def navigation_result(self, future, operation_id: int):
        if operation_id != self.operation_id:
            return
        status = future.result().status
        self.busy = False
        self.nav_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status('succeeded')
        elif status == GoalStatus.STATUS_CANCELED:
            self.publish_status('canceled')
        else:
            self.publish_status('failed')

    def cancel_navigation(self):
        if not self.busy:
            self.publish_status('rejected: no active navigation task')
            return
        self.operation_id += 1
        self.path_candidates = []
        self.busy = False
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None
            self.publish_status('cancel_requested')
        else:
            self.publish_status('canceled')

    def current_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
        except TransformException as error:
            self.publish_status(f'rejected: robot pose unavailable: {error}')
            return None
        translation = transform.transform.translation
        return Pose2D(translation.x, translation.y)

    def to_pose_stamped(self, pose: Pose2D):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.orientation.w = math.cos(pose.yaw / 2.0)
        return message


def main():
    rclpy.init()
    node = SemanticNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
