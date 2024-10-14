#!/usr/bin/env python3
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import yaml
from std_srvs.srv import Trigger


def load_trajectory(file_path: str) -> JointTrajectory:
    # Load the trajectory file
    with open(file_path, "r") as file:
        scan_traj = yaml.safe_load(file)

    # Create the trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = scan_traj['joint_names']

    points = scan_traj['points']

    # Add the trajectory points
    for i in range(len(points)):
        trajectory.points.append(
            create_point(
                **points[i],
            )
        )

    return trajectory


def create_point(positions,
                 velocities,
                 accelerations,
                 effort,
                 time_from_start) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()

    point.positions = positions
    point.velocities = velocities
    point.accelerations = accelerations
    point.effort = effort

    tfs_sec = time_from_start['sec']
    tfs_nsec = time_from_start['nanosec']
    point.time_from_start = Duration(sec=tfs_sec, nanosec=tfs_nsec)

    return point


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('motion_plan_from_file')

        # Declare parameters
        self.declare_parameter('fjt_action', 'follow_joint_trajectory')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('n_cycles', 1)

        # Create ROS interfaces
        fjt_action = self.get_parameter('fjt_action').value
        self._action_client = ActionClient(self, FollowJointTrajectory, fjt_action)
        self._server = self.create_service(Trigger, 'execute_trajectory', self.trigger_execution)

        self._trajectory = None
        self._n_cycles = 1
        self._cycle_idx = 0
        self._send_goal_future = None
        self._get_result_future = None

    def trigger_execution(self,
                          _req: Trigger.Request,
                          res: Trigger.Response) -> Trigger.Response:
        try:
            # Load the trajectory
            trajectory_file = self.get_parameter('trajectory_file').value
            self._trajectory = load_trajectory(trajectory_file)

            # Reset the internal counters
            self._n_cycles = self.get_parameter('n_cycles').value
            self._cycle_idx = 0

            # Start the trajectory execution
            self.start_trajectory_execution()

            res.success = True
        except Exception as e:
            res.success = False
            res.message = str(e)

        return res

    def start_trajectory_execution(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._trajectory

        self.get_logger().info(f"Starting trajectory execution")
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        # Increment the cycle index counter
        self._cycle_idx += 1

        self.get_logger().info(f"Trajectory {self._cycle_idx} completed with result: {self.error_code_to_str(result.error_code)}")
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            if self._cycle_idx < self._n_cycles:
                self.start_trajectory_execution()

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutor()
    node.get_logger().info('Started trajectory executor...')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
