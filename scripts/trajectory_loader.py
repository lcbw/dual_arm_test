#!/usr/bin/env python3

#Basically SNP example
#but calls the action as well

#load file
#start action ros node

#rclpy node, create action client and call action (controller will expose the action)

from builtin_interfaces.msg import Duration, Time
import rclpy
import rospkg
import os
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import yaml
from ament_index_python.packages import get_package_share_directory

class MotionPlanFromFileNode(Node):
    def __init__(self):
        super().__init__('motion_plan_from_file')
        self.declare_parameter("trajectory_file_path","config/naive_trajectory_single_ur_horizontal.yaml")
        self.declare_parameter("package_param","dual_arm_test")

        file_path = os.path.join(get_package_share_directory(self.get_parameter("package_param").value), self.get_parameter("trajectory_file_path").value)
        self.joints = self.open_file(file_path)

        self.declare_parameter("controller_name", "/joint_trajectory_controller")
        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.parse_trajectory()
        self.i = 0
        self.run_cycles = 100
        self._send_goal_future = None
        self._get_result_future = None
        self.execute_next_trajectory()
        #self.execute_trajectory()

    def create_point(self, point_val: dict,
                 first_point: dict) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = point_val['positions']
        point.velocities = point_val['velocities']
        point.accelerations = point_val['accelerations']

        sec = int(point_val['time_from_start']['sec']) - int(first_point['time_from_start']['sec'])
        nsec = int(point_val['time_from_start']['nanosec']) - int(first_point['time_from_start']['nanosec'])

        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        return point

    def open_file(self, file_path):
        try:
            with open(file_path, "r") as file:
                scan_traj = yaml.safe_load(file)

            joints = scan_traj['joint_names']

            points = scan_traj['points']
            trajectory= []

            for i in range(len(points)):
                trajectory.append(self.create_point(points[i],points[0]))

            self.trajectory = trajectory
            return joints

        except Exception as e:

          print(f"Error finding file in package: {e}")
          return None


    def parse_trajectory(self):

        goal = JointTrajectory()
        goal.joint_names = self.joints
        for pt in self.trajectory:
            goal.points.append(pt)

        self.goal = goal

    # def execute_next_trajectory(self):
    #     if self.i >= len(self.goals):
    #         self.get_logger().info("Done with all trajectories")
    #         return
    #     traj_name = list(self.goals)[self.i]
    #     self.i = self.i + 1
    #     if traj_name:
    #         self.execute_trajectory(traj_name)


    def execute_next_trajectory(self):
        if self.i >= self.run_cycles:
            self.get_logger().info("Done with all trajectories")
            return

        self.i = self.i + 1
        self.execute_trajectory()

    def execute_trajectory(self):#, traj_name):
        self.get_logger().info(f"Executing trajectory") #{traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goal
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
        self.get_logger().info(f"Done with result: {self.error_code_to_str(result.error_code)}")
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            time.sleep(2)
            self.execute_next_trajectory()

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
    motion_plan_from_file = MotionPlanFromFileNode()

    rclpy.spin(motion_plan_from_file)

    motion_plan_from_file.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

