from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


parameters = [
    {'name': 'robot_description',      'description': 'Path to the URDF/xacro file',                  'default': ''},
    {'name': 'controllers_file',       'description': 'Path to the ros2_control configuration file',  'default': ''},
]

def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])


def launch(context, *args, **kwargs):
    robot_description = ParameterValue(LaunchConfiguration('robot_description'))

    controllers_file = LaunchConfiguration('controllers_file')

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{'robot_description': robot_description}, controllers_file],
            output="both",
        )
    ]

    # Load the controllers YAML file
    with open(controllers_file.perform(context), 'r') as f:
        controllers = yaml.safe_load(f)

#        # Extract controller names in the order they appear in the yaml file
#        controllers_in_order = list(controllers['controller_manager']['ros__parameters'].keys())

#        # Load each controller sequentially
#        for i, controller_name in enumerate(controllers_in_order):
#          val = controllers['controller_manager']['ros__parameters'][controller_name]
#          if type(val) is dict and type(val['type']) is str:
#            controller_loader = Node(
#                   package="controller_manager",
#                   executable='spawner' if os.environ['ROS_DISTRO'] > 'foxy' else 'spawner.py',
#                   arguments=[controller_name, "-c", "/controller_manager"],
#               )

#            # Add a TimerAction to ensure controllers load one after the other
#            timer = TimerAction(
#                period=float(i),  # This can be adjusted for timing if necessary
#                actions=[controller_loader]
#            )
#            nodes.append(timer)
#        return nodes
    return nodes
