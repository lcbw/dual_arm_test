from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os
import time


def default_trajectory_file():
    filename = PathJoinSubstitution(
        [
            FindPackageShare('dual_arm_test'),
            'config',
            'traj_single_ur.yaml'
        ]
    )
    return filename


parameters = [
    {'name': 'trajectory_file',       'description': 'Trajectory file', 'default': default_trajectory_file()},
    {'name': 'n_cycles',              'description': 'Number of times to repeat the trajectory', 'default': '100'},
    {'name': 'robot_description',      'description': 'Path to the URDF/xacro file',                  'default': ''},
    {'name': 'controllers_file',       'description': 'Path to the ros2_control configuration file',  'default': 'controllers.yaml'},
    {'name': 'single_arm',       'description': 'simulating_1_arm=true, 2_arms=false',  'default': 'false'},
    {'name': 'tf_prefix',       'description': 'tf prefix',  'default': ''},
    {'name': 'use_fake_hardware',   'description': 'Start robot with fake hardware mirroring command to its states.',   'default': 'true'},
    {'name': 'fake_sensor_commands',   'description': 'Enable fake command interfaces for sensors used for simple simulations. Used only if use_fake_hardware parameter is true.',   'default': 'true'},
    {'name': 'robot_ip',       'description': 'simulating_1_arm=true, 2_arms=false',  'default': '172.31.1.138'},
    {'name': 'robot_ip_2',       'description': 'simulating_1_arm=true, 2_arms=false',  'default': '172.31.1.138'},
    {'name': 'ur_type',       'description': 'ur type',  'default': 'ur5e'},
    ]

def declare_launch_arguments():
  return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]

def generate_launch_description():
  return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])

def launch(context, *args, **kwargs):
  # Bring in launch parameters
  controllers_file = ParameterValue(LaunchConfiguration('controllers_file')).evaluate(context)
  single_arm = ParameterValue(LaunchConfiguration('single_arm')).evaluate(context)
  robot_ip = LaunchConfiguration("robot_ip")
  robot_ip_2 = LaunchConfiguration("robot_ip_2")
  use_fake_hardware = LaunchConfiguration('use_fake_hardware')
  fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
  ur_type = LaunchConfiguration('ur_type')

  if single_arm:
    arm_setup="single"
  else:
    arm_setup="multi"

# Get URDF via xacro
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [
                  FindPackageShare("dual_arm_test"),
                  "urdf/top_level.xacro",
              ],
            ),
            " ",
            "single_arm_setup:=",
            str(single_arm),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_ip_2:=",
            robot_ip_2,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "ur_type:=",
            ur_type,
      ]
  )
  robot_description = {"robot_description": robot_description_content}

  print(f"{robot_description_content}")
  robot_controllers = PathJoinSubstitution(
    [
      FindPackageShare("dual_arm_test"),
      "config/controllers.yaml",
    ],
  )

  rviz_config_file = PathJoinSubstitution(
      [FindPackageShare("dual_arm_test"), "config", "dual_arm_test.rviz"]
  )

  base_control_launch =  IncludeLaunchDescription(
      PythonLaunchDescriptionSource([FindPackageShare("dual_arm_test"), "/launch/ros2_control.launch.py"]),
      launch_arguments={
              "robot_description": robot_description_content,
              "controllers_file": robot_controllers,
      }.items(),)


  robot_state_pub_node = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      output="both",
      parameters=[robot_description],
      remappings=[("/joint_states",
      "/joint_state_broadcaster/joint_states",),],
  )

  rviz_node = Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      output="log",
      arguments=["-d",rviz_config_file]
  )

  trajectory_executor = Node(
      package='dual_arm_test',
      executable='trajectory_executor.py',
      parameters=[
          {
              'trajectory_file': LaunchConfiguration('trajectory_file'),
              'fjt_action': 'joint_trajectory_controller/follow_joint_trajectory',
              'n_cycles': LaunchConfiguration('n_cycles'),
          }
      ],
  )

  nodes = [
       base_control_launch,
       robot_state_pub_node,
       trajectory_executor,
       rviz_node]

  return nodes
