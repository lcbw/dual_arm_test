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

parameters = [
    {'name': 'robot_description',      'description': 'Path to the URDF/xacro file',                  'default': ''},
    {'name': 'controllers_file',       'description': 'Path to the ros2_control configuration file',  'default': 'controllers.yaml'},
    {'name': 'single_arm',       'description': 'simulating_1_arm=true, 2_arms=false',  'default': 'true'},
    {'name': 'tf_prefix',       'description': 'tf prefix',  'default': ''},
    {'name': 'use_fake_hardware',   'description': 'Start robot with fake hardware mirroring command to its states.',   'default': 'true'},
    {'name': 'fake_sensor_commands',   'description': 'Enable fake command interfaces for sensors used for simple simulations. Used only if use_fake_hardware parameter is true.',   'default': 'true'},
    {'name': 'robot_ip',       'description': 'simulating_1_arm=true, 2_arms=false',  'default': '172.31.1.137'},
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
                  "armatrix.xacro",
              ],
            ),
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

  robot_controllers = PathJoinSubstitution(
    [
      FindPackageShare("dual_arm_test"),
      "controllers.yaml",
    ],
  )

  base_control_launch =  IncludeLaunchDescription(
      PythonLaunchDescriptionSource([FindPackageShare("dual_arm_test"), "/launch/ros2_control.launch.py"]),
      launch_arguments={
              "robot_description": robot_description_content,
              "controllers_file": robot_controllers,
      }.items(),)

  ft1 = Node(
      package="controller_manager",
      executable='spawner' if os.environ['ROS_DISTRO'] > 'foxy' else 'spawner.py',
      arguments=['robot1_force_torque_sensor_broadcaster', "-c", "/controller_manager"],
  )
  
  jsb = Node(
      package="controller_manager",
      executable='spawner' if os.environ['ROS_DISTRO'] > 'foxy' else 'spawner.py',
      arguments=['joint_state_broadcaster', "-c", "/controller_manager"],
  )

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
      arguments=["-d", rviz_config_file],
  )

  jtc = Node(
      package="controller_manager",
      executable='spawner' if os.environ['ROS_DISTRO'] > 'foxy' else 'spawner.py',
      arguments=['joint_trajectory_controller', "-c", "/controller_manager"],
  )

  trajectory_loader = Node(
        package="armatrix_support",
        executable="trajectory_loader.py",
        parameters=[
            {"trajectory_file_path": "config/trajectory_dual_ur.yaml"}
            ]
  )

  delay_joint_trajectory_controller = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=r1_admit,
          on_exit=[jtc],
      )
  )

  delay_trajectory_loader = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=jtc,
          on_exit=[trajectory_loader],
      )
  )

  nodes = [
      base_control_launch,
      r1_admit,
      ft1,
      jsb,
      delay_joint_trajectory_controller,
      delay_trajectory_loader,
      robot_state_pub_node,
      rviz_node]


  return nodes
