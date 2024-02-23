import os
import pathlib
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
  package_dir = get_package_share_directory('op3_webots')
  robotis_op3_urdf_path = os.path.join(package_dir, 'resource', 'RobotisOp3.urdf')
  robot_description = pathlib.Path(robotis_op3_urdf_path).read_text()
  op3_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yaml')

  webots = WebotsLauncher(
    world=PathJoinSubstitution([package_dir, 'world', 'robotis_op3.wbt']),
    ros2_supervisor=True
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[
        {'robot_description': robot_description},
    ],
  )

  controller_manager_timeout = ['--controller-manager-timeout', '50']
  controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
  use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'
  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
    output='screen',
    prefix=controller_manager_prefix,
    arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    parameters=[
      {'use_sim_time': True}
    ]
  )

  ros_control_spawners = [joint_state_broadcaster_spawner]

  ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')

  op3_driver = Node(
    package='webots_ros2_driver',
    executable='driver',
    output='screen',
    additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'RobotisOp3'},
    parameters=[
      {'robot_description': robot_description,
      'use_sim_time': True,
      'set_robot_state_publisher' : True},
      ros2_control_params
    ],
    respawn=True
  )

  waiting_nodes = WaitForControllerConnection(
      target_driver=op3_driver,
      nodes_to_start=ros_control_spawners
    )

  return LaunchDescription([
    webots,
    webots._supervisor,
    robot_state_publisher,
    op3_driver,
    waiting_nodes,

    launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[
                launch.actions.EmitEvent(event=launch.events.Shutdown())
            ],
        )
    ),
  ])