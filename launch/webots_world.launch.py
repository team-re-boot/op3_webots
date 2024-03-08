import os
import xacro
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from ament_index_python import get_package_share_path

def generate_launch_description():
  package_dir = get_package_share_directory('op3_webots')
  robotis_op3_urdf_path= os.path.join(get_package_share_directory('op3_description'), 'urdf', 'robotis_op3.urdf.xacro')
  robot_description = xacro.process_file(robotis_op3_urdf_path, mappings={'name': 'robotis_op3'}).toxml()

  webots = WebotsLauncher(
    world=PathJoinSubstitution([package_dir, 'world', 'robotis_op3.wbt']),
  )
  ros2_supervisor = Ros2SupervisorLauncher()

  op3_robot_driver = WebotsController(
    robot_name='ROBOTIS OP3',
    namespace='robotis_op3',
    parameters=[
      {
        "robot_description": os.path.join(
          package_dir, "resource", "RobotisOp3.urdf"
        ),
      },
      {"use_sim_time": True},
      {"set_robot_state_publisher": False},
    ],
    respawn=True
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{
        'robot_description': robot_description,
        "use_sim_time": True,
    }],
    remappings=[('/joint_states', '/robotis_op3/joint_states')]
  )

  rviz_config_path = os.path.join(get_package_share_path('op3_webots'), 'rviz', 'op3.rviz')
  rviz2 = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_path]
  )

  return LaunchDescription(
    [
      webots,
      ros2_supervisor,
      robot_state_publisher,
      op3_robot_driver,
      rviz2,

      launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
          target_action=webots,
          on_exit=[
              launch.actions.EmitEvent(event=launch.events.Shutdown())
          ],
        )
      ),
    ]
  )
