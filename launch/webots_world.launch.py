import os
import pathlib
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
import launch
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher 

def generate_launch_description():
  package_dir = get_package_share_directory('op3_webots')
  robotis_op3_urdf_path = os.path.join(package_dir, 'resource', 'RobotisOp3.urdf')

  webots = WebotsLauncher(
    world=PathJoinSubstitution([package_dir, 'world', 'robotis_op3.wbt']),
    ros2_supervisor=True
  )

  return LaunchDescription([
    webots,
    webots._supervisor,

    launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[
                launch.actions.EmitEvent(event=launch.events.Shutdown())
            ],
        )
    ),
  ])