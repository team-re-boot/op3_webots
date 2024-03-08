from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  launch_arguments = []
  def add_launch_arg(name: str, default_value=None, description=None):
    arg = DeclareLaunchArgument(
      name,
      default_value=default_value,
      description=description
    )
    launch_arguments.append(arg)

  add_launch_arg('gui', 'true', 'gui config')
  joint_state_pubilsher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    condition=IfCondition(LaunchConfiguration('gui')),
    remappings=[('/joint_states', '/robotis_op3/target_joint_states')]
  )

  return LaunchDescription(
    launch_arguments + 
    [
      joint_state_pubilsher_gui_node
    ]
  )
