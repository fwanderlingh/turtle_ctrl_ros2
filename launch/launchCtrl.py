import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  config = os.path.join(
      get_package_share_directory('turtle_ctrl'),
      'config',
      'params.yaml'
  )

  return LaunchDescription([
    DeclareLaunchArgument(
        'params_file', 
        default_value='$(find turtle_ctrl)/config/params.yaml', 
        description='Path to the parameter file'
    ),
    
    Node(
      package='turtle_ctrl',
      #namespace='turtle1',
      executable='turtle_ctrl_node',
      name='turtle_ctrl_node1',
      output="screen",
      #emulate_tty=True,
      #parameters=[
      #  {"lambda": 2.0},  # guadagno errore angolare
      #  {"gamma": 0.5}    # guadagno errore posizione
      #],
      parameters=[config],
      #parameters=['$(arg params_file)']
      ),
  ])

