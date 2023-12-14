from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  return LaunchDescription([
    Node(
      package='turtle_ctrl',
      namespace='turtle1',
      executable='turtle_ctrl_node',
      name='turtle_ctrl_node1',
      output="screen",
      #emulate_tty=True,
      parameters=[
        {"lambda": 2.0},  # guadagno errore angolare
        {"gamma": 0.5}    # guadagno errore posizione
       ]
      ),
  ])

