# turtle_ctrl_ros2
Simple waypoint controller for the turtlesim package. Tested with Ubuntu 24.04 and ROS2 Kilted.

## Build

```
colcon build --symlink-install
```

## Run
Run directly the node:

```
ros2 run turtle_ctrl_ros2 turtle_ctrl_ros2_node
```

Use the launch file to set some gain parameters.
```
ros2 launch turtle_ctrl_ros2 launchCtrl.py
```

## Publish Goals

You can send the turtle to a specific point by publishing a `turtle_ctrl_ros2/msg/Goal` message on the topic "/turtle1/goal".
From command line:

```bash
ros2 topic pub --once /turtle1/goal turtle_ctrl_ros2/msg/Goal "{pos_x: 7.0, pos_y: 7.0}"
```
