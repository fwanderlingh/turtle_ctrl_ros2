# turtle_ctrl_ros2
Simple waypoint controller for the turtlesim package.

## Build

```
colcon build --symlink-install
```

## Run
Run directly the node:

```
ros2 run turtle_ctrl_ros2 turtle_ctrl_node
```

Use the launch file to set some gain parameters.
```
ros2 launch turtle_ctrl_ros2 launchCtrl.py
```
