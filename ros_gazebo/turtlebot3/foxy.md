# Turtlebot3 on Foxy

Reference: https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/

# Real world

## Turtlebot3
```
ros2 launch turtlebot3_bringup robot.launch.py
```

## SLAM
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/test
```

## Navigation
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/test.yaml
```

# Simulation

## Gazebo
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## SLAM
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## Navigation
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```

# control
```
ros2 run turtlebot3_teleop teleop_keyboard
```
