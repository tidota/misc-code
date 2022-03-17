# Turtlebot3 on Foxy

Reference: https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/

# Real world

## Turtlebot3

Note: establish an access point and have turtlebot3 automatically access it

Find the IP of turtlebot3
```
arp -a
```

Log in to turtlebot3 by SSH
```
ssh ubuntu@<IP address>
```

Then, launch the file.
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

## keyboard
```
ros2 run turtlebot3_teleop teleop_keyboard
```

## joypad
```
ros2 run joy joy_node
```

```
ros2 run teleop_twist_joy teleop_node
```

to check the game pad button
```
sudo jstest /dev/input/js0
```

### Sanwa gamepad config

reference: https://docs.ros.org/en/foxy/Guides/Node-arguments.html

#### by one-line command line
```
ros2 run teleop_twist_joy teleop_node --ros-args -p enable_button:=8 -p enable_turbo_button:=9 -p axis_linear.x:=1 -p axis_angular.yaw:=0 -p scale_linear.x:=0.1 -p scale_linear_turbo.x:=0.2 -p scale_angular.yaw:=1.0
```

#### by using a config file
sanwa.config.yaml
```
teleop_twist_joy_node:
  ros__parameters:
    axis_linear:  # Left thumb stick vertical
      x: 1
    scale_linear:
      x: 0.1
    scale_linear_turbo:
      x: 0.2

    axis_angular:  # Left thumb stick horizontal
      yaw: 0
    scale_angular:
      yaw: 1.0

    enable_button: 8  # Right trigger button 1
    enable_turbo_button: 9  # Right trigger button 2
```

```
ros2 run teleop_twist_joy teleop_node --ros-args --params-file sanwa.config.yaml
```