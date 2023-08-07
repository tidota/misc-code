# Turtlebot3 with ROS2

# Setup

Refer to their e-manual https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/, and click the "Foxy" tab.

Note (Feb 7, 2023):

They refer to ROS2 Foxy, but the life time of support is expiring in 2023. So this should move to ROS2 Humble, which runs on Ubuntu 22.04.
The only difference should be the package names and the `foxy` in each package name should be repalced with `humble`.


- Setup for ROS2 Foxy

   - Run [install_ros2_foxy.sh](./ros2_setup/install_ros2_foxy.sh)

   - Run [commands to install dependencies](./ros2_setup/foxy.md)

- Setup for ROS2 Humble

   - Run [install_ros2_humble.sh](./ros2_setup/install_ros2_humble.sh)

   - Run [commands to install dependencies](./ros2_setup/humble.md)

- Setup for the robot

   - Check [setup_robot.md](./ros2_setup/setup_robot.md)

# Real world

Reference: https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/

## Turtlebot3

Note: establish an access point and have turtlebot3 automatically access it

Find the IP of turtlebot3
```
arp -a
```
Another way is arp-scan
```
sudo arp-scan -I <interface name> <network>
```
example
```
sudo arp-scan -I wlan 192.168.0.0/24
```

Log in to turtlebot3 by SSH
```
ssh ubuntu@<IP address>
```

Then, launch the file.
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

## teleop

```
ros2 run joy joy_node
```

### xbox

```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

## SLAM
To build a map
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
To save the built map
```
ros2 run nav2_map_server map_saver_cli -f ~/test
```

cartographer:
- cartographer.launch.py
- occupancy_grid.launch.py


## Navigation
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/test.yaml
```
navigation2.launch.py uses this launch file.
nav2_bringup:
- bringup_launch.py
And this launch file also uses others?

# Simulation

Reference: https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/

To install the simulation package (ROS Humble)
```
sudo apt install ros-humble-turtlebot3-gazebo
```

## Gazebo
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## SLAM
To build a map,
```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
To save the built map
```
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
