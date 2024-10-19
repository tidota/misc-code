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

## Discovery Server (if multicast is blocked by the wifi router)

When the PC and Turtlebot are connected through wifi, there may be an issue that the ROS nodes and topics on Turtlebot are not visible from the PC.
It may be related to multicast being blocked by the wifi router.
In such a case, a discovery server needs to run.
https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html

Run the discovery server.
```
fastdds discovery --server-id 0
```

In `.bashrc` of both the PC and Turtlebot, add `export ROS_DISCOVERY_SERVER=<IP address>:11811` (Replace `<IP address>` with the IP address of the machine on which the server is running)

Then, source `.bashrc` (`. ~/.bashrc`), and launch the corresponding nodes on Turtlebot and the PC.

### ROS2 commands with the discovery server

To use ros2 commands such as `ros2 topic` and `ros2 node`, addional setup is required: https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html#ros-2-introspection

Save a file named `super_client_configuration_file.xml` containing the following text:
```
<?xml version="1.0" encoding="UTF-8" ?>
 <dds>
     <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
         <participant profile_name="super_client_profile" is_default_profile="true">
             <rtps>
                 <builtin>
                     <discovery_config>
                         <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
                         <discoveryServersList>
                             <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                                 <metatrafficUnicastLocatorList>
                                     <locator>
                                         <udpv4>
                                             <address>127.0.0.1</address>
                                             <port>11811</port>
                                         </udpv4>
                                     </locator>
                                 </metatrafficUnicastLocatorList>
                             </RemoteServer>
                         </discoveryServersList>
                     </discovery_config>
                 </builtin>
             </rtps>
         </participant>
     </profiles>
 </dds>
```

Then, append `export FASTRTPS_DEFAULT_PROFILES_FILE=<path to super_client_configuration_file.xml>` (replace `<path to super_client_configuration_file.xml>` with the absolute path to the file)

Then, source `.bashrc` (`. ~/.bashrc`), and restart the daemon:
```
ros2 daemon stop
ros2 daemon start
```

Finally, you can see the topics by `ros2 topic list`.


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
ros2 run nav2_map_server map_saver_cli -f ~/map
```

cartographer:
- cartographer.launch.py
- occupancy_grid.launch.py


## Navigation
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```
navigation2.launch.py uses this launch file.
nav2_bringup:
- bringup_launch.py
And this launch file also uses others?

Note: in case of some error complaining about nav2, edit `/opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml`
and change the value of `robot_model_type` as follows.
```
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
```
As of 8/20/2023, the debian package does not have this fix, so this parameter must be manually edited.

Even after the fix above, the AMCL gives this error.
```
[amcl]: AMCL cannot publish a pose or update the transform. Please set the initial pose...
```
But once the pose is manually set through rviz, it starts to work!

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

Note(2024/10/19): The gzserver seems not to like the settings for the discovery server. Before running a simulation, the following environment variables should be commented out in ~/.bashrc.
- ROS_DISCOVERY_SERVER
- FASTRTPS_DEFAULT_PROFILES_FILE

Note(2024/10/19): The gzclient had some issue that it crashes right after launching. The following command fixed the issue: `. /usr/share/gazebo/setup.sh`.
This should be included in ~/.bashrc.
For more details: https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/

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
