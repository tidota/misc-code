# personal practice to use ROS2

Tutorials: https://index.ros.org/doc/ros2/Tutorials/

After install ROS2, you need to install
```
sudo apt install python3-colcon-common-extensions
```

# quick setup
https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/

Create `ros2_setup.bash` containing the following contents:
```
#!/bin/bash
. /opt/ros/dashing/setup.bash
. /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/dashing
```

Then, `. ./ros2_setup.bash`

# (incomplete) comparison table

Most of the commands are straight forward.

| ROS2 | ROS1     |
| :------------- | :------------- |
| `ros2 run`  | `rosrun`|
| `ros2 node list/info` | `rosnode list/info` |
| `ros2 topic list/echo/info/pub/hz` | `rostopic list/echo/info/pub/hz` |
| `ros2 msg(interface) show <type>` | `rosmsg show <type>` |
| `ros2 service list/type/call` | `rosservice list/type/call` |
| `ros2 srv(interface) show <type>` | `rossrv show <type>` |
| `ros2 param list/get/set/dump` | `rosparam list/get/set/dump` |
| `ros2 action list/send_goal` | N/A? |
| `ros2 launch <package> <launch file>` | `roslaunch <package> <launch file>` |
| `ros2 launch <launch file in the current directory` | N/A |
| `ros2 bag record/info/play`* | `rosbag record/info/play`|
| `colcon_cd`* | `roscd` |
| `ros2 pkg executables <package>` | N/A?  |
| `rqt`* | N/A? |
| `rqt_graph`* | `rosrun rqt_graph rqt_graph` |
| `ros2 run rqt_console rqt_console`* | `rosrun rqt_console rqt_console` |
| `ros2 pkg create --build-type ament_cmake/python [--node-name <node>] <package>` | `catkin_create_pkg <package> <dependencies>` |
| `colcon build [--packages-select <package>]` | `catkin_make` |

## \*Note: must install additional packages.

- colcon-cd

  ```
  pip install -U colcon-common-extensions
  ```

  and setup the environment:
  ```
  . .local/share/colcon_cd/function/colcon_cd.sh
  export _colcon_cd_root=~/ros2_install
  ```

- rqt

  ```
  sudo apt install ros-dashing-rqt*
  ```

- ros2 bag

  ```
  sudo apt install ros-dashing-ros2bag ros-dashing-rosbag2-converter-default-plugins ros-dashing-rosbag2-storage-default-plugins
  ```

# environment variables

- `printenv | grep -i ROS`

  printout environment variables related to ROS

- `ROS_DOMAIN_ID`

  `export ROS_DOMAIN_ID=1`

# running demo
```
ros2 run demo_nodes_cpp talker
```
```
ros2 run demo_nodes_py listener
```

# turtlesim
https://index.ros.org/doc/ros2/Tutorials/Turtlesim/Introducing-Turtlesim/

Install turtlesim.
```
sudo apt update
sudo apt install ros-dashing-turtlesim
```

Check the installation.
```
$ ros2 pkg executables turtlesim
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

Run the turtle sim.
```
ros2 run turtlesim turtlesim_node
```

Control it in another terminal.
```
ros2 run turtlesim turtle_teleop_key
```

To run the node with a mapped topic.
```
ros2 run turtlesim turtle_teleop_key turtle1/cmd_vel:=turtle2/cmd_vel
```

To run the node with another node name.
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

Publish a message.
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```

## rqt
```
sudo apt update
sudo apt install ros-<distro>-rqt*
```

`rqt` can run by this command.
```
rqt
```
Choose a plugin in "Plugins" of the manu bar.

# launch

https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/


`turtlesim_mimic_launch.py`
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            node_namespace='turtlesim1',
            node_executable='turtlesim_node',
            node_name='sim'
        ),
        Node(
            package='turtlesim',
            node_namespace='turtlesim2',
            node_executable='turtlesim_node',
            node_name='sim'
        ),
        Node(
            package='turtlesim',
            node_executable='mimic',
            node_name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

```
ros2 launch turtlesim_mimic_launch.py
```

# rosbag

Install the related packages.
```
sudo apt install ros-dashing-ros2bag ros-dashing-rosbag2-converter-default-plugins ros-dashing-rosbag2-storage-default-plugins
```

```
ros2 bag record -o <output> <topic name>
```

```
ros2 bag info <rosbag data>
ros2 bag play <rosbag data>
```

# workspace

https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/


```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/ros/ros_tutorials.git -b dashing-devel
rosdep install -i --from-path src --rosdistro dashing -y
colcon build
. install/local_setup.bash
```

colcon build options:
- `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
- `--symlink-install` saves you from having to rebuild every time you tweak python scripts
- `--event-handlers console_direct+` shows console output while building (can otherwise be found in the log directory)

## create a pakcage

https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/
