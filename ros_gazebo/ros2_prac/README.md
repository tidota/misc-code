# personal practice to use ROS2

Tutorials: https://index.ros.org/doc/ros2/Tutorials/

After install ROS2, you need to install
```
sudo apt install python3-colcon-common-extensions
```

# quick setup
https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/

```
. /opt/ros/dashing/setup.bash
. /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/dashing
```

# running demo
```
ros2 run demo_nodes_cpp talker
```
```
ros2 run demo_nodes_py listener
```

# (incomplete) comparison table

| ROS2 | ROS1     |
| :------------- | :------------- |
| `ros2 run`  | `rosrun`|
| `colcon_cd`* | `roscd` |


\*Note: must install `colcon-cd`
```
pip install -U colcon-common-extensions
```
```
. .local/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/ros2_install
```

# environment variables

- `printenv | grep -i ROS`

  printout environment variables related to ROS

- `ROS_DOMAIN_ID`

  `export ROS_DOMAIN_ID=1`


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
