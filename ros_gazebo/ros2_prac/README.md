# personal practice to use ROS2

Tutorials: https://index.ros.org/doc/ros2/Tutorials/

After install ROS2, you need to install
```
sudo apt install python3-colcon-common-extensions
```

# quick setup

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
