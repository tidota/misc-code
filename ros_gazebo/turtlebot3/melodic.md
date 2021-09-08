# Turtlebot3 on Melodic

Check this as reference: https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-seq-goals-py.md

Assuming ROS melodic and Gazebo have been installed.

Install Turtlebot3 and some misc stuff
```
sudo apt-get install ros-melodic-turtlebot3-*
sudo apt install ros-melodic-dwa-local-planner
```

```
git clone https://github.com/FiorellaSibona/turtlebot3_nav.git
cd turtlebot3_nav/catkin_ws
rm -rf build devel
. /opt/ros/melodic/setup.bash
catkin_make
. devel/setup.bash
```

# Run gazebo and rviz

Setup
```
export TURTLEBOT3_MODEL=burger
```
Then,
```
roslaunch simple_navigation_goals gazebo_navigation_rviz.launch
```
On Rviz, you can provide messages for initial pose and goals.

# SLAM

## Start the simulation
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

## Manual control
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

## Mapping
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
To save the map
```
rosrun map_server map_saver -f ~/turtlebot_map
```

## Navigation
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/turtlebot_map.yaml
```
