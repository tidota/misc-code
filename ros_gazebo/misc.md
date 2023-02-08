# Miscellaneous notes

This is just my miscellaneous notes that are not a part of projects but somewhat personally helpful.

# view_frames

`rosrun tf view_frames` no longer works on Noetic?

Seems like it is because it is now using tf2 instead of tf.
Install tf2-tools.
```
sudo apt install ros-noetic-tf2-tools
```
Then,
```
rosrun tf2_tools view_frames.py
```

# build systems and tools?

There are many jargons such as catkin, catkin build, colcon, etc...

https://design.ros2.org/articles/build_tool.html

- Build System

   Software to build a specific package
   
   e.g., CMake, catkin, ament_cmake

- Build Tools

   Using a build system, it determines the order to build multiple packages that may depend on the others.

   e.g., catkin_make, catkin_tools, ament_tools, colcon

ROS1 historycally uses catkin and catkin_make. THat's why the command to compile all the code in a workspace is `catkin_make`.

ROS2 uses ament_cmake (or possibly others?) and colcon. So `colcon build` is the command to compile code.
