# Miscellaneous notes

Yes, these are just my miscellaneous notes that are not a part of projects but somewhat personally helpful.

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
