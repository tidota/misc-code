# Comparison between ROS1 and ROS2

This page compares ROS1 and ROS2 features by comparing files for [my Turtlebot3 wallfollower repository](https://github.com/tidota/turtlebot3_wallfollower).

# CMakeLists.txt

## catkin (ROS1) vs ament_cmake (ROS2)

ROS1
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
```

ROS2
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)
```

## add_executable (common)
```
add_executable(wallfollower src/main.cpp src/wallfollower.cpp)
```

## target_link_libraries (ROS1) vs ament_target_dependencies (ROS2)

ROS1
```
target_link_libraries(wallfollower ${catkin_LIBRARIES})
```

ROS2 (the install and test parts may not be really necessary?)
```
ament_target_dependencies(
  wallfollower
  rclcpp geometry_msgs sensor_msgs std_srvs)

install(TARGETS
  wallfollower
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

---------------------------------------------------------------------------------------

# package.xml

## buildtool_depend

ROS1
```
  <buildtool_depend>catkin</buildtool_depend>
```
ROS2
```
  <buildtool_depend>ament_cmake</buildtool_depend>
```

## dependencies

ROS1
```
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>rospy</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>geometry_msgs</run_depend>
  <run_depend>sensor_msgs</run_depend>
```
ROS2
```
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_srvs</depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
```

----------------------------------------------------------------

# include/turtlebot3_wallfollower/wallfollower.hpp

## Include Directives

ROS1
```
#include <mutex>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>
```

ROS2
```
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>
```

## Definition of the Class

ROS1
```
class Wallfollower
{
  public: Wallfollower();

  private: void topic_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
  private: void timer_callback(const ros::TimerEvent& event);
  private: bool set_running(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response);

  private: ros::NodeHandle nh;

  private: ros::Publisher publisher_;
  private: ros::Subscriber subscription_;
  private: ros::Timer timer_;
  private: ros::ServiceServer service_;
  private: sensor_msgs::LaserScan scan_msg_buff_;

  private: bool running_;
  private: std::mutex scan_mutex_;
};
```

ROS2
```
class Wallfollower : public rclcpp::Node
{
  public: Wallfollower();

  private: void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  private: void timer_callback();
  private: void set_running(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  private: rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  private: rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  private: rclcpp::TimerBase::SharedPtr timer_;
  private: rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  private: sensor_msgs::msg::LaserScan scan_msg_buff_;

  private: bool running_;
  private: std::mutex scan_mutex_;
};
```

-----------------------------------------------------------------

# src/wallfollower.cpp

## Include Directives

ROS1
```
#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

#include "turtlebot3_wallfollower/wallfollower.hpp"
```

ROS2
```
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "turtlebot3_wallfollower/wallfollower.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
```

## Constructor

ROS1
```
Wallfollower::Wallfollower(): running_(false)
{
  // Subscribe the topic for proximity data
  subscription_
    = this->nh.subscribe("scan", 1000, &Wallfollower::topic_callback, this);

  // Make a publisher for controll
  publisher_
    = this->nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Make a timer to give a command at a specific interval.
  timer_
    = this->nh.createTimer(ros::Duration(0.5), &Wallfollower::timer_callback, this);

  // Make a service to receive a command.
  service_
     = this->nh.advertiseService("set_running", &Wallfollower::set_running, this);
}
```

ROS2
```
Wallfollower::Wallfollower(): Node("wallfollower"), running_(false)
{
  // Subscribe the topic for proximity data
  subscription_
    = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&Wallfollower::topic_callback, this, _1));

  // Make a publisher for controll
  publisher_
    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Make a timer to give a command at a specific interval.
  timer_
    = this->nh.createTimer(ros::Duration(0.5), &Wallfollower::timer_callback, this);

  // Make a service to receive a command.
  service_
    = this->create_service<std_srvs::srv::SetBool>(
        "set_running", std::bind(&Wallfollower::set_running, this, _1, _2));
}
```

## Subscriber Callback

ROS1
```
void Wallfollower::topic_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(scan_mutex_);
  scan_msg_buff_ = *msg;
}
```

ROS2
```
void Wallfollower::topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(scan_mutex_);
  scan_msg_buff_ = *msg;
}
```

## Timer Callback

ROS1
```
void Wallfollower::timer_callback(const ros::TimerEvent& event)
{
  auto message = geometry_msgs::Twist();
  if (this->running_)
  {
    sensor_msgs::LaserScan msg;

    ...

    if (N == 0)
    {
      ROS_ERROR_STREAM("Empty scan data!");
    }

    ...

  }

  publisher_.publish(message);
}
```
ROS2
```
void Wallfollower::timer_callback()
{
  auto message = geometry_msgs::msg::Twist();
  if (this->running_)
  {
    sensor_msgs::msg::LaserScan msg;

    ...

    if (N == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Empty scan data!");
    }

    ...

  }

  publisher_->publish(message);
}
```

## Service Callback

ROS1
```
bool Wallfollower::set_running(
  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  this->running_ = request.data;
  response.success = true;
  if (this->running_)
  {
    ROS_INFO_STREAM("Got a command to run.");
    response.message = "Run.";
  }
  else
  {
    ROS_INFO_STREAM("Got a command to stop.");
    response.message = "Stop.";
  }

  return true;
}
```

ROS2
```
void Wallfollower::set_running(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  this->running_ = request->data;
  response->success = true;
  if (this->running_)
  {
    RCLCPP_INFO(this->get_logger(), "Got a command to run.");
    response->message = "Run.";
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Got a command to stop.");
    response->message = "Stop.";
  }

  return true;
}
```

---------------------------------------------------

# src/main.cpp

## Include Directives

ROS1
```
#include <memory>

#include <ros/ros.h>

#include "turtlebot3_wallfollower/wallfollower.hpp"
```

ROS2
```
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "turtlebot3_wallfollower/wallfollower.hpp"
```

## Main

ROS1
```
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "wallfollower");
  Wallfollower wallfollower;
  ros::spin();

  return 0;
}
```

ROS2
```
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Wallfollower>());
  rclcpp::shutdown();

  return 0;
}
```

-------------------------------------------------------

# launch/start_real.launch

ROS1
```
<?xml version="1.0"?>
<launch>
  <arg name="output" default="screen"/>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>

  <node pkg="turtlebot3_wallfollower" type="wallfollower" name="wallfollower" output="$(arg output)" />

  <node pkg="rviz" type="rviz" name="rviz" output="screen" args="-d $(find turtlebot3_wallfollower)/rviz/default_view.rviz" />

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="$(arg output)"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" output="$(arg output)"/>
</launch>
```

ROS2
```
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_dir = os.path.join(
            get_package_share_directory('turtlebot3_wallfollower'),
            'rviz',
            'default_view.rviz')

    return LaunchDescription([
        Node(
            package='turtlebot3_wallfollower',
            executable='wallfollower',
            name='wallfollower',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
        ),
    ])
```

--------------------------------------------------------------

# launch/start_sim.launch

ROS1
```
<?xml version="1.0"?>
<launch>
  <arg name="output" default="screen"/>

  <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_house.launch" />

  <node pkg="turtlebot3_wallfollower" type="wallfollower" name="wallfollower" output="$(arg output)" />

  <node pkg="rviz" type="rviz" name="rviz" output="screen" args="-d $(find turtlebot3_wallfollower)/rviz/default_view.rviz" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="$(arg output)"/>
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" output="$(arg output)"/>
</launch>
```

ROS2
```
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    rviz_config_dir = os.path.join(
            get_package_share_directory('turtlebot3_wallfollower'),
            'rviz',
            'default_view.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, \
                             'launch', 'turtlebot3_house.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='turtlebot3_wallfollower',
            executable='wallfollower',
            name='wallfollower',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
```
