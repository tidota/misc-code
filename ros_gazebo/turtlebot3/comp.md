# Comparison between ROS1 and ROS2

This page compares ROS1 and ROS2 features by comparing files for [my Turtlebot3 wallfollower repository](https://github.com/tidota/turtlebot3_wallfollower).

# CMakeLists.txt

diff --git a/CMakeLists.txt b/CMakeLists.txt
index aab575c..51e29e0 100644
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -1,60 +1,27 @@
-cmake_minimum_required(VERSION 3.5)
+cmake_minimum_required(VERSION 2.8.3)
 project(turtlebot3_wallfollower)

-# Default to C99
-if(NOT CMAKE_C_STANDARD)
-  set(CMAKE_C_STANDARD 99)
-endif()
+set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
+set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
+set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
+set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

-# Default to C++14
-if(NOT CMAKE_CXX_STANDARD)
-  set(CMAKE_CXX_STANDARD 14)
-endif()
+find_package(catkin REQUIRED COMPONENTS
+  roscpp
+  rospy
+  std_msgs

-if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
-  add_compile_options(-Wall -Wextra -Wpedantic)
-endif()
-
-# find dependencies
-find_package(ament_cmake REQUIRED)
-# uncomment the following section in order to fill in
-# further dependencies manually.
-# find_package(<dependency> REQUIRED)
+  geometry_msgs
+  sensor_msgs
+  std_srvs
+)

-find_package(rclcpp REQUIRED)
-find_package(geometry_msgs REQUIRED)
-# find_package(std_msgs REQUIRED)
-find_package(sensor_msgs REQUIRED)
-find_package(std_srvs REQUIRED)
+catkin_package()

-# add the include directory
 include_directories(
   include
+  ${catkin_INCLUDE_DIRS}
 )

 add_executable(wallfollower src/main.cpp src/wallfollower.cpp)
-ament_target_dependencies(
-  wallfollower
-  rclcpp geometry_msgs sensor_msgs std_srvs)
-install(TARGETS
-  wallfollower
-  DESTINATION lib/${PROJECT_NAME})
-
-if(BUILD_TESTING)
-  find_package(ament_lint_auto REQUIRED)
-  # the following line skips the linter which checks for copyrights
-  # uncomment the line when a copyright and license is not present in all source files
-  #set(ament_cmake_copyright_FOUND TRUE)
-  # the following line skips cpplint (only works in a git repo)
-  # uncomment the line when this package is not in a git repo
-  #set(ament_cmake_cpplint_FOUND TRUE)
-  ament_lint_auto_find_test_dependencies()
-endif()
-
-install(DIRECTORY
-  launch
-  rviz
-  DESTINATION share/${PROJECT_NAME}/
-)
-
-ament_package()
+target_link_libraries(wallfollower ${catkin_LIBRARIES})

---------------------------------------------------------------------------------------

# package.xml

diff --git a/package.xml b/package.xml
index ac1a2ee..1d315b9 100644
--- a/package.xml
+++ b/package.xml
@@ -1,23 +1,32 @@
 <?xml version="1.0"?>
-<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
-<package format="3">
+<package>
   <name>turtlebot3_wallfollower</name>
   <version>0.1.0</version>
   <description>Wall-following controller</description>
   <maintainer email="tidota@hawaii.edu">Tetsuya Idota</maintainer>
-  <license>Apache License 2.0</license>

-  <buildtool_depend>ament_cmake</buildtool_depend>
-  <depend>rclcpp</depend>
-  <depend>geometry_msgs</depend>
-  <!--<depend>std_msgs</depend>-->
-  <depend>sensor_msgs</depend>
-  <depend>std_srvs</depend>
+  <!-- One maintainer tag required, multiple allowed, one person per tag -->
+  <!-- Example:  -->
+  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
+  <maintainer email="tidota@hawaii.edu">Tetsuya Idota</maintainer>
+
+
+  <!-- One license tag required, multiple allowed, one license per tag -->
+  <!-- Commonly used license strings: -->
+  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
+  <license>BSD</license>
+
+  <buildtool_depend>catkin</buildtool_depend>

-  <test_depend>ament_lint_auto</test_depend>
-  <test_depend>ament_lint_common</test_depend>
+  <build_depend>roscpp</build_depend>
+  <build_depend>rospy</build_depend>
+  <build_depend>std_msgs</build_depend>
+  <build_depend>geometry_msgs</build_depend>
+  <build_depend>sensor_msgs</build_depend>

-  <export>
-    <build_type>ament_cmake</build_type>
-  </export>
+  <run_depend>roscpp</run_depend>
+  <run_depend>rospy</run_depend>
+  <run_depend>std_msgs</run_depend>
+  <run_depend>geometry_msgs</run_depend>
+  <run_depend>sensor_msgs</run_depend>
 </package>

----------------------------------------------------------------

# include/turtlebot3_wallfollower/wallfollower.hpp

diff --git a/include/turtlebot3_wallfollower/wallfollower.hpp b/include/turtlebot3_wallfollower/wallfollower.hpp
index 118110c..57073a3 100644
--- a/include/turtlebot3_wallfollower/wallfollower.hpp
+++ b/include/turtlebot3_wallfollower/wallfollower.hpp
@@ -2,32 +2,34 @@

 #include <mutex>

-#include <rclcpp/rclcpp.hpp>
-#include <geometry_msgs/msg/twist.hpp>
-#include <sensor_msgs/msg/laser_scan.hpp>
-#include <std_srvs/srv/set_bool.hpp>
+#include <ros/ros.h>

-class Wallfollower : public rclcpp::Node
+#include <geometry_msgs/Twist.h>
+#include <sensor_msgs/LaserScan.h>
+#include <std_srvs/SetBool.h>
+
+class Wallfollower
 {
   public: Wallfollower();
   private: void topic_callback(
-                  const sensor_msgs::msg::LaserScan::SharedPtr msg);
-  private: void timer_callback();
-  private: void set_running(
-    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
-          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
+                  const sensor_msgs::LaserScan::ConstPtr& msg);
+  private: void timer_callback(const ros::TimerEvent& event);
+  private: bool set_running(
+    std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
+
+  private: ros::NodeHandle nh;

   private:
-    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
+    ros::Publisher publisher_;
   private:
-    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
+    ros::Subscriber subscription_;
   private:
-    rclcpp::TimerBase::SharedPtr timer_;
+    ros::Timer timer_;
   private:
-    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
+    ros::ServiceServer service_;

   private: bool running_;

   private: std::mutex scan_mutex_;
-  private: sensor_msgs::msg::LaserScan scan_msg_buff_;
+  private: sensor_msgs::LaserScan scan_msg_buff_;
 };

-----------------------------------------------------------------

# src/wallfollower.cpp

diff --git a/src/wallfollower.cpp b/src/wallfollower.cpp
index 423b371..8716a57 100644
--- a/src/wallfollower.cpp
+++ b/src/wallfollower.cpp
@@ -3,64 +3,55 @@
 // the code was made by combining the two pieces of the sample code from
 // https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

-#include <chrono>
-#include <functional>
 #include <memory>
 #include <mutex>
 #include <string>

-#include <rclcpp/rclcpp.hpp>
-#include <geometry_msgs/msg/twist.hpp>
-#include <sensor_msgs/msg/laser_scan.hpp>
-#include <std_srvs/srv/set_bool.hpp>
+#include <ros/ros.h>

-#include "turtlebot3_wallfollower/wallfollower.hpp"
-
-using namespace std::chrono_literals;
+#include <geometry_msgs/Twist.h>
+#include <sensor_msgs/LaserScan.h>
+#include <std_srvs/SetBool.h>

-using std::placeholders::_1;
-using std::placeholders::_2;
+#include "turtlebot3_wallfollower/wallfollower.hpp"

 ////////////////////////////////////////////////////////////////////////////////
-Wallfollower::Wallfollower(): Node("wallfollower"), running_(false)
+Wallfollower::Wallfollower(): running_(false)
 {
   // Subscribe the topic for proximity data
   subscription_
-    = this->create_subscription<sensor_msgs::msg::LaserScan>(
-      "scan", rclcpp::SensorDataQoS(),
-      std::bind(&Wallfollower::topic_callback, this, _1));
+    = this->nh.subscribe("scan", 1000, &Wallfollower::topic_callback, this);

   // Make a publisher for controll
   publisher_
-    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
+    = this->nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

   // Make a timer to give a command at a specific interval.
   timer_
-    = this->create_wall_timer(
-      500ms, std::bind(&Wallfollower::timer_callback, this));
+    = this->nh.createTimer(
+        ros::Duration(0.5), &Wallfollower::timer_callback, this);

   // Make a service to receive a command.
   service_
-    = this->create_service<std_srvs::srv::SetBool>(
-      "set_running", std::bind(&Wallfollower::set_running, this, _1, _2));
+    = this->nh.advertiseService(
+        "set_running", &Wallfollower::set_running, this);
 }

 ////////////////////////////////////////////////////////////////////////////////
-void Wallfollower::topic_callback(
-  const sensor_msgs::msg::LaserScan::SharedPtr msg)
+void Wallfollower::topic_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
 {
   std::lock_guard<std::mutex> lk(scan_mutex_);
   scan_msg_buff_ = *msg;
 }

 ////////////////////////////////////////////////////////////////////////////////
-void Wallfollower::timer_callback()
+void Wallfollower::timer_callback(const ros::TimerEvent& event)
 {
-  auto message = geometry_msgs::msg::Twist();
+  auto message = geometry_msgs::Twist();

   if (this->running_)
   {
-    sensor_msgs::msg::LaserScan msg;
+    sensor_msgs::LaserScan msg;
     {
       std::lock_guard<std::mutex> lk(scan_mutex_);
       msg = scan_msg_buff_;
@@ -75,7 +66,7 @@ void Wallfollower::timer_callback()

     if (N == 0)
     {
-      RCLCPP_ERROR(this->get_logger(), "Empty scan data!");
+      ROS_ERROR_STREAM("Empty scan data!");
     }
     else
     {
@@ -143,24 +134,24 @@ void Wallfollower::timer_callback()
     }
   }

-  publisher_->publish(message);
+  publisher_.publish(message);
 }

 ////////////////////////////////////////////////////////////////////////////////
-void Wallfollower::set_running(
-  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
-        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
+bool Wallfollower::set_running(
+  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
 {
-  this->running_ = request->data;
-  response->success = true;
+  this->running_ = request.data;
+  response.success = true;
   if (this->running_)
   {
-    RCLCPP_INFO(this->get_logger(), "Got a command to run.");
-    response->message = "Run.";
+    ROS_INFO_STREAM("Got a command to run.");
+    response.message = "Run.";
   }
   else
   {
-    RCLCPP_INFO(this->get_logger(), "Got a command to stop.");
-    response->message = "Stop.";
+    ROS_INFO_STREAM("Got a command to stop.");
+    response.message = "Stop.";
   }
+  return true;
 }

---------------------------------------------------

# src/main.cpp

diff --git a/src/main.cpp b/src/main.cpp
index eb0817d..bc2cdd6 100644
--- a/src/main.cpp
+++ b/src/main.cpp
@@ -1,14 +1,14 @@
 // main.cpp

 #include <memory>
-#include <rclcpp/rclcpp.hpp>
+#include <ros/ros.h>
 #include "turtlebot3_wallfollower/wallfollower.hpp"

 ////////////////////////////////////////////////////////////////////////////////
 int main(int argc, char * argv[])
 {
-  rclcpp::init(argc, argv);
-  rclcpp::spin(std::make_shared<Wallfollower>());
-  rclcpp::shutdown();
+  ros::init(argc, argv, "wallfollower");
+  Wallfollower wallfollower;
+  ros::spin();
   return 0;
 }

-------------------------------------------------------

# launch/start_real.launch

diff --git a/launch/start_real.launch b/launch/start_real.launch
new file mode 100644
index 0000000..95437cd
--- /dev/null
+++ b/launch/start_real.launch
@@ -0,0 +1,13 @@
+<?xml version="1.0"?>
+<launch>
+  <arg name="output" default="screen"/>
+  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
+
+  <node pkg="turtlebot3_wallfollower" type="wallfollower" name="wallfollower" output="$(arg output)" />
+
+  <node pkg="rviz" type="rviz" name="rviz" output="screen" args="-d $(find turtlebot3_wallfollower)/rviz/default_view.rviz" />
+
+  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
+  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="$(arg output)"/>
+  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" output="$(arg output)"/>
+</launch>

--------------------------------------------------------------

# launch/start_sim.launch

diff --git a/launch/start_sim.launch b/launch/start_sim.launch
new file mode 100644
index 0000000..f43d616
--- /dev/null
+++ b/launch/start_sim.launch
@@ -0,0 +1,13 @@
+<?xml version="1.0"?>
+<launch>
+  <arg name="output" default="screen"/>
+
+  <include file="$(find turtlebot3_gazebo)/launch/turtlebot3_house.launch" />
+
+  <node pkg="turtlebot3_wallfollower" type="wallfollower" name="wallfollower" output="$(arg output)" />
+
+  <node pkg="rviz" type="rviz" name="rviz" output="screen" args="-d $(find turtlebot3_wallfollower)/rviz/default_view.rviz" />
+
+  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" output="$(arg output)"/>
+  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" output="$(arg output)"/>
+</launch>
