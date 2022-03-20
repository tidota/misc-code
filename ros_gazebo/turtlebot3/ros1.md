# Turtlebot3 on Noetic

# Setup

## Real World

Refer to their e-manual https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

The following items are just my personal notes.

### Laptop PC

In `~/.bashrc`
```
DEV_NAME=<device name>
export ROS_MASTER_URI=http://`ip addr show dev $DEV_NAME | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1 -`:11311
export ROS_HOSTNAME=`ip addr show dev $DEV_NAME | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1 -`
```

### Turtlebot3

Download the disk image.
After downloading the image for Raspberry PI 3B+, check the file
```
echo "a7c57e20f2ee4204c95315866f4a274886094f7c63ed390b6d06d95074830309 *tb3_rpi3b+_noetic_20210708.zip" | shasum -a 256 --check
```

For writing the image into a SD card, just follow the instructions in their e-manual.

After that, the e-manual asks to edit some file like `writable/etc/netplan/50-cloud-init.yaml`.

In addition, maybe, apt systemd daily should be disabled? (https://linuxhint.com/apt-systemd-daily/)

To do so, edit `writable/etc/apt/apt.conf.d/20auto-upgrades` by replacing 1 with 0 so that it looks like this below.
```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

Physical setup:
1. Insert the SD card
1. Connect a LAN cable
1. Connect a monitor with a HDMI cable
1. Connect a USB keyboard
1. Connect the AC adaptor

Then, turn on the switch on the OpenCR.

#### Replacing networkd

Somehow, `networkd` did not work. (it may be a specific issue for Ubuntu 20.04?)
Instead, the network manager, which is for Ubuntu Desktop, worked.

Install `network-manager` by `sudo apt install network-manager`.

If it fails due to the locked file, kill the process of daily update.
https://unix.stackexchange.com/questions/315502/how-to-disable-apt-daily-service-on-ubuntu-cloud-vm-image
```
systemctl stop apt-daily.service
systemctl kill --kill-who=all apt-daily.service
```
If it is persistent and `sudo apt upgrade` shows the process ID of the running process, it may be necessary to (sudo) kill the process.

After installing `network-manager`, open the netplan file `/etc/netplan/50-cloud-init.yaml` and replace `networkd` with `NetworkManager`
```
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: true
      optional: true
  wifis:
    wlan0:
      dhcp4: yes
      dhcp6: yes
      access-points:
        WIFI_SSID:
          password: WIFI_PASSWORD
```

Shutdown Turtlebot3. Disconnect the LAN cable. Set up the wifi access point, and then start Turtlebot3 again.

Run `ip addr`. If it shows its IP address for wlan0, the network is correctly setup.

Then, shutdown Turtlebot3. Disconnect the HDMI cable and the USB keyboard. They are not necessary as Turtlebot3 can be now accessible through SSH.

### SSH connection

Assuming Turtlebot3 is accessing the wifi access point on the laptop PC.

Turn on Turtlebot3. And on the laptop PC, find the IP of turtlebot3
```
arp -a
```
Based on the IP address, log in to turtlebot3 by SSH
```
ssh ubuntu@<IP address>
```

Then, just move back to the e-manual.

## Simulation

Check this as reference: https://github.com/HotBlackRobotics/hotblackrobotics.github.io/blob/master/en/blog/_posts/2018-01-29-seq-goals-py.md

Assuming ROS noetic and Gazebo have been installed.

Install Turtlebot3 and some misc stuff
```
sudo apt-get install ros-noetic-turtlebot3-*
sudo apt install ros-noetic-dwa-local-planner
```

```
git clone https://github.com/FiorellaSibona/turtlebot3_nav.git
cd turtlebot3_nav/catkin_ws
rm -rf build devel
. /opt/ros/noetic/setup.bash
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

Start the simulation
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Manual control
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Mapping
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
To save the map
```
rosrun map_server map_saver -f ~/turtlebot_map
```

Navigation
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/turtlebot_map.yaml
```

# Frontier exploration

For preparation, install explore-lite by `sudo apt update && sudo apt install ros-noetic-explore-lite`

Start the simulation
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Start SLAM (gmapping)
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Start move-base
```
roslaunch turtlebot3_navigation move_base.launch
```

Start navigation
```
roslaunch explore_lite explore.launch
```
