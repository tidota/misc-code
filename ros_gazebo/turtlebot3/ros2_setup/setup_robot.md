# Setup of the robot

The instructions for Foxy are available at https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/

The instructions below are for Humble.


Go to https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/

## Steps to setup Raspberry Pi

- Download Ubunu 22.04 Server from https://ubuntu.com/download/raspberry-pi

- Unzip the file

   ```
   unxz ubuntu-22.04.2-preinstalled-server-arm64+raspi.img.xz 
   ```
- burn the .img file to an SD card by Raspberry Pi Imager

- Hardware settings to setup

   - Connect devices

      - SD card
      - HDMI
      - USB input devices
   
   - Power on and log in (ubuntu/ubuntu)

      - in case the font is too small: https://askubuntu.com/questions/1402246/how-change-the-font-size-in-ubuntu-server-20-04-lts

      - edit files

         - `sudo nano /etc/apt/apt.conf.d/20auto-upgrades`

            ```
            APT::Periodic::Update-Package-Lists "0";
            APT::Periodic::Unattended-Upgrade "0";
            ```
         
         - `sudo nano /etc/netplan/50-cloud-init.yaml`

            ```
            network:
              version: 2
              renderer: networkd
              ethernets:
                eth0:
                  dhcp4: yes
                  dhcp6: yes
                  optional: true
              wifis:
                wlan0:
                  dhcp4: yes
                  dhcp6: yes
                  access-points:
                    (WIFI_SSID):
                      password: (WIFI_PASSWORD)
            ```

      - reboot

         - `sudo reboot`

- Download `install_ros2_humble_rpi.sh` and run it

- Setup the swap memory

`colcon build` in the next step may hung up! You may need to setup the swap memory.
https://github.com/ROBOTIS-GIT/turtlebot3/issues/460
https://github.com/ROBOTIS-GIT/turtlebot3/issues/965

On Turtlebot,
```
sudo swapoff /swapfile
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo nano /etc/fstab
```

In `/etc/fstab`, append `/swapfile swap swap defaults 0 0`.

Then, reboot Turtlebot.

- Run the following commands

Note: The line at `colcon build` may take a few hours.

```
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
sudo apt install ros-humble-hls-lfcd-lds-driver
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-dynamixel-sdk
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
colcon build --symlink-install --parallel-workers 1
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

- Run the following commands

```
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

- Run the following commands (this is for LDS-01)

```
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
source ~/.bashrc
```

## Steps to setup OpenRC

```
sudo dpkg --add-architecture armhf
sudo apt update
sudo apt install libc6:armhf
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
rm -rf ./opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xvf ./opencr_update.tar.bz2
cd ~/opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```
