# Setup of the robot

The instructions for Foxy are available at https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/

The instructions below are for Humble.


TODO:
- go to https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/
- Check Manual SBS setup instructions

- Does ubuntu 22.04 has different wifi settings?
   networkd? NetworkManager?

Steps

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

