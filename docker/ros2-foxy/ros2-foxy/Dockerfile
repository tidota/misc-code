# Ubuntu 20.04
FROM ubuntu:20.04

# This line is to skip the interactive part that is incurred by installation of tzdata.
# With this line, somehow, the interactive menu no longer appears.
# https://github.com/phusion/baseimage-docker/issues/58
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# additional notes: make sure to use apt-get instead of apt
# https://github.com/hackafake/hackafake-backend/issues/32

# Tools I find useful during development
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        gdb \
        git \
        libbluetooth-dev \
        libcwiid-dev \
        libspnav-dev \
        libusb-dev \
        lsb-release \
        mercurial \
        python3-dbg \
        python3-pip \
        python3-venv \
        software-properties-common \
        sudo \
        vim \
        wget \
 && apt-get clean

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

RUN export TZ=America/Los_Angeles \
 && export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
 #&& sudo echo $TZ > /etc/timezone \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean

#-----

RUN export DEBIAN_FRONTEND=noninteractive \
 && echo "[Set Locale]" \
 && sudo apt-get update && sudo apt-get install -y  locales \
 && sudo locale-gen en_US en_US.UTF-8 \
 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && export LANG=en_US.UTF-8

RUN export DEBIAN_FRONTEND=noninteractive \
 && echo "[Setup Sources]" \
 && sudo rm -rf /var/lib/apt/lists/* && sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release \
 && sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

RUN export DEBIAN_FRONTEND=noninteractive \
 && echo "[Install ROS 2 packages]" \
 && sudo apt-get update && sudo apt-get install -y ros-foxy-desktop

RUN export DEBIAN_FRONTEND=noninteractive \
 && echo "[Environment setup]" \
 && . /opt/ros/foxy/setup.sh \
 && sudo apt-get install -y python3-argcomplete python3-colcon-common-extensions python3-vcstool git

#-----

RUN export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get install -y ros-foxy-gazebo-* \
                            ros-foxy-cartographer \
                            ros-foxy-cartographer-ros \
                            ros-foxy-navigation2 \
                            ros-foxy-nav2-bringup \
 && . ~/.bashrc \
 && sudo apt install -y ros-foxy-dynamixel-sdk \
                        ros-foxy-turtlebot3-msgs \
                        ros-foxy-turtlebot3 \
 && echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc \
 && . ~/.bashrc

#RUN sudo rosdep init \
# && sudo apt-get clean

#RUN rosdep update

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
