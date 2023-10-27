# Ubuntu 20.04
FROM osrf/ros:noetic-desktop-full

# Tools I find useful during development
RUN apt-get update \
 && apt-get install -y \
        build-essential \
        cmake \
        gdb \
        git \
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

RUN export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean

RUN rosdep update

RUN /bin/sh -c 'echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc'

RUN sudo apt-get update \
 && sudo apt-get install -y lsb-release wget gnupg \
 && sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
 && sudo apt-get update \
 && sudo apt-get install -y gz-garden

RUN sudo apt install -y ros-noetic-ros-ign-gazebo \
 && sudo apt install -y ros-noetic-ros-ign-gazebo-demos

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
