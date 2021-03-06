# Ubuntu 16.04
FROM ubuntu:xenial

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

RUN export DEBIAN_FRONTEND=noninteractive \
 && sudo apt-get update \
 && sudo -E apt-get install -y \
    tzdata \
 && sudo ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && sudo dpkg-reconfigure --frontend noninteractive tzdata \
 && sudo apt-get clean

RUN sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && sudo apt-get update \
 && sudo apt-get install -y \
    libgazebo7-dev \
    gazebo7 \
    python-catkin-tools \
    python-rosinstall \
    ros-kinetic-desktop-full \
    ros-kinetic-robot-localization \
    ros-kinetic-hector-localization \
    ros-kinetic-hector-gazebo \
    ros-kinetic-hector-models \
    ros-kinetic-teleop-twist-keyboard \
    ros-kinetic-teleop-twist-joy \
    ros-kinetic-ros-control \
    ros-kinetic-gazebo-ros-control \
    libignition-transport4 \
    libignition-transport4-dev \
 && sudo rosdep init \
 && sudo apt-get clean

RUN rosdep update

# RUN mkdir -p ~/catkin_ws/src \
#  && cd ~/catkin_ws/src \
#  && git clone -b kinetic-devel http://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git

RUN /bin/sh -c 'echo ". /opt/ros/kinetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc'

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
