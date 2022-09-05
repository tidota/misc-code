FROM ubuntu:22.04

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

RUN sudo apt-get update \
 && sudo apt-get install -y lifeograph \
 && sudo apt-get clean

RUN cd /home/$USERNAME \
 && git clone https://github.com/tfussell/xlnt.git \
 && cd xlnt/third-party \
 && git clone https://git.codesynthesis.com/libstudxml/libstudxml.git \
 && cd .. \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make \
 && sudo make install \
 && cd /home/$USERNAME \
 && rm -rf xlnt

RUN sudo apt-get install -y libgtest-dev cmake \
 && cd /usr/src/gtest \
 && sudo cmake . \
 && sudo make \
 && cd lib \
 && sudo cp *.a /usr/lib

RUN sudo apt-get install -y openjdk-8-jdk \
 && sudo apt-get clean

# Make a couple folders for organizing docker volumes
#RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
