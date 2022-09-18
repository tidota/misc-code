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

# Japanese
RUN sudo apt-get update \
 && sudo apt-get install -y locales
RUN sudo apt-get install -y fcitx-mozc mozc-utils-gui fonts-noto-cjk language-pack-ja-base language-pack-ja
#RUN sudo apt-get install -y fcitx-mozc mozc-utils-gui fonts-noto-cjk language-pack-ja-base language-pack-ja dbus-x11

# misc
RUN sudo apt-get update \
 && sudo apt-get install -y ranger \
                            gedit \
 && sudo apt-get clean

RUN echo "set expandtab"     > /home/$USERNAME/.vimrc \
 && echo "set tabstop=2"    >> /home/$USERNAME/.vimrc \
 && echo "set shiftwidth=2" >> /home/$USERNAME/.vimrc


# Setup locale
RUN sudo localedef -f UTF-8 -i ja_JP ja_JP.utf8
RUN im-config -n fcitx
#RUN echo "ibus-daemon -d" >> /home/$USERNAME/.bashrc
RUN echo "fcitx-autostart < /dev/null 2> /dev/null > /dev/null &" >> /home/$USERNAME/.bashrc
#ENV GTK_IM_MODULE=xim
ENV GTK_IM_MODULE=fcitx
ENV XMODIFIERS=@im=fcitx
ENV QT_IM_MODULE=fcitx
ENV DefaultIMModule=fcitx
RUN sudo locale-gen ja_JP.UTF-8
ENV LANG ja_JP.UTF-8
ENV LANGUAGE ja_JP:ja
ENV LC_ALL=ja_JP.UTF-8
#RUN echo "sed -i 's/TriggerKey=CTRL_SPACE ZENKAKUHANKAKU/TriggerKey=CTRL_SPACE ALT_\`/g' /home/$USERNAME/.config/fcitx/config" >> /home/$USERNAME/.bashrc
#RUN echo "echo \"[Hotkey]\" > /home/$USERNAME/.config/fcitx/config" >> /home/$USERNAME/.bashrc
#RUN echo "echo \"TriggerKey=CTRL_SPACE ALT_\\\`\" >> /home/$USERNAME/.config/fcitx/config" >> /home/$USERNAME/.bashrc
RUN mkdir -p /home/$USERNAME/.config/fcitx
RUN echo "[Hotkey]" > /home/$USERNAME/.config/fcitx/config
RUN echo "TriggerKey=CTRL_SPACE ALT_\`" >> /home/$USERNAME/.config/fcitx/config

#RUN echo "/usr/bin/fcitx-autostart < /dev/null 2> /dev/null > /dev/null &" | sudo tee /etc/init.d/fcitx-startup.sh
#RUN sudo chmod +x /etc/init.d/fcitx-startup.sh

# Make a couple folders for organizing docker volumes
#RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# Customize your image here.
# E.g.:
# ENV PATH="/opt/sublime_text:$PATH"
