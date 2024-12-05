# This file was adopted from https://github.com/zijiechenrobotics/slam_in_autonomous_driving_docker

# DOCKER_BUILDKIT=1 docker build --build-arg USER_ID=$(id -u) --build-arg USER_NAME=$(whoami) --build-arg GROUP_ID=$(id -g) -t deep-blue-slam-rico .

FROM ubuntu:20.04

RUN apt-get update

# unminimize ubuntu
RUN yes | unminimize

# config CN environment
RUN apt install language-pack-zh-hans -y
RUN locale-gen
RUN /bin/bash -c "source ~/.bashrc"

# install xfce4
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y xfce4 xfce4-terminal

RUN apt install dbus-x11 -y
RUN apt install fonts-wqy-microhei -y
RUN apt install -y \
    gnome-user-docs-zh-hans \
    language-pack-gnome-zh-hans \
    # fcitx \
    # fcitx-pinyin \
    # fcitx-table-wubi \
    vim

# install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get install curl -y
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN apt install ros-noetic-desktop-full -y
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# install SAD dependence
RUN apt-get install -y \
    ros-noetic-pcl-ros \
    ros-noetic-velodyne-msgs \
    libopencv-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libpcl-dev\
    libyaml-cpp-dev \
    libbtbb-dev \
    libgmock-dev \
    pcl-tools \
    libspdlog-dev \
    libqglviewer-dev-qt5

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgl1-mesa-dev \
    libglew-dev \
    libglfw3-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libopenexr-dev \
    liblz4-dev \
    libzstd-dev \
    libeigen3-dev \
    libboost-all-dev \
    libglm-dev \
    libx11-dev \
    libxrandr-dev \
    libxi-dev \
    libxxf86vm-dev \
    libxinerama-dev \
    libxcursor-dev \
    libepoxy-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# install pangolin
RUN apt-get install git -y
WORKDIR /root/software
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
RUN cd Pangolin && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j8 && \
    make install && \
    ldconfig

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y tigervnc-standalone-server x11vnc && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get install tigervnc-standalone-server x11vnc -y
WORKDIR /root/.vnc
COPY ./docker/xstartup ./
RUN chmod u+x ~/.vnc/xstartup

# set up noVNC
WORKDIR /usr/lib
RUN git clone https://github.com/novnc/noVNC.git -o noVNC
WORKDIR /usr/lib/noVNC/utils
RUN git clone https://github.com/novnc/websockify.git -o websockify

WORKDIR /
COPY ./docker/startup.sh ./
RUN chmod u+x startup.sh
ENTRYPOINT ["./startup.sh"]
