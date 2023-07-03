# FROM ubuntu:focal
FROM ghcr.io/linuxserver/baseimage-ubuntu:focal

# set version label
ARG BUILD_DATE
ARG VERSION
ARG CODE_RELEASE
LABEL build_version="Linuxserver.io version:- ${VERSION} Build-date:- ${BUILD_DATE}"
LABEL maintainer="aptalca"

# environment settings
ARG DEBIAN_FRONTEND="noninteractive"
ENV HOME="/config"

RUN echo "**** install runtime dependencies ****" && \
    apt-get update && \
    apt-get install -y \
    git \
    jq \
    libatomic1 \
    nano \
    net-tools \
    iputils-ping \
    netcat \
    fluxbox \
    x11vnc \
    xvfb \
    xterm \
    openbox \
    sudo && \
    echo "**** install code-server ****" && \
    if [ -z ${CODE_RELEASE+x} ]; then \
      CODE_RELEASE=$(curl -sX GET https://api.github.com/repos/coder/code-server/releases/latest \
        | awk '/tag_name/{print $4;exit}' FS='[""]' | sed 's|^v||'); \
    fi && \
    mkdir -p /app/code-server && \
    curl -o \
      /tmp/code-server.tar.gz -L \
      "https://github.com/coder/code-server/releases/download/v${CODE_RELEASE}/code-server-${CODE_RELEASE}-linux-amd64.tar.gz" && \
    tar xf /tmp/code-server.tar.gz -C \
      /app/code-server --strip-components=1 && \
    echo "**** clean up ****" && \
    apt-get clean && \
    rm -rf \
      /config/* \
      /tmp/* \
      /var/lib/apt/lists/* \
      /var/tmp/*




######################
#   foxGlove
######################
# add local files
COPY /root /
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*
# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*




######################
#   ROS
######################
# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list && \
    echo "deb http://snapshots.ros.org/galactic/final/ubuntu focal main" > /etc/apt/sources.list.d/ros2-snapshots.list && \
    echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
# # install ros packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-ros-core=1.5.0-1* \
#     ros-galactic-ros-core=0.9.3-2* \
#     ros-foxy-ros-core=0.9.2-1* \
#     && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pip \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    python3-vcstool \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*
# bootstrap rosdep
# RUN rosdep init && \
#     rosdep update --rosdistro noetic && \
#     rosdep update --rosdistro galactic && \
#     rosdep update --rosdistro foxy
RUN pip3 install rosdepc
RUN rosdepc init
RUN rosdepc update --rosdistro noetic
RUN rosdepc update --rosdistro galactic
RUN rosdepc update --rosdistro foxy

RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-ros-base=1.5.0-1* \
#     ros-galactic-ros-base=0.9.3-2* \
#     ros-foxy-ros-base=0.9.2-1* \
#     && rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-robot=1.5.0-1* \
#     ros-galactic-desktop=0.9.3-2* \
#     ros-foxy-desktop=0.9.2-1* \
#     && rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-desktop=1.5.0-1* \
#     && rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-noetic-desktop-full=1.5.0-1* \
#     && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    ros-galactic-desktop=0.9.3-2* \
    ros-foxy-desktop=0.9.2-1* \
    ros-noetic-rosbridge-suite \
    ros-galactic-rosbridge-suite \
    ros-foxy-rosbridge-suite \
    nginx \
    vim \
    && rm -rf /var/lib/apt/lists/*




######################
#   gzweb
######################
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-gazebo-* \
    ros-galactic-gazebo-* \
    ros-foxy-gazebo-* \
    libjansson-dev \
    libboost-dev \
    imagemagick \
    libtinyxml-dev \
    mercurial \
    cmake \
    build-essential \
    nodejs \
    npm \
    && rm -rf /var/lib/apt/lists/*
RUN npm install -g n && n stable
RUN cd / && \
    git clone https://github.com/osrf/gzweb && \
    cd /gzweb && \
    /bin/bash -c 'source /usr/share/gazebo/setup.sh; npm run deploy ---'
COPY assets.tar.xz /gzweb/http/client/assets.tar.xz
COPY models.tar.xz /usr/share/gazebo-11/models.tar.xz
RUN cd /gzweb/http/client && \
    rm -rf /gzweb/http/client/assets && \
    tar -xvf assets.tar.xz && \
    rm assets.tar.xz && \
    cd /usr/share/gazebo-11 && \
    rm -rf /usr/share/gazebo-11/models && \
    tar -xvf models.tar.xz && \
    rm models.tar.xz




######################
#   nginx config
######################
COPY /foxGlove.conf /etc/nginx/conf.d

# ports and volumes
EXPOSE 8443
