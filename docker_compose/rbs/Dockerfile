FROM ros:remote_bot_system

# # set version label
# ARG BUILD_DATE
# ARG VERSION
# ARG CODE_RELEASE
# LABEL build_version="Linuxserver.io version:- ${VERSION} Build-date:- ${BUILD_DATE}"
# LABEL maintainer="aptalca"

# # environment settings
# ARG DEBIAN_FRONTEND="noninteractive"
# ENV HOME="/config"
# ENV LANG C.UTF-8
# ENV LC_ALL C.UTF-8

COPY /remote_bot_system /
COPY /vrpn /

RUN \
# ftp mount dir
    mkdir -p /mnt/ftp && \
# frontend
    cd /remote_bot_system/frontend && \
    npm install && \
# backend
    cd /remote_bot_system/backend && \
    python3 -m pip install -r requirements.txt && \
# vrpn
    mkdir -p /vrpn/build && \
    cd /vrpn/build && \
    cmake .. && \
    make && \
    make install && \
    cd /remote_bot_system/vrpn_listener_ws && \
    /bin/bash -c 'source /usr/share/gazebo/setup.sh; colcon build'