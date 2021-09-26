FROM ros:noetic-ros-base-focal

WORKDIR /root/workspace/uav_ros

RUN apt-get update && apt-get install wget build-essential ros-desktop-full ros-noetic-mavros-* -y

RUN wget https://github.com/premake/premake-core/releases/download/v5.0.0-alpha16/premake-5.0.0-alpha16-linux.tar.gz
RUN tar -xf premake-5.0.0-alpha16-linux.tar.gz
RUN cp premake5 /usr/local/bin
RUN rm premake*

ENV ROS_PACKAGE_PATH=/root/workspace:/opt/ros/noetic/share

COPY . .

