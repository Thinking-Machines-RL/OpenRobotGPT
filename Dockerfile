# Use the nvidia/cuda image as base
FROM nvidia/cuda:12.3.2-base-ubuntu20.04

RUN apt-get update && apt-get install -y --no-install-recommends \
        mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# Setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list

# Setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV DEBIAN_FRONTEND noninteractive

# Update the package repository and install the required packages
RUN apt-get update
RUN apt-get install -y \
  ros-noetic-desktop-full \
  python3-rosdep

# Initialize rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

# Specify ROS distro
ENV ROS_DISTRO noetic

# Source distro
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Install MoveIt
RUN apt-get install -y ros-noetic-moveit

# Install PyBullet
RUN apt-get install -y python3-pip
RUN pip3 install pybullet

# Install x11 for graphic forwarding
RUN apt-get update && \
    apt-get install -y \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install VSCode Server for Remote Development
RUN curl -fsSL https://code-server.dev/install.sh | sh

# Expose code-server port
EXPOSE 8080

# Start code-server
CMD ["code-server", "--host", "0.0.0.0", "--port", "8080", "--auth", "none", "--disable-telemetry"]