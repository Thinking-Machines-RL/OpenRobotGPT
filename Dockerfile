# Use the nvidia/cuda image as base
FROM nvidia/cuda:12.4.1-base-ubuntu22.04

RUN apt-get update && apt-get install -qq -y --no-install-recommends \
        curl \
        gnupg2 \
        lsb-release \
        git \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 sources
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Setup keys
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV DEBIAN_FRONTEND noninteractive

# Specify ROS distro
ENV ROS_DISTRO iron

# Update the package repository and install the required packages
RUN apt-get update
RUN apt-get install -qq -y --no-install-recommends \
  ros-$ROS_DISTRO-desktop \
  python3-rosdep

# Initialize rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

# Source distro
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# Install MoveIt
RUN apt-get install -y ros-$ROS_DISTRO-moveit
RUN apt-get install -y ros-${ROS_DISTRO}-moveit-py

# Install pip
RUN apt-get install -y python3-pip

# Install colcon
RUN pip install -U colcon-common-extensions

# Install Python dependencies from requirements.txt
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Install x11 for graphic forwarding
RUN apt-get update && \
    apt-get install -y \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install SSH server
RUN apt-get update && apt-get install -y openssh-server && \
    mkdir /var/run/sshd && \
    echo 'root:password' | chpasswd && \
    sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/' /etc/ssh/sshd_config && \
    # Permit X11 forwarding
    echo "X11Forwarding yes" >> /etc/ssh/sshd_config && \
    echo "X11DisplayOffset 10" >> /etc/ssh/sshd_config && \
    # Avoid locale issues
    echo "export LANG=C.UTF-8" >> /etc/profile && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /root

ENV API_KEY_PATH=/root/workspace/secrets/api_key.json

# Clone the ASRSE3_CoRL20 repository
RUN git clone https://github.com/pointW/asrse3_corl20.git /root/asrse3_corl20

# Expose SSH port
EXPOSE 22

# Copy the workspace directory
COPY workspace /root/workspace

# Add an entrypoint
ENTRYPOINT ["/root/workspace/setup.sh"]
