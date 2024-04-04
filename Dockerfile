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

# Install pip
RUN apt-get install -y python3-pip

# Install Python dependencies from requirements.txt
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Install x11 for graphic forwardingCannot call `env.render()` before calling `env.reset()`, if this is a intended action, set `disable_render_order_enforcing=True` on the OrderEnforcer wrapper.
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

# Expose SSH port
EXPOSE 22

# Add an entrypoint
ENTRYPOINT ["/root/workspace/setup.sh"]