#!/bin/bash

# Install gym_example package
cd /root/workspace/ros2_ws/src/panda_env/panda_env
pip3 install -e gym_example
cd ..

# Open up ssh server
/usr/sbin/sshd -D