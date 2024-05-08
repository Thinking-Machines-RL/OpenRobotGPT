#!/bin/bash

gnome-terminal --wait -- bash -c "
echo \"Let's try this\"
ssh-keygen -f '/home/$USER/.ssh/known_hosts' -R '[localhost]:2222'
ssh -X -p 2222 root@localhost << EOF
cd workspace/ros2_ws;
colcon build;
EOF;
"

echo "Finished first part"

gnome-terminal --wait -- bash -c "
ssh -X -p 2222 root@localhost << EOF
cd workspace/ros2_ws;
source ./install/setup.sh;
ros2 run panda_env GymNode;
EOF;
"


