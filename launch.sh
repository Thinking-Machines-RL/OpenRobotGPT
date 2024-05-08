#!/bin/bash

gnome-terminal --wait -- bash -c "
echo \"ssh\"
ssh-keygen -f '/home/$USER/.ssh/known_hosts' -R '[localhost]:2222'
ssh -X -p 2222 root@localhost << EOF
cd workspace/ros2_ws;
colcon build;
EOF;
"

gnome-terminal -- bash -c "
echo \"GymNode\"
ssh -X -p 2222 root@localhost << EOF
echo \"Succesful ssh\"
cd workspace/ros2_ws;
source ./install/setup.sh;
bash -i
ros2 run panda_env GymNode;
EOF;
"

gnome-terminal -- bash -c "
echo \"MoveNodeBasic\"
ssh -X -p 2222 root@localhost << EOF
echo \"Succesful ssh\"
cd workspace/ros2_ws;
source ./install/setup.sh;
bash -i
ros2 run panda_env MoveNodeBasic;
EOF;
"

gnome-terminal -- bash -c "
echo \"robot_api_node\"
ssh -X -p 2222 root@localhost << EOF
echo \"Succesful ssh\"
cd workspace/ros2_ws;
source ./install/setup.sh;
bash -i
ros2 run robot_api_layer robot_api_node;
EOF;
"

gnome-terminal -- bash -c "
echo \"test_node\"
ssh -X -p 2222 root@localhost << EOF
echo \"Succesful ssh\"
cd workspace/ros2_ws;
source ./install/setup.sh;
bash -i
ros2 run code_bot test_node;
EOF;
"


