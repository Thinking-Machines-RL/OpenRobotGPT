#!/bin/bash

# Add localhost:22 to list of known hosts, ssh and colcon build
gnome-terminal --wait --bash -c "
  ssh-keygen -f '/home/$USER/.ssh/known_hosts' -R '[localhost]:2222' &&
  expect -c '
    spawn ssh -X -p 2222 root@localhost
    expect \"Are you sure you want to continue connecting (yes/no/[fingerprint])?\" {
      send \"yes\r\"
      expect \"root@localhost's password\"
      send \"password\r\"
      interact
    }
  ';
  cd workspace/ros2_ws;
  colcon build;
  bash;
"

# Start GymNode shell
gnome-terminal --tab --wait --bash -c "expect -c '
    spawn ssh -X -p 2222 root@localhost
    expect \"Are you sure you want to continue connecting (yes/no/[fingerprint])?\" {
      send \"yes\r\"
      expect \"root@localhost's password\"
      send \"password\r\"
      interact
    }
  ';
  cd workspace/ros2_ws;
  source ./install/setup.sh;
  ros2 run panda_env GymNode;
  bash;
"

# Start MoveNodebasic shell
gnome-terminal --tab --wait --bash -c "expect -c '
    spawn ssh -X -p 2222 root@localhost
    expect \"Are you sure you want to continue connecting (yes/no/[fingerprint])?\" {
      send \"yes\r\"
      expect \"root@localhost's password\"
      send \"password\r\"
      interact
    }
  ';
  cd workspace/ros2_ws;
  source ./install/setup.sh;
  ros2 run panda_env MoveNodebasic;
  bash;
"

# Start robot_api_node shell
gnome-terminal --tab --wait --bash -c "expect -c '
    spawn ssh -X -p 2222 root@localhost
    expect \"Are you sure you want to continue connecting (yes/no/[fingerprint])?\" {
      send \"yes\r\"
      expect \"root@localhost's password\"
      send \"password\r\"
      interact
    }
  ';
  cd workspace/ros2_ws;
  source ./install/setup.sh;
  ros2 run robot_api_layer robot_api_node;
  bash;
"

# Start bot_node shell
gnome-terminal --tab --wait --bash -c "expect -c '
    spawn ssh -X -p 2222 root@localhost
    expect \"Are you sure you want to continue connecting (yes/no/[fingerprint])?\" {
      send \"yes\r\"
      expect \"root@localhost's password\"
      send \"password\r\"
      interact
    }
  ';
  cd workspace/ros2_ws;
  source ./install/setup.sh;
  ros2 run code_bot bot_node;
  bash;
"

