# /bin/bash

# Install gym_example package
cd workspace
pip3 install -e gym_example
cd ..

# Open up ssh server
/usr/sbin/sshd -D