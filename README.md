# OpenRobotGPT
Open source implementation of RobotGPT

## How to run
Create an image from the dockerfile

```bash
docker build -t <image_name> .
```

Run the docker with gpus, ssh port enabled and x11 forwarding
```bash
docker run --gpus all -it --rm \
    -p 2222:22 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd)/workspace:/root/workspace:rw \
    <image_name>
```

(From another command line) Connect to the docker
```bash
ssh -X -p 2222 root@localhost
```

The password is "password"

Every time you need to reconnect to the container the SHA key will change. You can fix the error that pops up with this bash code
```bash
ssh-keygen -f "/home/nicola/.ssh/known_hosts" -R "[localhost]:2222"
```

In order to be able to connect from an Ubuntu machine use the following command
```bash
xhost +
```