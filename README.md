# OpenRobotGPT
Open source implementation of RobotGPT

## How to run
Create an image from the dockerfile

```bash
docker build -t <image_name> .
```

Run the container
```bash
docker run --gpus all -it \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    <image_name>
```
