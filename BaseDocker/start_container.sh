#!/usr/bin/bash

docker run \
    -it \
    --rm \
    --name $1 \
    --net=host \
    -e DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -v $(pwd):/workspace \ # This workspace is for already built components if you don't want to colcon buildin uros_ws
    ece4600-base:2 
    #udp4 \
    #--port 8888
