#usr/bin/bash
#xhost +local:docker

docker run \
    -it \
    --rm \
    --name $1 \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -v $(pwd):/workspace \
    ece4600-base:$2 

# This workspace is for already built components if you don't want to colcon buildin uros_wsece4600-base
# For RVIZ 
    #-e DISPLAY=$DISPLAY \
    #-e QT_X11_NO_MITSHM=1 \
    #-v /tmp/.X11-unix/tmp/.X11-unix \

