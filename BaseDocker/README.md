Official Documentation on running the Docker Implementation of ECE4600-G13

By: Edcel Abanto

Assuming you have Docker already installed and know the basics.

Download the following files and put it into any directory you want to work in. 

* Note: in the following statements with "<name>" you only need to put --name-- not the "<>"
* Note: -> refers to the code you are supposed to write in terminal

First, to build the image from the Dockerfile.

-> docker build -t <image-name>:<tag> .

* Note remember the name of the image and the tag eg. ece4600-image:latest 

Let it build the image :)

Once image is built we can start the container:

-> ./start_container.sh $1 $2
$1 refers to container name <container-name>
$2 refers to the image name and tag <image-name>:<tag> 

Once that is done, it will place you into the Docker container, and into the /uros_ws directory.
You can then run the run_setup script to start the microROS agent and build the gesture node.

-> ./run_setup.sh

This will do start the microROS agent and buildthe gesture node. This is the purpose of this terminal.

You can then open up another terminal, and run the followin code to go into the same container.

-> docker exec -it <container-name> bash

Once inside do the following to run the gesture node, ensure you are still in /uros_ws directory.

-> source install/setup.bash
-> ros2 run gesture_node_pkg gesture_node
