#   Use the uROS Agent Image
FROM ros:humble

#   Update 
RUN apt update 

#   Install 
RUN apt-get install -y \ 
#   Utilities 
    vim \
    htop \
    xterm \
    python3 \
    python3-pip \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-vcstool \
    wget \
# rqt-graph
    ros-humble-rqt-graph \ 
# Remove temporart packages 
    && rm -rf /var/lib/apt/lists/*

#   Build and source the workspace for ROS2 -> could be deleted will need to check if needed since it will be overwritten 
WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \ 
    #rosdep install --from-paths src --ignore-src -y && \
    colcon build 

#   Setup the uROS workspace for uROS agent but also for other packages 
RUN mkdir -p /uros_ws 
WORKDIR /uros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Update 
RUN apt update && rosdep update
RUN rosdep install --from-paths src --ignore-src -y 

# Copy the script needed to run: colcon build, setup and run uROS agent  
COPY run_setup.sh /uros_ws/run_setup.sh
RUN chmod +x /uros_ws/run_setup.sh

# Copy the packages 
COPY src/gesture_node_pkg /uros_ws/src/gesture_node_pkg 

CMD ["/bin/bash"]
