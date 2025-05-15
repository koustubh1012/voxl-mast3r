#!/bin/bash
docker run -it   --user $(id -u):$(id -g)   --network=host   --ipc=host   --pid=host   -e DISPLAY=$DISPLAY   -e ROS_DOMAIN_ID=0   -e ROS_LOCALHOST_ONLY=0   -v /tmp/.X11-unix:/tmp/.X11-unix   --name voxl_humble   voxl-ros2-humble:latest
