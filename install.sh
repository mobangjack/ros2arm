#!/bin/bash

docker run -v $pwd:$pwd ros2arm:latest bash -c "cp -r /ros2_ws/install $pwd"