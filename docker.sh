#!/bin/bash

set -e
set -x

docker build . -t ros2arm:latest
# docker run -v $pwd:$pwd ros2arm:latest bash -c "cp -r /ros2_ws/install $pwd"