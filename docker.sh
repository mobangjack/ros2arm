#!/bin/bash

set -e

stamp=$(date +%Y%m%d%H%M%S)
docker build . -t ros2arm:$stamp
docker tag ros2arm:$stamp ros2arm:latest