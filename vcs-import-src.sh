#!/bin/bash

set -e
set -x

vcs-import src < ros2.repos
vcs-import src < ros2-for-arm.repos