#!/bin/bash

# android build configuration
export PYTHON3_EXEC=/usr/bin/python3
export ANDROID_STL=c++_static
export ANDROID_TOOLCHAIN=clang

${PYTHON3_EXEC} generate_package_xml.py base.yml

sed -i "s/libssl-dev//g" src/eProsima/Fast-DDS/package.xml
sed -i "s/ rt//g" src/eProsima/Fast-DDS/cmake/modules/FindThirdpartyBoost.cmake

touch \
  src/ros2/examples/rclpy/COLCON_IGNORE \
  src/ros2/rcl/rcl/test/COLCON_IGNORE \
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE \
  src/ros2/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_launch/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_read/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_test/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_trace/COLCON_IGNORE

colcon build \
    --merge-install \
    --cmake-args \
        --no-warn-unused-cli \
        -DPYTHON_EXECUTABLE=${PYTHON3_EXEC} \
        -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
        -DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL} \
        -DANDROID_TOOLCHAIN=${ANDROID_TOOLCHAIN} \
        -DANDROID_ABI=${ANDROID_ABI} \
        -DANDROID_NDK=${ANDROID_NDK} \
        -DANDROID_STL=${ANDROID_STL} \
        -DTHIRDPARTY=ON \
        -DCOMPILE_EXAMPLES=OFF \
        -DBUILD_TESTING:BOOL=OFF \
        -DBUILD_MEMORY_TESTS=OFF \
        -DBUILD_MEMORY_EXAMPLES=OFF \
        -DFOONATHAN_MEMORY_BUILD_EXAMPLES=OFF\
        -DFOONATHAN_MEMORY_BUILD_TESTS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_FIND_ROOT_PATH="$PWD/install"