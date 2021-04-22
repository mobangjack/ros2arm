FROM ros:foxy AS toolchain

RUN apt-get update && apt-get install -y \
    wget \
    unzip

RUN wget -q -nv https://dl.google.com/android/repository/android-ndk-r22b-linux-x86_64.zip
RUN unzip -qq android-ndk-r22b-linux-x86_64.zip

ENV ANDROID_NDK=/android-ndk-r22b

FROM toolchain AS manifest

WORKDIR /ros2_ws

# RUN wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
COPY ./ros2.repos .

FROM manifest AS source

RUN mkdir src
RUN vcs-import src < ros2.repos

FROM source AS build

RUN touch \
  src/ros-perception/laser_geometry/COLCON_IGNORE \
  src/ros/resource_retriever/COLCON_IGNORE \
  src/ros2/geometry2/COLCON_IGNORE \
  src/ros2/urdf/COLCON_IGNORE \
  src/ros2/demos/COLCON_IGNORE \
  src/ros2/kdl_parser/COLCON_IGNORE \
  src/ros2/ros1_bridge/COLCON_IGNORE \
  src/ros2/rmw_connext/COLCON_IGNORE \
  src/ros2/orocos_kinematics_dynamics/COLCON_IGNORE \
  src/ros2/examples/rclpy/COLCON_IGNORE \
  src/ros2/robot_state_publisher/COLCON_IGNORE \
  src/ros2/rviz/COLCON_IGNORE \
  src/ros2/rcl/rcl/test/COLCON_IGNORE \
  src/ros2/urdfdom/COLCON_IGNORE \
  src/ros2/rclpy/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_opensplice/COLCON_IGNORE \
  src/ros2/system_tests/COLCON_IGNORE \
  src/ros2/rosidl_python/COLCON_IGNORE \
  src/ros2/rmw_opensplice/COLCON_IGNORE \
  src/ros2/rosidl_typesupport_connext/COLCON_IGNORE \
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE \
  src/osrf/osrf_testing_tools_cpp/COLCON_IGNORE

# disable SECURITY
RUN sed -i '/option(SECURITY/a\set(SECURITY\ OFF)' src/eProsima/Fast-RTPS/CMakeLists.txt

# fix ld: error: unable to find library -lpthread
RUN sed -i '/set(SECURITY/a\find_package(Threads)' src/eProsima/Fast-RTPS/CMakeLists.txt

# android build configuration
ARG PYTHON3_EXEC=/usr/bin/python3
ENV ANDROID_ABI=armeabi-v7a
ENV ANDROID_NATIVE_API_LEVEL=android-21
ENV ANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang

RUN colcon build \
    --cmake-force-configure \
    --cmake-args \
        --no-warn-unused-cli \
        -DPYTHON_EXECUTABLE=${PYTHON3_EXEC} \
        -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
        -DANDROID_FUNCTION_LEVEL_LINKING=OFF \
        -DANDROID_NATIVE_API_LEVEL=${ANDROID_NATIVE_API_LEVEL} \
        -DANDROID_TOOLCHAIN_NAME=${ANDROID_TOOLCHAIN_NAME} \
        -DANDROID_STL=c++_shared \
        -DANDROID_ABI=${ANDROID_ABI} \
        -DANDROID_NDK=${ANDROID_NDK} \
        -DTHIRDPARTY=ON \
        -DCOMPILE_EXAMPLES=ON \
        -DBUILD_TESTING:BOOL=OFF \
        -DCMAKE_FIND_ROOT_PATH="$PWD/install"