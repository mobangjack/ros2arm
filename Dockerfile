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
COPY ./ros2.yml .

FROM manifest AS source

RUN mkdir src
RUN vcs-import src < ros2.yml

FROM source AS build

RUN touch \
  src/ros2/examples/rclpy/COLCON_IGNORE \
  src/ros2/rcl/rcl/test/COLCON_IGNORE \
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE

# android build configuration
ARG PYTHON3_EXEC=/usr/bin/python3
ARG ANDROID_ABI=armeabi-v7a
ARG ANDROID_STL=c++_static
ARG ANDROID_NATIVE_API_LEVEL=23
ARG ANDROID_TOOLCHAIN=clang

RUN colcon build \
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
        -DCMAKE_FIND_ROOT_PATH="$PWD/install"