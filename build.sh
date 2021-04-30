#!/bin/bash

# android build configuration
export PYTHON3_EXEC=/usr/bin/python3
export ANDROID_STL=c++_static
export ANDROID_TOOLCHAIN=clang

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
        -DBUILD_MEMORY_TOOLS=OFF \
        -DCMAKE_FIND_ROOT_PATH="$PWD/install"