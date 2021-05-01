#!/bin/bash

# android build configuration
export PYTHON3_EXEC=/usr/bin/python3
export ANDROID_STL=c++_static
export ANDROID_TOOLCHAIN=clang

${PYTHON3_EXEC} generate_package_xml.py base.yml

sed_append() {
  ref="$1"
  line="$2"
  file="$3"
  grep "$line" "$file"
  if [ $? -ne 0 ]; then
    sed -i "/${ref}/a\\${line}" "$file"
  fi
}

sed_append "FOONATHAN_MEMORY" "  <depend>foonathan_memory</depend>" src/eProsima/foonathan_memory_vendor/package.xml
sed_append "option(SECURITY" "set(SECURITY OFF)" src/eProsima/Fast-DDS/CMakeLists.txt
sed -i 's/file(WRITE  \${CMAKE_CURRENT_BINARY_DIR}\/container_node_sizes_impl.hpp "#define FOONATHAN_MEMORY_NO_NODE_SIZE")/file(COPY \${CMAKE_CURRENT_SOURCE_DIR}\/..\/include\/foonathan\/memory\/detail\/container_node_sizes_impl.hpp DESTINATION \${CMAKE_CURRENT_BINARY_DIR})/g' \
  src/foonathan/foonathan_memory/src/CMakeLists.txt
# sed -i "s/libssl-dev//g" src/eProsima/Fast-DDS/package.xml
sed -i "s/ rt//g" src/eProsima/Fast-DDS/cmake/modules/FindThirdpartyBoost.cmake
sed -i "s/stdc++fs//g" src/ros/pluginlib/pluginlib/CMakeLists.txt
sed -i "s/stdc++fs//g" src/ros/pluginlib/pluginlib/pluginlib-extras.cmake

cp arch/${ANDROID_ABI}/container_node_sizes_impl.hpp \
    src/foonathan/foonathan_memory/include/foonathan/memory/detail/

touch \
  src/ros2/examples/rclpy/COLCON_IGNORE \
  src/ros2/rcl/rcl/test/COLCON_IGNORE \
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE \
  src/ros2/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_launch/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_read/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_test/COLCON_IGNORE \
  src/ros-tracing/ros2_tracing/tracetools_trace/COLCON_IGNORE \
  src/ros2/rosbag2/rosbag2_transport/COLCON_IGNORE \
  src/ros2/rosbag2/ros2bag/COLCON_IGNORE


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
        -DBUILD_MEMORY_TOOLS=OFF \
        -DBUILD_MEMORY_EXAMPLES=OFF \
        -DFOONATHAN_MEMORY_BUILD_TESTS=OFF \
        -DFOONATHAN_MEMORY_BUILD_TOOLS=OFF \
        -DFOONATHAN_MEMORY_BUILD_EXAMPLES=OFF\
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_FIND_ROOT_PATH="$PWD/install"