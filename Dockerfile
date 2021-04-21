FROM ros:foxy AS toolchain

COPY ./sources.list /etc/apt/
COPY ./ros2-latest.list /etc/apt/sources.list.d/

RUN apt-get update && apt-get install -y \
    g++-aarch64-linux-gnu \
    gcc-aarch64-linux-gnu \
    wget

FROM toolchain AS manifest

WORKDIR /ros2_ws

COPY ./ros2.repos .
COPY ./ros2-for-arm.repos .
COPY ./aarch64_toolchainfile.cmake .

FROM manifest AS source

RUN mkdir src
RUN vcs-import src < ros2.repos
RUN vcs-import src < ros2-for-arm.repos

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
  src/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE

RUN export CROSS_COMPILE=arm-linux-gnueabi- \
    && colcon build \
        --symlink-install \
        --cmake-force-configure \
        --cmake-args \
            --no-warn-unused-cli \
            -DCMAKE_TOOLCHAIN_FILE=`pwd`/aarch64_toolchainfile.cmake \
            -DTHIRDPARTY=ON \
            -DBUILD_TESTING:BOOL=OFF \
            -DCMAKE_BUILD_RPATH="`pwd`/build/poco_vendor/poco_external_project_install/lib/;`pwd`/build/libyaml_vendor/libyaml_install/lib/"