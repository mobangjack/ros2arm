## Setup the development environment
The normal setup described in the following link is required for the cross-compilation -> https://github.com/ros2/ros2/wiki/Linux-Development-Setup


## Get the source
Create a workspace and clone all repos:

```
mkdir -p ros2_ws/src
cd ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
wget https://raw.githubusercontent.com/ros2-for-arm/ros2/master/ros2-for-arm.repos
wget https://raw.githubusercontent.com/ros2-for-arm/ros2/master/aarch64_toolchainfile.cmake
vcs-import src < ros2.repos
vcs-import src < ros2-for-arm.repos
```


## Get an aarch64 toolchain and export it

```
sudo apt install g++-aarch64-linux-gnu gcc-aarch64-linux-gnu

# If you are not using a custom toolchain
export PATH="<path to toolchain>/bin:$PATH"
export CROSS_COMPILE=aarch64-linux-gnu-
```


## Remove Python support and ignore optional packages
Packages that depend on Python or other libraries that are not part of the package itself are ignored.
The following command disables the build of such packages by adding an empty file called [COLCON_IGNORE](https://github.com/ros2/ros2/wiki/Colcon-Tutorial) in their base directory:

```bash
touch \
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
```

Note: If you require any of these packages, you can still cross-compile them if you pre-build their dependencies.
You can set _CMAKE_SYSROOT_ pointing to a root filesystem containing the pre-built dependencies. For more information, see [CMAKE_SYSROOT documentation](https://cmake.org/cmake/help/latest/variable/CMAKE_SYSROOT.html)


## Trigger a build
```bash
colcon build \
  --symlink-install \
  --cmake-force-configure \
  --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_TOOLCHAIN_FILE=`pwd`/aarch64_toolchainfile.cmake \
    -DTHIRDPARTY=ON \
    -DBUILD_TESTING:BOOL=OFF \
    -DCMAKE_BUILD_RPATH="`pwd`/build/poco_vendor/poco_external_project_install/lib/;`pwd`/build/libyaml_vendor/libyaml_install/lib/"
```


## Installation
Once the compilation is done you will have to move the generated libraries (`install/<Package Name>/lib`) in your target filesystem.
Get all the libraries using `cp` `` `find . -name "*.so*"` `` `<destination_directory>`.
On your target, place the libraries into the /lib directory (default search path used when a binary is launched).
Alternatively, set the `LD_LIBRARY_PATH` environment variable to the directory containing the libraries.
