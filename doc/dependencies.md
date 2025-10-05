# Dependencies for ur_cartesian_controller

## System Dependencies

### Required packages to install:

```bash
# OSQP QP solver and Eigen interface
sudo apt install libosqp-dev
sudo apt install libeigen3-dev

# OsqpEigen (C++ wrapper)
# Install from source if not available via apt:
cd /tmp
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
sudo make install
```

## ROS2 Dependencies

All ROS2 dependencies are declared in `package.xml`:

- `rclcpp` - ROS2 C++ client library
- `sensor_msgs` - Joint state messages
- `geometry_msgs` - Point and Pose messages
- `std_msgs` - Float64MultiArray for velocity commands
- `kdl_parser` - URDF to KDL conversion
- `orocos_kdl` - Kinematics library for FK/Jacobian
- `eigen` - Linear algebra
- `osqp` - QP solver backend
- `osqp_vendor` - ROS2 wrapper for OSQP

## Install all ROS2 dependencies:

```bash
cd ~/10x_ws
rosdep update
rosdep install --from-paths src/ur_cartesian_controller --ignore-src -y
```

## Verify dependencies are installed:

```bash
# Check OSQP
pkg-config --modversion osqp

# Check OsqpEigen headers
ls /usr/include/OsqpEigen/ || ls /usr/local/include/OsqpEigen/

# Check Eigen
pkg-config --modversion eigen3
```
