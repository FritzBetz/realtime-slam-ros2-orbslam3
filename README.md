# realtime-ros2-orbslam3
⁂   ⁂   ⁂
**No GPS? No problem!**
A compact ROS 2 Humble node for Ubuntu 22.04 that subscribes to an RGB camera and publishes a `tf` transform based on [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3).
Ideal for robotics, drones, and any system needing robust visual localization without GPS. I will make a Raspberry Pi 5 version and install guide soon.

---

## Features

- **Pure Visual SLAM**: Accurate localization using only a monocular RGB camera.
- **ROS 2 Humble Support**: Designed for ROS 2 Humble on Ubuntu 22.04.
- **TF Publishing**: Publishes real-time camera pose as a transform for easy integration with your robot stack.
- **Compact \& Fast**: Lightweight node for real-time performance.
- **Easy Integration**: Drop into any ROS 2 workspace.

---

## Quick Start

### 1. Clone into Your ROS 2 Workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/FritzBetz/realtime-slam-ros2-orbslam3.git
```


### 2. Install Dependencies

Follow the comprehensive [install guide](https://github.com/FritzBetz/ORB-SLAM3-update) for ORB-SLAM3 and dependencies (OpenCV, Pangolin, Eigen3, etc.).
**Note:** This package is tested with **ROS 2 Humble** on **Ubuntu 22.04**.

### 3. Build with Colcon (Symlink Install Recommended)

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select orbslam_cpp
```

- The `--symlink-install` option lets you edit source files without rebuilding the whole workspace.


### 4. Source Your Workspace

```bash
source ~/ros2_ws/install/setup.bash
```


### 5. Run the Node

```bash
ros2 run orbslam_cpp orbslam3_localization
```


---

## About ORB-SLAM3

[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) is a state-of-the-art Visual SLAM system that supports monocular, stereo, and RGB-D cameras.
It provides real-time, accurate pose estimation and mapping, and is widely used in robotics and computer vision research.

This ROS 2 wrapper brings the power of ORB-SLAM3 to modern ROS 2 systems, making it easy to integrate advanced visual localization into your robot.

---

## Configuration

- **Camera Parameters**: Place your camera calibration YAML and vocabulary files in the appropriate location (see install guide).
- **TF Frames**: The node publishes the camera pose as a TF transform for easy consumption by other ROS 2 nodes.

---

## Requirements

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **ORB-SLAM3** (see [install guide](https://github.com/FritzBetz/ORB-SLAM3-update))
- **OpenCV**, **Pangolin**, **Eigen3**, and other dependencies as listed in the guide.

---

## Useful Links

- [ORB-SLAM3 Official Repository](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Comprehensive Install Guide](https://github.com/FritzBetz/ORB-SLAM3-update)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)

---

## Example Launch

```bash
ros2 run orbslam_cpp orbslam3_localization
```

- Make sure your camera topic is publishing images and your config files are correctly set.

---

## License

This project is distributed under the GNU GENERAL PUBLIC License. See [LICENSE](LICENSE) for details.

---

**Enjoy robust, real-time visual localization with ROS 2 and ORB-SLAM3!**
For questions, issues, or contributions, please open an issue or pull request on GitHub.

⁂   ⁂   ⁂

---
