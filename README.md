# SLAM Project

Visual SLAM using NVIDIA Isaac ROS (cuVSLAM) with ZED X Mini stereo camera on ROS 2 Humble.

Configured for real-time performance with BEST_EFFORT QoS.

## Requirements

- Ubuntu 22.04
- NVIDIA Jetson or GPU with CUDA support
- ROS 2 Humble
- just

  ```sh
  cargo install just
  ```

You can optionally use ZED cameras in the project. To enable it,
- Download and install ZED SDK 5.0.7+ from [here](https://www.stereolabs.com/en-tw/developers/release/5.0).
- Pull the ZED ROS wrapper source code

  ```sh
  git submodule update --init --recursive src/zed-ros2-wrapper/
  ```


## Installation

```bash
# Setup system (installs ROS, dependencies, downloads assets)
just setup

# Build workspace
just build
```

## Quick Start

### Live Camera

Run visual SLAM with ZED X Mini and record data:

```bash
# Terminal 1: Record rosbag
ros2 bag record $(cat docs/record_topics.txt)

# Terminal 2: Launch ZED X Mini + Visual SLAM
just launch-zedxm
```

### Replay Recorded Data

Play back a rosbag for testing or analysis:

```bash
# Terminal 1: Launch Visual SLAM only
just launch

# Terminal 2: Replay rosbag
ros2 bag play \
    --topics $(cat docs/replay_topics.txt) \
    rosbag2_2025_11_05-16_54_25/
```

Download example rosbag [here](https://newslabn.csie.ntu.edu.tw/drive/d/s/15gzknb3bTyhsyzhOAgokJcroCHYvIIw/ijNJaQ2II8Iy2B8faVji1QhMmmr3Yp3p-WL3g69Gctww)

## Verifying SLAM Operation

### Check Data Flow

```bash
# Camera images (should show ~30 Hz)
ros2 topic hz /zedxm/zed_node/left_gray/image_rect_gray

# SLAM odometry (should show ~30 Hz)
ros2 topic hz /visual_slam/tracking/odometry
```

### Check Topic Connections

```bash
# Verify camera_info subscription
ros2 topic info /zedxm/zed_node/left_gray/camera_info -v

# Verify Visual SLAM is receiving data
ros2 node info /visual_slam_node | grep camera_info
```

### Check QoS Configuration

All topics should use BEST_EFFORT reliability:

```bash
# Camera topics
ros2 topic info /zedxm/zed_node/left_gray/image_rect_gray -v | grep Reliability

# Converted images
ros2 topic info /left/image_rect -v | grep Reliability
```

## Expected Output

## ZED X Mini Integration - Critical Steps

### 1. Image Format

Isaac ROS Visual SLAM requires mono8 grayscale images. Use gray image topics:
```
/zedxm/zed_node/left_gray/image_rect_gray
/zedxm/zed_node/right_gray/image_rect_gray
```

### 2. Topic Remapping

Camera_info must follow image_pipeline convention (same namespace as images):
```xml
<remap from="/visual_slam/camera_info_0" to="/zedxm/zed_node/left_gray/camera_info"/>
<remap from="/visual_slam/camera_info_1" to="/zedxm/zed_node/right_gray/camera_info"/>
```

### 3. QoS Configuration

All components must use BEST_EFFORT QoS for real-time performance:

**ZED Camera** - Modified source code:
```cpp
// src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_main.cpp:69
mQos(rclcpp::QoS(QOS_QUEUE_SIZE).best_effort())
```

**ImageFormatConverter** - Launch parameters:
```xml
<param name="input_qos" value="SENSOR_DATA"/>
<param name="output_qos" value="SENSOR_DATA"/>
```

**Visual SLAM** - Uses BEST_EFFORT by default (no changes needed).

### 4. Image Pipeline

```
ZED Camera (mono8, BEST_EFFORT)
    → ImageFormatConverter (nitros_image_mono8, BEST_EFFORT)
    → Visual SLAM (BEST_EFFORT subscriber)
```

All nodes run in same composable container for zero-copy NITROS communication.

### 5. Network Configuration

Required for reliable DDS with large messages:

```bash
# Enable multicast on loopback
sudo ip link set lo multicast on

# Set kernel parameters
echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.conf
echo "net.ipv4.ipfrag_time=3" | sudo tee -a /etc/sysctl.conf
echo "net.ipv4.ipfrag_high_thresh=134217728" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

See `docs/ZEDXM_SETUP.md` for details.

## Available Commands

```bash
just                  # List all commands
just setup            # Setup system (one-time)
just build            # Build workspace
just clean            # Remove build artifacts
just launch-zedxm     # Launch ZED X Mini + SLAM
just play-zedxm       # Launch with detailed logging
just launch           # Launch with rosbag
just play-bag         # Play rosbag data
```

## Troubleshooting

### ZED Camera Video Not Shown in RViz

It can be caused by hardware connectivity or driver configuration issues. Run this command to check the camera availability and driver status on the system.

```sh
ZED_Diagnostic
```

If the camera is detected by the diagnostic system, run the ZED tool to check if can read stereo videos from the camera.

```sh
ZED_Explorer
```

### SLAM Not Publishing Odometry

Check all initialization requirements are met:
1. Stereo images publishing at same rate
2. Camera_info in correct namespaces (`left_gray/camera_info`)
3. QoS matching (all BEST_EFFORT)
4. Sufficient visual features in scene
5. Camera motion (rotation + translation)

Verify:
```bash
ros2 topic hz /left/image_rect /right/image_rect
ros2 topic info /zedxm/zed_node/left_gray/camera_info -v
ros2 node info /visual_slam_node | grep camera_info
```

### QoS Mismatch

Symptom: Topics advertised but no data flowing.

Check reliability settings:
```bash
ros2 topic info <topic_name> -v | grep Reliability
```

Publisher and subscriber must both use BEST_EFFORT.

### Network Issues

If seeing message drops or latency:
```bash
# Check multicast
ip link show lo | grep MULTICAST

# Check kernel parameters
sysctl net.core.rmem_max
sysctl net.ipv4.ipfrag_high_thresh
```

## Project Structure

```
slam/
├── justfile                 # Build automation
├── .envrc                   # ROS environment (direnv)
├── cyclonedds.xml           # DDS configuration
├── src/
│   ├── slam_launch/        # Launch files, configs, RViz
│   └── zed-ros2-wrapper/   # ZED SDK wrapper (modified)
├── docs/                    # Setup guides
└── assets/                  # SLAM test data
```

## Documentation

- `docs/ZEDXM_SETUP.md` - ZED X Mini setup guide
- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/

## License

MIT
