# Project Context for Claude

## Overview
ROS2 SLAM workspace using NVIDIA Isaac ROS Visual SLAM (cuVSLAM 12.6) with ZED X Mini stereo camera on Ubuntu 22.04 / Jetson platform.

## Project Structure

```
slam/
├── justfile                 # Build automation (replaces Makefile)
├── .envrc                   # Auto-sources ROS Humble + network config checks (direnv)
├── cyclonedds.xml           # Cyclone DDS configuration for reliable data transfer
├── CLAUDE.md                # This file
├── README.md                # Project overview
├── scripts/
│   ├── setup.sh            # System setup script
│   └── download_assets.sh  # Downloads Isaac ROS Visual SLAM assets
├── docs/
│   └── ZEDXM_SETUP.md      # ZED X Mini setup guide (minimalist style)
├── src/
│   ├── slam_launch/        # Custom ROS2 package
│   │   ├── launch/
│   │   │   ├── visual_slam.launch.xml        # Basic visual SLAM (rosbag)
│   │   │   └── zedxm_visual_slam.launch.xml  # ZED X Mini camera
│   │   ├── config/
│   │   │   ├── quickstart_interface_specs.json       # For rosbag
│   │   │   └── zedxm_quickstart_interface_specs.json # ZED X Mini: 960x600
│   │   └── rviz/
│   │       └── isaac_zedxm.rviz              # Custom RViz configuration
│   └── zed-ros2-wrapper/   # ZED SDK ROS2 wrapper (git submodule, modified)
├── assets/                  # Downloaded SLAM assets (rosbag, configs)
├── play_log/                # Detailed logs from play-zedxm runs
└── build/, install/, log/   # ROS2 workspace artifacts
```

## Key Setup Details

### System Requirements
- Ubuntu 22.04
- ROS 2 Humble installed at `/opt/ros/humble`
- Isaac ROS packages (APT): `isaac_ros_visual_slam`, `isaac_ros_image_proc`
- ZED SDK 5.0.7+ (for ZED X Mini support)
- NVIDIA Jetson with CUDA support

### Build System
Uses **Just** (modern Make alternative):
- Minimalist style (no emojis, concise output)
- All recipes source ROS automatically via `.envrc`

### Important Files

**justfile** - Main automation:
- `just setup` - System setup (Ubuntu check, ROS install, dependencies, downloads assets)
- `just build` - Builds workspace (Release mode)
- `just clean` - Removes build artifacts
- `just launch` - Launches visual SLAM with rosbag (auto-starts RViz if DISPLAY set)
- `just launch-zedxm` - Launches ZED X Mini + Visual SLAM (auto-starts RViz)
- `just play` - Plays rosbag with topic remapping
- `just play-zedxm` - Runs launch-zedxm with detailed logging to `play_log/`
- `just rosdep-update` - Updates rosdep database
- `just download-assets` - Downloads SLAM assets (automatically run by setup)

**.envrc** (direnv):
- Sources `/opt/ros/humble/setup.bash` when entering directory
- Sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Sets `CYCLONEDDS_URI` to project's `cyclonedds.xml`
- Validates network configuration (multicast, kernel buffers)
- Shows warnings with documentation links if config missing
- Requires: `direnv allow` after first clone

**cyclonedds.xml**:
- Configures DDS for reliable image/point cloud transfer
- Uses loopback interface (multicast required)
- Large receive buffers (10MB) and high watermarks (500kB)
- Based on Autoware network recommendations

**docs/ZEDXM_SETUP.md**:
- Comprehensive ZED X Mini setup guide
- Network configuration (multicast, kernel params)
- Architecture explanation (ZED → ImageFormatConverter → Visual SLAM)
- Troubleshooting common issues
- Written in minimalist style

## Dependencies

### APT Packages
- `ros-humble-desktop` - ROS 2 Humble with tools
- `ros-dev-tools` - Development tools (colcon, rosdep)
- `isaac_ros_visual_slam` - Visual SLAM node (cuVSLAM)
- `isaac_ros_image_proc` - Image format converter (NITROS)
- ZED SDK from Stereolabs (not in APT)

### Git Submodules
- `zed-ros2-wrapper` - ZED camera ROS2 driver

## Current State

### What Works
- ZED X Mini + Isaac ROS Visual SLAM integration (fully operational)
- Grayscale stereo images at 960x600 @ 15Hz
- NITROS zero-copy communication in composable nodes
- ImageFormatConverter: mono8 → nitros_image_mono8
- Visual SLAM odometry output at ~14.8 Hz
- BEST_EFFORT QoS for real-time performance
- RViz auto-start based on DISPLAY environment
- Network configuration validation on direnv load
- Detailed play logs with timestamps

### Configuration

**ZED X Mini Camera**:
- Resolution: 960x600 @ 15Hz (downscaled 2x from native 1920x1200)
- Topics (following image_pipeline convention):
  - `/zedxm/zed_node/left_gray/image_rect_gray` (mono8)
  - `/zedxm/zed_node/left_gray/camera_info` (calibration)
  - `/zedxm/zed_node/right_gray/image_rect_gray` (mono8)
  - `/zedxm/zed_node/right_gray/camera_info` (calibration)
- Frames:
  - `zedxm_camera_center` (base_frame)
  - `zedxm_left_camera_optical_frame`
  - `zedxm_right_camera_optical_frame`

**Visual SLAM Parameters** (`zedxm_visual_slam.launch.xml`):
- `rectified_images`: True (ZED publishes rectified images)
- `enable_slam_visualization`: True
- `enable_landmarks_view`: True
- `enable_observations_view`: True
- `camera_optical_frames`: List of left/right optical frames
- `base_frame`: Camera center frame for odometry

**Image Pipeline**:
```
ZED Camera (mono8) → ImageFormatConverterNode (nitros_image_mono8) → Visual SLAM
```

All nodes run in same `component_container` for zero-copy NITROS communication.

**QoS Configuration** (BEST_EFFORT for real-time performance):
- **ZED Camera**: Modified source code (`src/zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_main.cpp:69`)
  ```cpp
  mQos(rclcpp::QoS(QOS_QUEUE_SIZE).best_effort())
  ```
  Sets ALL ZED topics (images, camera_info) to BEST_EFFORT

- **ImageFormatConverter**: Launch file parameters (`zedxm_visual_slam.launch.xml:25-26, 36-37`)
  ```xml
  <param name="input_qos" value="SENSOR_DATA"/>
  <param name="output_qos" value="SENSOR_DATA"/>
  ```
  SENSOR_DATA is a ROS 2 preset profile using BEST_EFFORT reliability

- **Visual SLAM**: Subscribes with BEST_EFFORT by default (no configuration needed)

**Topic Remapping** (Critical for initialization):
Camera_info topics MUST follow image_pipeline convention (same namespace as images):
```xml
<remap from="/visual_slam/camera_info_0" to="/zedxm/zed_node/left_gray/camera_info"/>
<remap from="/visual_slam/camera_info_1" to="/zedxm/zed_node/right_gray/camera_info"/>
```

## Important Technical Details

### cuVSLAM Image Format Requirements
**Isaac ROS Visual SLAM requires grayscale (mono8) images.** Color images are NOT supported.

Evidence:
- Official NVIDIA examples disable color cameras (`enable_color: False`)
- RealSense example uses infrared (grayscale) topics only
- Community discussions confirm "Isaac Visual SLAM takes in mono8 rectified images"
- Attempting color conversion results in:
  ```
  ERROR: invalid input/output type for image color conversion
  CAMERA STREAM FAILED TO START
  ```

Visual SLAM algorithms perform feature detection/tracking on intensity values. Color information provides no benefit and is incompatible with cuVSLAM's internal processing.

### Network Configuration
Required for reliable DDS communication with large messages (images, point clouds):

**Multicast on loopback**:
```bash
sudo ip link set lo multicast on
```

**Kernel parameters** (`/etc/sysctl.conf` or `/etc/sysctl.d/10-cyclone-max.conf`):
```
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
```

`.envrc` validates these on directory entry and shows warnings if missing.

See: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/

## Common Tasks

### First Time Setup
```bash
direnv allow              # Enable .envrc
just setup                # Install dependencies and download assets
just build                # Build workspace
```

### ZED X Mini Usage
```bash
just launch-zedxm         # Launch camera + SLAM (RViz auto-starts)
just play-zedxm           # Launch with detailed logs in play_log/
```

### Rosbag Testing
```bash
# Assets are already downloaded during setup
just launch               # Terminal 1: Start Visual SLAM
just play                 # Terminal 2: Play rosbag
```

### Development Workflow
```bash
# After editing launch files or code:
just build
just launch-zedxm
```

## Troubleshooting

### ROS not found
- Check: `ls /opt/ros/humble`
- Source manually: `source /opt/ros/humble/setup.bash`
- Or: `direnv allow` (loads `.envrc`)

### Network warnings on directory entry
Fix using commands in docs/ZEDXM_SETUP.md "Network Configuration" section.

### Build fails
```bash
just rosdep-update
ros2 pkg list | grep isaac    # Verify Isaac ROS installed
```

### ZED camera not detected
```bash
/usr/local/zed/tools/ZED_Diagnostic    # Run ZED diagnostics
```

### Visual SLAM not producing odometry

**Initialization Requirements** (all must be met):
1. **Synchronized stereo images** - Both cameras publishing at same rate
2. **Camera_info messages** - Valid calibration for both cameras in correct namespaces
3. **Minimum synchronized frames** - Default `min_num_images = 2`
4. **Sufficient visual features** - Camera must see textured environment (not blank walls)
5. **Camera motion** - Move camera with rotation + translation to initialize

**Common Issues**:
- Wrong camera_info topic remapping (must match image namespaces: `left_gray/camera_info`)
- QoS mismatch between publishers and subscribers (use BEST_EFFORT throughout)
- IMU fusion enabled but no IMU data (disable with `enable_imu_fusion: False`)
- Camera pointing at featureless surface during initialization

**Verification Steps**:
```bash
# Check if Visual SLAM is receiving images
ros2 topic hz /left/image_rect /right/image_rect

# Check if camera_info is being received
ros2 topic info /zedxm/zed_node/left_gray/camera_info -v

# Check Visual SLAM subscription status
ros2 node info /visual_slam_node | grep camera_info

# Monitor odometry output
ros2 topic hz /visual_slam/tracking/odometry
```

### "Could not negotiate" warnings
- Normal during startup while NITROS negotiates formats
- Warnings should stop after all nodes initialize

### Color image errors
- Visual SLAM only supports mono8 grayscale
- Do NOT use `image_rect_color` topics
- Use `left_gray/image_rect_gray` and `right_gray/image_rect_gray`

## References

- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/
- ZED ROS2 Wrapper: https://www.stereolabs.com/docs/ros2/zed-node
- Autoware DDS Settings: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/
- Just Manual: https://just.systems/man/en/
- ROS 2 Launch XML: https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html

---

**Last Updated**: 2025-11-04 | Isaac ROS 3.2 | ZED SDK 5.0.7 | ROS 2 Humble | BEST_EFFORT QoS
