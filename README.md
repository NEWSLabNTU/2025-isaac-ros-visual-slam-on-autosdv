# SLAM Project

Visual SLAM using NVIDIA Isaac ROS on ROS 2 Humble.

## Requirements

- Ubuntu 22.04
- NVIDIA GPU

## Installation

### 1. Install Just

```bash
cargo install just
# or
sudo apt install just
```

### 2. Setup System

```bash
just setup
```

This will:
- Check Ubuntu version
- Install ROS 2 Humble
- Install dependencies
- Configure rosdep

### 3. Download Assets

```bash
just download-assets
```

## Build

```bash
just build
```

## Usage

### Run Visual SLAM with Rosbag

Terminal 1:
```bash
just launch
```

Terminal 2:
```bash
just play-bag
```

RViz will start automatically if you have a display.

### Run with ZED2 Camera

```bash
just launch-zed2
```

Make sure your ZED2 camera is connected and publishing rectified stereo images to:
- `/left/image_rect`
- `/left/camera_info_rect`
- `/right/image_rect`
- `/right/camera_info_rect`

## Available Commands

```bash
just                  # List all commands
just check-system     # Check prerequisites
just build            # Build workspace
just clean            # Remove build artifacts
just launch           # Launch visual SLAM
just launch-zed2      # Launch with ZED2
just play-bag         # Play rosbag data
just rosdep-update    # Update rosdep
```

## Troubleshooting

**ROS not found?**
```bash
source /opt/ros/humble/setup.bash
```

**Build fails?**
```bash
just rosdep-update
just build
```

**RViz doesn't start?**

Check if display is available:
```bash
echo $DISPLAY
```

## Project Structure

```
slam/
├── justfile              # Build automation
├── src/slam_launch/      # Launch files and configs
└── assets/               # SLAM assets (downloaded)
```

## License

MIT
