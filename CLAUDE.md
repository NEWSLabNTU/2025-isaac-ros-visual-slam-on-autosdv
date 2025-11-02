# Project Context for Claude

## Overview
This is a ROS2 SLAM workspace using NVIDIA Isaac ROS Visual SLAM on Ubuntu 22.04 / Jetson platform.

## Project Structure

```
slam/
├── justfile                 # Build automation (replaces Makefile)
├── .envrc                   # Auto-sources ROS Humble when entering directory (direnv)
├── scripts/
│   ├── setup.sh            # System setup script
│   └── download_assets.sh  # Downloads Isaac ROS Visual SLAM assets
├── src/
│   └── slam_launch/        # Custom ROS2 package
│       ├── launch/
│       │   ├── visual_slam.launch.xml        # Basic visual SLAM launch
│       │   └── zed2_visual_slam.launch.xml   # ZED2 camera launch
│       └── config/
│           ├── quickstart_interface_specs.json
│           └── zed2_quickstart_interface_specs.json
└── assets/                  # Downloaded SLAM assets (rosbag, configs)
```

## Key Setup Details

### System Requirements
- Ubuntu 22.04
- ROS Humble installed at `/opt/ros/humble`
- Isaac ROS packages installed via APT
- NVIDIA GPU with CUDA support

### Build System
Uses **Just** (modern Make alternative) instead of Makefile.
- Justfile is minimalist style (no emojis, concise output)
- All recipes source ROS automatically

### Important Files

**justfile** - Main automation:
- `just setup` - System setup (checks Ubuntu version, installs ROS/deps)
- `just build` - Builds workspace with Release mode
- `just clean` - Removes build artifacts
- `just launch` - Launches visual SLAM (auto-starts RViz if DISPLAY set)
- `just launch-zed2` - Launches with ZED2 camera
- `just play-bag` - Plays rosbag with topic remapping
- `just rosdep-update` - Updates rosdep database
- `just download-assets` - Downloads SLAM assets

**Launch Files** (XML format):
- Use composable nodes for zero-copy communication
- Include image format converters (RGB to mono8)
- Conditionally start RViz based on `start_rviz` argument
- Auto-detect DISPLAY in justfile targets

**.envrc**:
- Sources `/opt/ros/humble/setup.bash` when entering directory
- Shows warning if ROS not installed
- Requires `direnv` to be installed and enabled

## Dependencies

### APT Packages
- `ros-humble-desktop` - ROS 2 Humble with tools
- `ros-dev-tools` - Development tools
- `isaac_ros_visual_slam` - Visual SLAM node
- `isaac_ros_image_proc` - Image processing nodes
- `curl`, `jq`, `tar` - Utilities

### Python Dependencies
- rosdep

## Current State

### What Works
- Full workspace builds successfully
- Launch files start visual SLAM with composable nodes
- RViz auto-starts when DISPLAY is available
- Topic remapping for rosbag playback configured
- Image format conversion (RGB→mono8) integrated

### What's Not Set Up
- Isaac ROS APT repository (requires manual setup - see justfile comments)
- ZED2 camera driver configuration
- Actual rosbag testing with data playback

## Configuration Notes

### Visual SLAM Parameters
Both launch files configure:
- `rectified_images`: False for basic launch, True for ZED2
- `enable_slam_visualization`: True
- `enable_landmarks_view`: True
- `enable_observations_view`: True
- Configurable `base_frame` and `camera_optical_frames`

### Topic Remapping
Rosbag topics remapped in `play-bag` target:
- Front stereo camera: `/front_stereo_camera/{left,right}/*` → `/{left,right}/*`
- Back stereo camera: `/back_stereo_camera/{left,right}/*` → `/rear_{left,right}/*`

### Build Configuration
- Uses `--symlink-install` for faster development
- Release build (`-DCMAKE_BUILD_TYPE=Release`) for performance
- colcon builds only the `slam_launch` package

## Common Tasks

### First Time Setup
```bash
just setup          # Install all dependencies
just download-assets # Get SLAM assets/rosbag
just build          # Build workspace
```

### Development Workflow
```bash
just build          # After code changes
just launch         # Test visual SLAM
just play-bag       # Play rosbag in another terminal
```

### Clean Build
```bash
just clean
just build
```

## Troubleshooting

### ROS not found
- Check if `/opt/ros/humble` exists
- Source manually: `source /opt/ros/humble/setup.bash`
- Or use direnv: `direnv allow`

### Build fails
- Run `just rosdep-update` first
- Check if all Isaac ROS packages installed: `ros2 pkg list | grep isaac`

### Launch fails
- Ensure workspace is built: `just build`
- Check if sourced: `source install/setup.bash` (justfile does this automatically)

### RViz doesn't start
- Check DISPLAY: `echo $DISPLAY`
- Force RViz: `ros2 launch slam_launch visual_slam.launch.xml start_rviz:=true`

## Next Steps / TODO

1. Test with actual rosbag playback
2. Verify visual SLAM tracking works
3. Configure ZED2 camera if available
4. Tune SLAM parameters for your environment
5. Set up map saving/loading if needed

## References

- Isaac ROS Visual SLAM Docs: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/
- ROS 2 Launch XML Format: https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html
- Just Manual: https://just.systems/man/en/
