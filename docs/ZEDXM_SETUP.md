# ZED X Mini + Isaac ROS Visual SLAM

Setup guide for ZED X Mini stereo camera with NVIDIA Isaac ROS Visual SLAM.

## Requirements

- Ubuntu 22.04 / ROS 2 Humble / NVIDIA Jetson
- ZED X Mini camera
- Packages: `isaac_ros_visual_slam`, `isaac_ros_image_proc`, `zed-ros2-wrapper`

## Network Configuration

Required for reliable image/point cloud transfers. Cyclone DDS is configured in `cyclonedds.xml`.

Enable multicast on loopback (permanent):
```bash
sudo tee /etc/systemd/system/multicast-lo.service <<EOF
[Unit]
Description=Enable Multicast on Loopback
[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set lo multicast on
[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now multicast-lo.service
```

Configure kernel parameters (permanent):
```bash
sudo tee /etc/sysctl.d/10-cyclone-max.conf <<EOF
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF

sudo sysctl -p /etc/sysctl.d/10-cyclone-max.conf
```

Verify:
```bash
ip link show lo | grep MULTICAST
sysctl net.core.rmem_max net.ipv4.ipfrag_time net.ipv4.ipfrag_high_thresh
```

See: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/

## Configuration

ZED X Mini publishes 960x600 @ 15 Hz (native 1920x1200 downscaled 2x).

Key topics:
- `/zedxm/zed_node/left_gray/image_rect_gray` (mono8)
- `/zedxm/zed_node/right_gray/image_rect_gray` (mono8)
- `/zedxm/zed_node/left/camera_info`
- `/zedxm/zed_node/right/camera_info`

Frames:
- `zedxm_camera_center` (base_frame)
- `zedxm_left_camera_optical_frame`
- `zedxm_right_camera_optical_frame`

## Architecture

Data flow: `ZED Camera → ImageFormatConverter → Visual SLAM`

Image format converters:
- Subscribe to ZED grayscale topics (mono8)
- NITROS negotiation declares `nitros_image_rgb8` but processes mono8
- Output `nitros_image_mono8` to `/left/image_rect` and `/right/image_rect`
- Composable nodes in shared container for zero-copy communication

**Why grayscale?** Isaac ROS Visual SLAM (cuVSLAM) requires mono8 grayscale images. Color images are not supported as the algorithm extracts features from intensity values only.

Launch file (`src/slam_launch/launch/zedxm_visual_slam.launch.xml`):
- ZED camera with `enable_ipc: false`
- Two ImageFormatConverterNode instances (left/right) at 960x600
- VisualSlamNode with `rectified_images: True`
- All nodes in same container

## Usage

Build:
```bash
just build
```

Launch:
```bash
just launch-zedxm          # Auto-starts RViz if DISPLAY set
just play-zedxm            # With detailed logs in play_log/
```

Verify:
```bash
ros2 topic hz /left/image_rect /right/image_rect        # Should show ~14-15 Hz
ros2 topic list | grep visual_slam                      # Check SLAM topics
```

Expected log output:
```
[INFO] [visual_slam_node]: cuVSLAM version: 12.6
[INFO] [image_format_node_left]: [NitrosNode] Node was started
[INFO] [image_format_node_right]: [NitrosNode] Node was started
```

## Troubleshooting

**ZED container crash on startup**
```
terminate called: intraprocess communication allowed only with volatile durability
```
Solution: Ensure `enable_ipc: false` in launch file (already configured).

**"Could not negotiate" warnings**
Normal during startup. NITROS negotiation completes after all nodes initialize.

**Camera stream failed to start**
```bash
pkill -f "ros2|play_launch|zed"    # Kill stale processes
sleep 5
just launch-zedxm                   # Retry
```

**No images in RViz**
Check topic names match RViz configuration (`/left/image_rect`, `/right/image_rect`).

**Visual SLAM not tracking**
Requires camera motion to initialize. Move camera slowly with rotation and translation.

**Resolution mismatch**
Ensure matching 960x600 in:
- Launch file ImageFormatConverter params
- `config/zedxm_quickstart_interface_specs.json`

**Network warnings on directory entry**
Fix using commands in Network Configuration section above.

**Color images cause conversion errors**
cuVSLAM only supports mono8 grayscale images. Attempting to use color topics (`image_rect_color`) will result in "invalid input/output type for image color conversion" errors and camera stream failures. Use grayscale topics (`left_gray/image_rect_gray`, `right_gray/image_rect_gray`) as configured.

## References

- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/
- ZED ROS2 Docs: https://www.stereolabs.com/docs/ros2/zed-node
- Autoware DDS Settings: https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/

---

Last updated: 2025-11-04 | Isaac ROS 3.2 | ZED SDK 5.0.7 | ROS 2 Humble
