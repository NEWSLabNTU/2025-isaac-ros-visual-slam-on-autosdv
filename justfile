# SLAM Project Automation

default:
    @just --list

check-sudo:
    @sudo -v

check-system:
    #!/usr/bin/env bash
    set -e
    MISSING=()

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        if [ "$VERSION_ID" != "22.04" ]; then
            echo "WARNING: Ubuntu 22.04 not detected (found: $PRETTY_NAME)"
            MISSING+=("ubuntu-22.04")
        fi
    else
        echo "WARNING: Cannot detect OS version"
        MISSING+=("os-detection")
    fi

    [ -d /opt/ros/humble ] || MISSING+=("ros-humble")
    grep -rq "isaac" /etc/apt/sources.list.d/ 2>/dev/null || MISSING+=("isaac-repo")
    command -v curl &> /dev/null || MISSING+=("curl")
    command -v jq &> /dev/null || MISSING+=("jq")
    command -v tar &> /dev/null || MISSING+=("tar")

    if [ ${#MISSING[@]} -eq 0 ]; then
        echo "All prerequisites satisfied"
    else
        echo "Missing: ${MISSING[*]}"
        echo "Run 'just setup' to install"
        exit 1
    fi

install-ros:
    #!/usr/bin/env bash
    set -e

    [ -d /opt/ros/humble ] && exit 0

    sudo apt update
    sudo apt install -y locales software-properties-common curl
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    sudo add-apt-repository universe -y

    ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    rm /tmp/ros2-apt-source.deb

    sudo apt update
    sudo apt upgrade -y
    sudo apt install -y ros-humble-desktop ros-dev-tools

    echo "ROS Humble installed. Source: /opt/ros/humble/setup.bash"

install-isaac-repo:
    #!/usr/bin/env bash
    set -e

    grep -rq "isaac" /etc/apt/sources.list.d/ 2>/dev/null && exit 0

    echo "Isaac ROS APT repository requires manual setup."
    echo "Follow: https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/isaac_apt_repository.html"
    echo ""
    echo "Example:"
    echo "  wget -qO - https://isaac-ros.github.io/isaac-ros/KEY.gpg | sudo apt-key add -"
    echo "  echo 'deb https://isaac-ros.github.io/isaac-ros/ubuntu/jammy jammy main' | sudo tee /etc/apt/sources.list.d/isaac-ros.list"
    echo "  sudo apt update"
    echo ""
    read -p "Press Enter after adding the repository..."

rosdep-update:
    #!/usr/bin/env bash
    set -e

    if ! command -v rosdep &> /dev/null; then
        sudo apt install -y python3-rosdep
    fi

    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi

    rosdep update

install-deps: check-sudo rosdep-update
    #!/usr/bin/env bash
    set -e

    sudo apt update
    sudo apt install -y curl jq tar
    rosdep install --from-paths src --ignore-src -r -y

download-assets:
    @bash scripts/download_assets.sh

play-bag:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    ros2 bag play assets/isaac_ros_visual_slam/quickstart_bag --remap \
        /front_stereo_camera/left/image_raw:=/left/image_rect \
        /front_stereo_camera/left/camera_info:=/left/camera_info_rect \
        /front_stereo_camera/right/image_raw:=/right/image_rect \
        /front_stereo_camera/right/camera_info:=/right/camera_info_rect \
        /back_stereo_camera/left/image_raw:=/rear_left/image_rect \
        /back_stereo_camera/left/camera_info:=/rear_left/camera_info_rect \
        /back_stereo_camera/right/image_raw:=/rear_right/image_rect \
        /back_stereo_camera/right/camera_info:=/rear_right/camera_info_rect

launch:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    if [ -n "$DISPLAY" ]; then
        ros2 launch slam_launch visual_slam.launch.xml start_rviz:=true
    else
        ros2 launch slam_launch visual_slam.launch.xml start_rviz:=false
    fi

launch-zedxm:
    #!/usr/bin/env bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    if [ -n "$DISPLAY" ]; then
        ros2 launch slam_launch zedxm_visual_slam.launch.xml start_rviz:=true
    else
        ros2 launch slam_launch zedxm_visual_slam.launch.xml start_rviz:=false
    fi
    fi

build:
    @colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

clean:
    @rm -rf build/ install/ log/

setup: check-sudo check-system
    #!/usr/bin/env bash
    set -e

    NEEDS_INSTALL=false

    if [ ! -d /opt/ros/humble ]; then
        echo "Will install: ROS Humble"
        NEEDS_INSTALL=true
    fi

    if ! grep -rq "isaac" /etc/apt/sources.list.d/ 2>/dev/null; then
        echo "Will configure: Isaac ROS APT repository"
        NEEDS_INSTALL=true
    fi

    if ! command -v curl &> /dev/null || ! command -v jq &> /dev/null; then
        echo "Will install: project dependencies"
        NEEDS_INSTALL=true
    fi

    if [ "$NEEDS_INSTALL" = false ]; then
        echo "All components installed"
        read -p "Update dependencies? (y/N): " -n 1 -r
        echo
        [[ ! $REPLY =~ ^[Yy]$ ]] && exit 0
    else
        read -p "Proceed with installation? (y/N): " -n 1 -r
        echo
        [[ ! $REPLY =~ ^[Yy]$ ]] && exit 0
    fi

    [ ! -d /opt/ros/humble ] && just install-ros
    ! grep -rq "isaac" /etc/apt/sources.list.d/ 2>/dev/null && just install-isaac-repo
    just install-deps

    echo ""
    echo "Setup complete"
    echo "Next: source /opt/ros/humble/setup.bash && just download-assets && just build"
