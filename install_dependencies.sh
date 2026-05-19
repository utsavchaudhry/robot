#!/bin/bash
# Installation script for VRobot ROS2 Teleoperation System
# Run with: bash install_dependencies.sh

set -e  # Exit on error

echo "========================================="
echo "VRobot ROS2 Dependencies Installation"
echo "========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Detect NVIDIA Jetson via tegra release file (reliable, doesn't false-positive on ARM laptops)
if [ -f /etc/nv_tegra_release ]; then
    echo -e "${GREEN}✓ Detected NVIDIA Jetson platform${NC}"
    IS_JETSON=true
else
    IS_JETSON=false
    if [ "$(uname -m)" = "aarch64" ]; then
        echo -e "${YELLOW}⚠ aarch64 detected but not Jetson — GPU acceleration unavailable${NC}"
    else
        echo -e "${YELLOW}⚠ x86 platform — GPU acceleration unavailable${NC}"
    fi
fi

# Check if ROS2 Humble is installed
if [ -d "/opt/ros/humble" ]; then
    echo -e "${GREEN}✓ ROS2 Humble detected${NC}"
else
    echo -e "${RED}✗ ROS2 Humble not found${NC}"
    echo "Please install ROS2 Humble first:"
    echo "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

echo ""
echo "Installing system dependencies..."
sudo apt update

# GStreamer
echo -e "${YELLOW}Installing GStreamer and plugins...${NC}"
sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-nice \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    python3-gi \
    python3-gi-cairo \
    gir1.2-gstreamer-1.0 \
    gir1.2-gst-plugins-base-1.0 \
    gir1.2-gst-plugins-bad-1.0

# Python dependencies
echo -e "${YELLOW}Installing Python dependencies...${NC}"
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-websockets

# Pinocchio
echo -e "${YELLOW}Installing Pinocchio...${NC}"
sudo apt install -y ros-humble-pinocchio

# Additional ROS2 packages (xacro for .urdf.xacro in launch files)
echo -e "${YELLOW}Installing ROS2 packages...${NC}"
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport

# Python packages first: numpy<2, qpsolvers, loop-rate-limiters, quadprog (QP solver for Pink IK), etc.
# pin-pink is installed with --no-deps and uses ros-humble-pinocchio for Pinocchio.
if [ -f "requirements.txt" ]; then
  echo -e "${YELLOW}Installing Python packages from requirements.txt...${NC}"
  pip3 install -r requirements.txt || echo -e "${YELLOW}Warning: some pip packages failed. See README.${NC}"
else
  echo -e "${YELLOW}requirements.txt not found; run this script from the project root.${NC}"
fi

# Pink IK with --no-deps: use ros-humble-pinocchio for Pinocchio (already installed above).
# The pip "pin" 3.8 stack requires numpy>=2.2 (cmeel-boost 1.89), which conflicts with numpy<2
# needed for python3-opencv and ros pinocchio. So we avoid pulling pin/cmeel-boost from pip.
echo -e "${YELLOW}Installing Pink IK (pin-pink, --no-deps; uses ros-humble-pinocchio for Pinocchio)...${NC}"
pip3 install --no-deps pin-pink 2>/dev/null || {
    echo -e "${YELLOW}pin-pink not available for this arch; building from source...${NC}"
    (cd /tmp && rm -rf pink && git clone https://github.com/stephane-caron/pink.git && cd pink && pip3 install --no-deps -e .) || echo -e "${YELLOW}Pink install failed. See README.${NC}"
}

# ── Cloudflared (Cloudflare Tunnel) ──────────────────────────────────────────
echo ""
echo -e "${YELLOW}Installing Cloudflare Tunnel (cloudflared)...${NC}"
if command -v cloudflared &>/dev/null; then
    echo -e "${GREEN}✓ cloudflared already installed ($(cloudflared --version 2>/dev/null | head -1))${NC}"
else
    if [ "$(uname -m)" = "aarch64" ]; then
        CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64"
    else
        CLOUDFLARED_URL="https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64"
    fi
    echo "Downloading cloudflared..."
    sudo curl -fsSL -o /usr/local/bin/cloudflared "$CLOUDFLARED_URL"
    sudo chmod +x /usr/local/bin/cloudflared
    echo -e "${GREEN}✓ cloudflared installed ($(cloudflared --version 2>/dev/null | head -1))${NC}"
fi

# Remove pip opencv-python if installed — it shadows the ARM-optimized system python3-opencv
if pip3 show opencv-python &>/dev/null; then
    echo -e "${YELLOW}Removing pip opencv-python (shadows system python3-opencv)...${NC}"
    pip3 uninstall -y opencv-python 2>/dev/null || true
    pip3 uninstall -y opencv-python-headless 2>/dev/null || true
fi

# ── Jetson GPU acceleration (CUDA, NVJPEG, NVIDIA GStreamer plugins) ────────
if [ "$IS_JETSON" = true ]; then
    echo ""
    echo -e "${YELLOW}Installing Jetson GPU acceleration packages...${NC}"

    # nvidia-jetpack is the meta-package that pulls CUDA toolkit, cuDNN,
    # TensorRT, VPI, and NVIDIA GStreamer plugins (nvjpegenc, nvvidconv, etc.)
    if sudo apt install -y nvidia-jetpack 2>/dev/null; then
        echo -e "${GREEN}✓ nvidia-jetpack installed (CUDA + NVIDIA GStreamer plugins)${NC}"
    else
        # Fallback: try individual packages if the meta-package isn't available
        echo -e "${YELLOW}nvidia-jetpack not available, trying individual packages...${NC}"
        sudo apt install -y \
            nvidia-cuda-toolkit \
            nvidia-l4t-gstreamer \
            nvidia-l4t-multimedia \
            2>/dev/null || echo -e "${YELLOW}⚠ Some Jetson GPU packages failed. Install JetPack SDK via SDK Manager for full GPU support.${NC}"
    fi

    # Verify NVIDIA GStreamer plugins
    if gst-inspect-1.0 nvjpegenc &>/dev/null; then
        echo -e "${GREEN}✓ NVIDIA GStreamer plugins available (nvjpegenc, nvjpegdec, nvvidconv)${NC}"
    else
        echo -e "${YELLOW}⚠ nvjpegenc not found. NVIDIA GStreamer plugins may need JetPack SDK Manager install.${NC}"
    fi
fi

# ── CycloneDDS (reliable DDS for non-Jetson boards) ──────────────────────────
if [ "$IS_JETSON" = false ]; then
    echo ""
    echo -e "${YELLOW}Installing CycloneDDS (more reliable DDS for x86/non-Jetson boards)...${NC}"
    sudo apt install -y ros-humble-rmw-cyclonedds-cpp 2>/dev/null \
        && echo -e "${GREEN}✓ CycloneDDS installed${NC}" \
        || echo -e "${YELLOW}⚠ CycloneDDS install failed — FastDDS (default) will be used${NC}"
fi

# ── Arduino CLI + ESP32 board support (for flashing motor controller firmware) ─
echo ""
echo -e "${YELLOW}Installing Arduino CLI and ESP32 toolchain...${NC}"

ACLI=/usr/local/bin/arduino-cli
# Config/cores/libraries must be owned by the real user, not root.
ACLI_USER="${SUDO_USER:-$USER}"

if [ -x "$ACLI" ]; then
    echo -e "${GREEN}✓ arduino-cli already installed ($(sudo -u "$ACLI_USER" $ACLI version 2>/dev/null | head -1))${NC}"
else
    echo "Downloading arduino-cli..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh -o /tmp/arduino-install.sh
    sudo env BINDIR=/usr/local/bin bash /tmp/arduino-install.sh
    rm -f /tmp/arduino-install.sh
    echo -e "${GREEN}✓ arduino-cli installed${NC}"
fi

# ESP32 board core (run as the real user so config lives in ~/.arduino15)
sudo -u "$ACLI_USER" $ACLI config init 2>/dev/null || true
sudo -u "$ACLI_USER" $ACLI config add board_manager.additional_urls \
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json 2>/dev/null || true
echo "Updating board index (this may take a minute)..."
sudo -u "$ACLI_USER" $ACLI core update-index
if sudo -u "$ACLI_USER" $ACLI core list | grep -q esp32:esp32; then
    echo -e "${GREEN}✓ esp32:esp32 core already installed${NC}"
else
    sudo -u "$ACLI_USER" $ACLI core install esp32:esp32
    echo -e "${GREEN}✓ esp32:esp32 core installed${NC}"
fi

# SCServo library (Feetech STS/SCS servo control — used by SC_ST_Servo and SMServo firmwares)
if sudo -u "$ACLI_USER" $ACLI lib list | grep -q SCServo; then
    echo -e "${GREEN}✓ SCServo library already installed${NC}"
else
    sudo -u "$ACLI_USER" $ACLI lib install SCServo
    echo -e "${GREEN}✓ SCServo library installed${NC}"
fi

echo ""
echo -e "${GREEN}✓ Dependencies installed${NC}"
echo ""

echo "Updating user permissions..."
sudo usermod -aG audio,video,render,dialout,plugdev $(whoami)
echo -e "${GREEN}✓ Permissions updated${NC}"
echo ""

echo "========================================="
echo "Next steps (from project root):"
echo "========================================="
echo "1. source /opt/ros/humble/setup.bash"
echo "2. rosdep install --from-paths src --ignore-src -r -y"
echo "   (if rosdep fails for some pkgs, the script above already installed gst-plugins-bad etc.)"
echo "3. colcon build --symlink-install"
echo "4. source install/setup.bash"
echo "5. ros2 launch robot_bringup robot_teleop.launch.py"
echo ""
echo "Flash ESP32 firmware (plug in one board at a time, check port with: arduino-cli board list):"
echo "  # SC_ST_Servo → 'tc' board (ESP32 Dev Module)"
echo "  arduino-cli compile --fqbn esp32:esp32:esp32 firmwares/Deployed/SC_ST_Servo"
echo "  arduino-cli upload  --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 firmwares/Deployed/SC_ST_Servo"
echo ""
echo "  # SMServo → 'm' board (ESP32 Dev Module)"
echo "  arduino-cli compile --fqbn esp32:esp32:esp32 firmwares/Deployed/SMServo"
echo "  arduino-cli upload  --fqbn esp32:esp32:esp32 --port /dev/ttyUSB1 firmwares/Deployed/SMServo"
echo ""
echo "  # xiaomi_cybergear → 'xiaomi' board (XIAO ESP32S3)"
echo "  arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 firmwares/Deployed/xiaomi_cybergear"
echo "  arduino-cli upload  --fqbn esp32:esp32:XIAO_ESP32S3 --port /dev/ttyACM0 firmwares/Deployed/xiaomi_cybergear"
echo ""
echo -e "${GREEN}Installation complete!${NC}"
