#!/usr/bin/env bash
# setup_system.sh — Install base system prerequisites on fresh Ubuntu 22.04
# Run with: sudo bash setup_system.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Error: run with sudo"
  exit 1
fi

REAL_USER="${SUDO_USER:?\"Run with sudo, not as root directly (need SUDO_USER)\"}"
HOME_DIR="/home/${REAL_USER}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "========================================="
echo "System Setup for Robot Board"
echo "User: ${REAL_USER}"
echo "========================================="
echo ""

# ── Basic tools ───────────────────────────────────────────────────────────────
echo -e "${YELLOW}Installing basic tools...${NC}"
apt update
apt install -y git curl software-properties-common openssh-server

# ── SSH ───────────────────────────────────────────────────────────────────────
systemctl enable --now ssh
echo -e "${GREEN}✓ SSH enabled${NC}"

# ── ROS2 Humble ───────────────────────────────────────────────────────────────
if [ -d "/opt/ros/humble" ]; then
    echo -e "${GREEN}✓ ROS2 Humble already installed${NC}"
else
    echo -e "${YELLOW}Installing ROS2 Humble (this takes a while)...${NC}"
    add-apt-repository -y universe
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list
    apt update
    apt install -y ros-humble-desktop
    echo -e "${GREEN}✓ ROS2 Humble installed${NC}"
fi

# ── Colcon ────────────────────────────────────────────────────────────────────
echo -e "${YELLOW}Installing colcon...${NC}"
apt install -y python3-colcon-common-extensions
echo -e "${GREEN}✓ colcon installed${NC}"

# ── rosdep ────────────────────────────────────────────────────────────────────
echo -e "${YELLOW}Initializing rosdep...${NC}"
apt install -y python3-rosdep
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo -e "${GREEN}✓ rosdep already initialized${NC}"
else
    rosdep init
fi
sudo -u "$REAL_USER" rosdep update
echo -e "${GREEN}✓ rosdep ready${NC}"

# ── Tailscale ─────────────────────────────────────────────────────────────────
if command -v tailscale &>/dev/null; then
    echo -e "${GREEN}✓ Tailscale already installed${NC}"
else
    echo -e "${YELLOW}Installing Tailscale...${NC}"
    curl -fsSL https://tailscale.com/install.sh | sh
    echo -e "${GREEN}✓ Tailscale installed${NC}"
fi

# ── PATH for pip-installed binaries ───────────────────────────────────────────
BASHRC="${HOME_DIR}/.bashrc"
if ! grep -q '\.local/bin' "$BASHRC" 2>/dev/null; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$BASHRC"
    chown "${REAL_USER}:${REAL_USER}" "$BASHRC"
    echo -e "${GREEN}✓ Added ~/.local/bin to PATH${NC}"
fi

echo ""
echo -e "${GREEN}✓ System setup complete${NC}"
echo ""
echo "Next steps:"
echo "  1. sudo tailscale up   (authenticate this device)"
echo "  2. cd ~/robot && bash install_dependencies.sh"
echo "  3. make"
echo "  4. sudo TUNNEL_ID=<id> TUNNEL_HOSTNAME=<host> bash setup_boot_services.sh"
