#!/usr/bin/env bash
# setup_board.sh — Full board setup from fresh Ubuntu 22.04
# Run with: sudo bash setup_board.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -ne 0 ]]; then
  echo "Error: run with sudo"
  exit 1
fi

REAL_USER="${SUDO_USER:?\"Run with sudo, not as root directly (need SUDO_USER)\"}"
HOME_DIR="/home/${REAL_USER}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "========================================="
echo "Full Board Setup"
echo "User: ${REAL_USER}"
echo "========================================="
echo ""

# ── Step 1: System prerequisites ──────────────────────────────────────────────
echo -e "${YELLOW}>>> Step 1/4: System prerequisites${NC}"
bash "${SCRIPT_DIR}/setup_system.sh"

# ── Step 2: Project dependencies ──────────────────────────────────────────────
echo -e "${YELLOW}>>> Step 2/4: Project dependencies${NC}"
sudo -u "$REAL_USER" bash -c "cd '${SCRIPT_DIR}' && source /opt/ros/humble/setup.bash && bash install_dependencies.sh"

# ── Step 3: Build ─────────────────────────────────────────────────────────────
echo -e "${YELLOW}>>> Step 3/4: Building project${NC}"
sudo -u "$REAL_USER" bash -c "cd '${SCRIPT_DIR}' && source /opt/ros/humble/setup.bash && colcon build"

# ── Step 4: Boot services ────────────────────────────────────────────────────
echo -e "${YELLOW}>>> Step 4/4: Boot services${NC}"

read -rp "Cloudflare Tunnel ID: " TUNNEL_ID
read -rp "Tunnel hostname (e.g. yourdomain.com): " TUNNEL_HOSTNAME

if [ ! -f "${HOME_DIR}/.cloudflared/${TUNNEL_ID}.json" ]; then
    echo ""
    echo -e "${YELLOW}Tunnel credentials not found: ${HOME_DIR}/.cloudflared/${TUNNEL_ID}.json${NC}"
    echo "  Copy from another machine:"
    echo "  scp ~/.cloudflared/${TUNNEL_ID}.json ${REAL_USER}@\$(hostname):~/.cloudflared/"
    echo ""
    read -rp "Continue anyway? (y/N) " REPLY
    [[ $REPLY =~ ^[Yy]$ ]] || exit 1
fi

TUNNEL_ID="$TUNNEL_ID" TUNNEL_HOSTNAME="$TUNNEL_HOSTNAME" bash "${SCRIPT_DIR}/setup_boot_services.sh"

# ── Tailscale auth ────────────────────────────────────────────────────────────
if ! tailscale status &>/dev/null; then
    echo ""
    echo -e "${YELLOW}Tailscale needs authentication:${NC}"
    tailscale up
fi

echo ""
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}Board setup complete!${NC}"
echo -e "${GREEN}=========================================${NC}"
echo "  sudo systemctl status robot-teleop"
echo "  sudo systemctl status cloudflared"
echo "  journalctl -u robot-teleop -f"
