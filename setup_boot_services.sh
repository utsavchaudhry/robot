#!/usr/bin/env bash
# setup_boot_services.sh — Install systemd services for cloudflared tunnel + robot teleop
# Run once on the robot with: sudo bash setup_boot_services.sh
set -euo pipefail

# --- Configuration (edit if needed) ---
TUNNEL_ID="${TUNNEL_ID:?\"Set TUNNEL_ID env var (e.g. TUNNEL_ID=abc123 sudo bash setup_boot_services.sh)\"}"
TUNNEL_HOSTNAME="${TUNNEL_HOSTNAME:?\"Set TUNNEL_HOSTNAME env var (e.g. TUNNEL_HOSTNAME=yourdomain.com)\"}"
USER="${SUDO_USER:?\"Run with sudo, not as root directly (need SUDO_USER)\"}"
HOME_DIR="/home/${USER}"
CLOUDFLARED_DIR="${HOME_DIR}/.cloudflared"
ROS_DISTRO="humble"
ROBOT_WS="${HOME_DIR}/robot"

# Detect board type for DDS configuration
if [ -f /etc/nv_tegra_release ]; then
    IS_JETSON=true
else
    IS_JETSON=false
fi
# ---------------------------------------

if [[ $EUID -ne 0 ]]; then
  echo "Error: run with sudo"
  exit 1
fi

echo "=== 0/4  Session recorder setup ==="
sudo -u "${USER}" mkdir -p "${HOME_DIR}/teleop_recordings"
echo "  Ensured ${HOME_DIR}/teleop_recordings"

echo "=== 1/4  Cloudflared config ==="
cat > "${CLOUDFLARED_DIR}/config.yml" <<EOF
tunnel: ${TUNNEL_ID}
credentials-file: ${CLOUDFLARED_DIR}/${TUNNEL_ID}.json

ingress:
  - hostname: ${TUNNEL_HOSTNAME}
    service: http://localhost:8443
  - service: http_status:404
EOF
chown "${USER}:${USER}" "${CLOUDFLARED_DIR}/config.yml"
echo "  Wrote ${CLOUDFLARED_DIR}/config.yml"

echo "=== 2/4  Cloudflared service ==="
cat > /etc/systemd/system/cloudflared.service <<EOF
[Unit]
Description=Cloudflare Tunnel (robot-teleop)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER}
ExecStart=/usr/local/bin/cloudflared --config ${CLOUDFLARED_DIR}/config.yml tunnel run
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
echo "  Wrote /etc/systemd/system/cloudflared.service"

echo "=== 3/4  Robot teleop service ==="

# Non-Jetson boards use CycloneDDS for reliable DDS discovery
DDS_ENV=""
if [ "$IS_JETSON" = false ]; then
    DDS_ENV="Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    echo "  Using CycloneDDS (non-Jetson board detected)"
fi

cat > /etc/systemd/system/robot-teleop.service <<EOF
[Unit]
Description=Robot Teleoperation Stack
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER}
Environment=HOME=${HOME_DIR}
${DDS_ENV}
EnvironmentFile=-${HOME_DIR}/.r2_credentials
ExecStart=/bin/bash -c '\
  source /opt/ros/${ROS_DISTRO}/setup.bash && \
  source ${ROBOT_WS}/install/setup.bash && \
  ros2 launch robot_bringup robot_teleop.launch.py'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF
echo "  Wrote /etc/systemd/system/robot-teleop.service"

echo "=== 4/5  Sudoers rule for service self-restart ==="
cat > /etc/sudoers.d/robot-teleop-restart <<EOF
${USER} ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart robot-teleop
EOF
chmod 440 /etc/sudoers.d/robot-teleop-restart
echo "  Wrote /etc/sudoers.d/robot-teleop-restart"

echo "=== 5/5  Enable & start ==="
systemctl daemon-reload
systemctl enable cloudflared.service
systemctl enable robot-teleop.service
systemctl restart cloudflared.service
systemctl restart robot-teleop.service

echo ""
echo "Done. Both services are enabled and running."
echo "  sudo systemctl status cloudflared"
echo "  sudo systemctl status robot-teleop"
echo "  journalctl -u cloudflared -f"
echo "  journalctl -u robot-teleop -f"
