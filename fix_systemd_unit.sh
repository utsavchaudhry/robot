#!/usr/bin/env bash
# Replace /etc/systemd/system/robot-teleop.service with a version that:
#   - uses `exec` so ros2 launch becomes the cgroup root (signals propagate)
#   - uses SIGINT on stop so nodes get their normal Ctrl-C cleanup path
#   - keeps Restart=on-failure for production resilience
# Run as:  sudo ./fix_systemd_unit.sh

set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "Run with sudo." >&2
    exit 1
fi

UNIT=/etc/systemd/system/robot-teleop.service
BACKUP="${UNIT}.bak-$(date +%Y%m%d-%H%M%S)"

if [ -f "$UNIT" ]; then
    echo "Backing up existing unit -> $BACKUP"
    cp -p "$UNIT" "$BACKUP"
fi

cat > "$UNIT" <<'EOF'
[Unit]
Description=Robot Teleoperation Stack
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=robot
Environment=HOME=/home/robot
EnvironmentFile=-/home/robot/.r2_credentials

# `exec` replaces bash with ros2 launch so systemd signals reach the launch
# process directly. Without `exec`, bash is the cgroup root and grandchildren
# (the actual ROS nodes) end up detached from systemd's control after weird
# crash paths — that's how orphan node swarms used to build up across restarts.
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/robot/robot/install/setup.bash && exec ros2 launch robot_bringup robot_teleop.launch.py'

# ros2 launch traps SIGINT and forwards it to every node for clean shutdown.
# SIGTERM (the systemd default) works but skips node-level cleanup hooks.
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=30

Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

echo "Reloading systemd"
systemctl daemon-reload

echo
echo "==== New unit ===="
systemctl cat robot-teleop.service
echo
echo "Service state:"
systemctl is-enabled robot-teleop.service || true
systemctl is-active  robot-teleop.service || true

cat <<NOTE

Done. The service is left in whatever enabled/active state it was in before.
If you previously ran:  sudo systemctl disable robot-teleop
…it's still disabled.  Re-enable when you want autostart back:

    sudo systemctl enable robot-teleop
    sudo systemctl start  robot-teleop

To validate the fix without enabling autostart:

    sudo systemctl start  robot-teleop      # bring up
    ros2 node list                          # confirm nodes are alive
    sudo systemctl stop   robot-teleop      # bring down
    ros2 node list                          # should be EMPTY within a few seconds
    pgrep -af 'robot_webrtc|robot_kinematics|motor_handler|signaling' | head
                                            # should be empty too
NOTE
