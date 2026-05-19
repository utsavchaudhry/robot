# robot

Humanoid robot teleoperation stack — ROS 2 Humble, GStreamer WebRTC, x86_64 (LattePanda). Streams stereo video to the operator's headset / browser and accepts VR controller + head poses back to drive arms, head, and differential-drive wheels.

The operator UI lives in a separate repo: [utsavchaudhry/webapp](https://github.com/utsavchaudhry/webapp).

## Layout

```
firmwares/                  ESP32 sketches for the servo + cybergear buses
install_dependencies.sh     One-shot apt/pip/rosdep install for Ubuntu 22.04 + Humble
Makefile                    build / deploy / restart targets
requirements.txt            Python deps used outside rosdep (pinocchio, pink, etc.)
setup_board.sh              First-time provisioning of a fresh LattePanda
src/
  motor_handler/            ESP32 discovery + servo / cybergear bridge
  robot_bringup/            Launch files, RViz configs, IK tooling, recorder
  robot_description/        URDF (xacro) + meshes
  robot_interfaces/         TeleopCommand.msg, ArmCommand.msg, ComputeIK.action
  robot_kinematics/         Pinocchio + Pink whole-body IK
  robot_webrtc/             GStreamer webrtcbin pipeline, signaling, teleop controller
workers/                    Cloudflare Worker for recordings browser
```

The `src/*` packages used to be GitLab submodules; this repo is now flat.

## Bring-up on a fresh board

```bash
git clone git@github.com:utsavchaudhry/robot.git
cd robot
./install_dependencies.sh
make build
```

`install_dependencies.sh` installs ROS 2 Humble, GStreamer 1.22+ (needed — 1.20 has a webrtcbin SIGABRT under data-channel load), Pinocchio, and the Python deps for `robot_kinematics`.

## Running

The production target is `make deploy` (rebuilds the runtime packages and restarts `robot-teleop.service`). For manual testing, stop the service and launch by hand:

```bash
sudo systemctl stop robot-teleop
source /opt/ros/humble/setup.bash && source ./install/setup.bash

# Head-only (safe default while testing):
ros2 launch robot_bringup robot_teleop.launch.py enabled_joints:='head_yaw,head_pitch'

# Add the left arm:
ros2 launch robot_bringup robot_teleop.launch.py \
  enabled_joints:='head_yaw,head_pitch,left_shoulder_pitch,left_shoulder_yaw,left_shoulder_roll,left_elbow_flex,left_wrist_roll,left_wrist_yaw,left_hand_wrist_pitch,left_gripper'

# Everything:
ros2 launch robot_bringup robot_teleop.launch.py enabled_joints:='all'
```

`enabled_joints` only gates servo *commands*. Position readback works for every discovered joint regardless. Wheels go through `/cmd_vel` → `XiaomiESP32` and are always live (not gated by this arg).

## Make targets

| target          | what it does                                                           |
| --------------- | ---------------------------------------------------------------------- |
| `make build`    | rosdep + `colcon build` (everything)                                   |
| `make deploy`   | targeted rebuild of runtime packages + `systemctl restart robot-teleop` |
| `make restart`  | just restart the service                                               |
| `make clean`    | wipe `build/ install/ log/`                                            |

## Visualizing the live IK in RViz

The robot board can drive a monitor and watch RViz alongside live teleop. From a second terminal while `robot-teleop` is running:

```bash
source /opt/ros/humble/setup.bash && source ./install/setup.bash
ros2 launch robot_bringup teleop_rviz.launch.py
# source:=commands (default) shows what IK is asking for
# source:=states            shows what the motors actually report
```

This starts `robot_state_publisher`, the `joint_state_relay` aggregator, and RViz with `ik_viz.rviz` — read-only, doesn't touch the running stack.

## Configuration

All tunable runtime params live in `src/robot_bringup/config/teleop_config.yaml`:

- `teleop_controller_node` — workspace bounds, drive speed limits, watchdog timeout
- `humanoid_kinematics_node` — IK cost weights (`position_cost`, `orientation_cost`, `regularization_cost`), update rate, smoothing filter
- `session_recorder` — output dir + R2 upload bucket

The URDF lives in `src/robot_description/urdf/humanoid.urdf.xacro`. Meshes are STL (collision) / DAE (visual) under `src/robot_description/meshes/`.

## Two robots

There are two physical units. They share this repo and the URDF. Differences (calibration, motor IDs, signaling URL) are handled either by the per-host `teleop_config.yaml` or by Cloudflare Pages projects on the operator side. Default deploy target on the operator UI is `robot-b` — robot-a is a production unit and shouldn't be redeployed without explicit confirmation.

## Operator UI deploy

The webapp this stack talks to is a Cloudflare Pages site with two project slots:

- `robot-control` — points at robot-a
- `robot-control-b` — points at robot-b

See [utsavchaudhry/webapp](https://github.com/utsavchaudhry/webapp) for the `make deploy-a` / `make deploy-b` workflow.
