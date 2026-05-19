#!/usr/bin/env python3
"""
Interactive terminal UI for testing individual motors.

Launch motor_handler with the joint(s) you want to move enabled:

    ros2 run motor_handler motor_handler_node --ros-args \
        -p enabled_joints:="['head_yaw']"

Then launch this tool (separate terminal):

    ros2 run motor_handler motor_debug

Keys:
    UP/DOWN      Select motor
    LEFT/RIGHT   Nudge by step size
    +/-          Double / halve step size
    A            Type an exact angle (radians)
    0            Send zero (0.0 rad)
    C            Send center of joint range
    R            Sync target to current readback
    S            Toggle sinusoidal sweep
    [/]          Decrease/increase sweep amplitude
    {/}          Decrease/increase sweep frequency
    Q            Quit
"""

import curses
import math
import os
import time
import threading

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory


def _topic_for(name):
    if name.startswith('head'):
        return 'head/joint_commands'
    if name.startswith('right'):
        return 'right_arm/joint_commands'
    return 'left_arm/joint_commands'


class MotorDebugNode(Node):

    def __init__(self):
        super().__init__('motor_debug')

        default_cfg = os.path.join(
            get_package_share_directory('motor_handler'),
            'config', 'servo_config.yaml')
        self.declare_parameter('servo_config', default_cfg)
        path = (self.get_parameter('servo_config')
                .get_parameter_value().string_value)

        with open(path) as f:
            self.servo_config = yaml.safe_load(f).get('servos', [])

        self._positions = {}
        self._pos_lock = threading.Lock()

        for topic in ('head/joint_states',
                      'right_arm/joint_states',
                      'left_arm/joint_states'):
            self.create_subscription(JointState, topic,
                                     self._state_cb, 10)

        self._pubs = {}
        for topic in ('head/joint_commands',
                      'right_arm/joint_commands',
                      'left_arm/joint_commands'):
            self._pubs[topic] = self.create_publisher(JointState, topic, 10)

    def _state_cb(self, msg):
        with self._pos_lock:
            for n, p in zip(msg.name, msg.position):
                self._positions[n] = p

    def get_pos(self, name):
        with self._pos_lock:
            return self._positions.get(name)

    def send(self, name, angle):
        topic = _topic_for(name)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [name]
        msg.position = [float(angle)]
        self._pubs[topic].publish(msg)


# ---------------------------------------------------------------------------
# Curses UI
# ---------------------------------------------------------------------------

def _run_ui(stdscr, node):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)    # selected row
    curses.init_pair(2, curses.COLOR_YELLOW, -1)   # sweeping
    curses.init_pair(3, curses.COLOR_RED, -1)       # warning
    curses.init_pair(4, curses.COLOR_CYAN, -1)      # header

    motors = node.servo_config
    if not motors:
        stdscr.addstr(0, 0, "No servos in config!", curses.color_pair(3))
        stdscr.getch()
        return

    sel = 0
    step = 0.05
    targets = {}          # name -> float
    sweep_on = {}         # name -> bool
    sweep_amp = 0.30
    sweep_freq = 0.50
    sweep_t0 = time.monotonic()

    status = "Ready. Select a motor and press LEFT/RIGHT to nudge."
    input_mode = False
    input_buf = ""

    while True:
        now = time.monotonic()

        # --- sweep active motors ---
        for cfg in motors:
            nm = cfg['name']
            if not sweep_on.get(nm):
                continue
            mid = (cfg['angle_min'] + cfg['angle_max']) / 2.0
            half = (cfg['angle_max'] - cfg['angle_min']) / 2.0
            amp = min(sweep_amp, half)
            angle = mid + amp * math.sin(2 * math.pi * sweep_freq * (now - sweep_t0))
            angle = max(cfg['angle_min'], min(cfg['angle_max'], angle))
            targets[nm] = angle
            node.send(nm, angle)

        # --- input ---
        try:
            key = stdscr.getch()
        except Exception:
            key = -1

        if input_mode:
            if key == 27:  # ESC
                input_mode = False
                input_buf = ""
                status = "Input cancelled."
            elif key in (curses.KEY_ENTER, 10, 13):
                input_mode = False
                try:
                    val = float(input_buf)
                    cfg = motors[sel]
                    nm = cfg['name']
                    val = max(cfg['angle_min'], min(cfg['angle_max'], val))
                    targets[nm] = val
                    node.send(nm, val)
                    status = f"Set {nm} = {val:+.4f} rad"
                except ValueError:
                    status = f"Bad input: '{input_buf}'"
                input_buf = ""
            elif key in (curses.KEY_BACKSPACE, 127, 8):
                input_buf = input_buf[:-1]
            elif 32 <= key < 127:
                input_buf += chr(key)
        else:
            if key in (ord('q'), ord('Q')):
                break
            elif key == curses.KEY_UP:
                sel = max(0, sel - 1)
            elif key == curses.KEY_DOWN:
                sel = min(len(motors) - 1, sel + 1)
            elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
                cfg = motors[sel]
                nm = cfg['name']
                if sweep_on.get(nm):
                    status = "Stop sweep first (S)"
                else:
                    cur = targets.get(nm) or node.get_pos(nm)
                    if cur is None:
                        cur = (cfg['angle_min'] + cfg['angle_max']) / 2.0
                    delta = step if key == curses.KEY_RIGHT else -step
                    nv = max(cfg['angle_min'], min(cfg['angle_max'], cur + delta))
                    targets[nm] = nv
                    node.send(nm, nv)
                    status = f"{nm} = {nv:+.4f} rad"
            elif key in (ord('+'), ord('=')):
                step = min(1.0, step * 2)
                status = f"Step: {step:.4f} rad"
            elif key in (ord('-'), ord('_')):
                step = max(0.001, step / 2)
                status = f"Step: {step:.4f} rad"
            elif key == ord('0'):
                cfg = motors[sel]
                nm = cfg['name']
                if sweep_on.get(nm):
                    status = "Stop sweep first (S)"
                else:
                    val = max(cfg['angle_min'], min(cfg['angle_max'], 0.0))
                    targets[nm] = val
                    node.send(nm, val)
                    status = f"{nm} = {val:+.4f} rad (zero)"
            elif key in (ord('c'), ord('C')):
                cfg = motors[sel]
                nm = cfg['name']
                if sweep_on.get(nm):
                    status = "Stop sweep first (S)"
                else:
                    val = (cfg['angle_min'] + cfg['angle_max']) / 2.0
                    targets[nm] = val
                    node.send(nm, val)
                    status = f"{nm} = {val:+.4f} rad (center)"
            elif key in (ord('r'), ord('R')):
                cfg = motors[sel]
                nm = cfg['name']
                pos = node.get_pos(nm)
                if pos is not None:
                    targets[nm] = pos
                    status = f"Synced target for {nm} to readback {pos:+.4f}"
                else:
                    status = f"No readback yet for {nm}"
            elif key in (ord('a'), ord('A')):
                input_mode = True
                input_buf = ""
                status = "Type angle (rad), ENTER to send, ESC to cancel"
            elif key in (ord('s'), ord('S')):
                nm = motors[sel]['name']
                if sweep_on.get(nm):
                    sweep_on[nm] = False
                    status = f"Sweep OFF: {nm}"
                else:
                    sweep_on[nm] = True
                    sweep_t0 = now
                    status = f"Sweep ON: {nm} (amp={sweep_amp:.2f} freq={sweep_freq:.2f})"
            elif key == ord('['):
                sweep_amp = max(0.01, sweep_amp - 0.05)
                status = f"Sweep amp: {sweep_amp:.2f} rad"
            elif key == ord(']'):
                sweep_amp = min(1.5, sweep_amp + 0.05)
                status = f"Sweep amp: {sweep_amp:.2f} rad"
            elif key == ord('{'):
                sweep_freq = max(0.1, sweep_freq - 0.1)
                status = f"Sweep freq: {sweep_freq:.2f} Hz"
            elif key == ord('}'):
                sweep_freq = min(2.0, sweep_freq + 0.1)
                status = f"Sweep freq: {sweep_freq:.2f} Hz"

        # --- draw ---
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        if h < 6 or w < 40:
            stdscr.addstr(0, 0, "Terminal too small")
            stdscr.refresh()
            continue

        # title
        title = " Motor Debug "
        stdscr.addstr(0, max(0, (w - len(title)) // 2), title,
                       curses.A_BOLD | curses.color_pair(4))

        keys_help = ("UP/DN:select  L/R:nudge  +/-:step  "
                     "A:angle  0:zero  C:center  R:sync  S:sweep  Q:quit")
        stdscr.addstr(1, 0, keys_help[:w - 1], curses.A_DIM)

        # column header
        row = 3
        hdr = (f"   {'Motor':<28}{'Read':>9} {'Target':>9}"
               f" {'Min':>9} {'Max':>9} {'ESP':>4} {'ID':>4}")
        stdscr.addstr(row, 0, hdr[:w - 1],
                       curses.A_BOLD | curses.color_pair(4))
        row += 1
        stdscr.addstr(row, 0, "\u2500" * min(w - 1, len(hdr)), curses.A_DIM)
        row += 1

        # scrollable list
        list_h = max(1, h - row - 5)
        scroll = max(0, sel - list_h + 1)

        for i in range(scroll, min(len(motors), scroll + list_h)):
            if row >= h - 4:
                break
            cfg = motors[i]
            nm = cfg['name']
            rp = node.get_pos(nm)
            read_s = f"{rp:+.4f}" if rp is not None else "   ---  "
            tgt = targets.get(nm)
            tgt_s = f"{tgt:+.4f}" if tgt is not None else "   ---  "
            sw = " ~" if sweep_on.get(nm) else ""
            marker = ">" if i == sel else " "
            line = (f" {marker} {nm:<28}{read_s:>9} {tgt_s:>9}"
                    f" {cfg['angle_min']:>9.4f} {cfg['angle_max']:>9.4f}"
                    f" {cfg['esp']:>4} {cfg['servo_id']:>4}{sw}")

            if i == sel:
                attr = curses.A_BOLD | curses.color_pair(1)
            elif sweep_on.get(nm):
                attr = curses.color_pair(2)
            else:
                attr = 0
            stdscr.addstr(row, 0, line[:w - 1], attr)
            row += 1

        # footer
        foot = h - 4
        if foot > row:
            row = foot
        info = (f"Step: {step:.4f} rad  |  "
                f"Sweep: amp={sweep_amp:.2f}  freq={sweep_freq:.2f} Hz  "
                f"( [/] amp   {{/}} freq )")
        if row < h - 1:
            stdscr.addstr(row, 0, info[:w - 1], curses.A_DIM)
            row += 1

        # hint for motor_handler
        sel_name = motors[sel]['name']
        hint = f"motor_handler cmd:  --ros-args -p enabled_joints:=\"['{sel_name}']\""
        if row < h - 1:
            stdscr.addstr(row, 0, hint[:w - 1], curses.A_DIM)
            row += 1

        if input_mode:
            prompt = f"Angle: {input_buf}_"
            if row < h - 1:
                stdscr.addstr(row, 0, prompt[:w - 1],
                              curses.A_BOLD | curses.color_pair(2))
                row += 1

        stdscr.addstr(min(row, h - 1), 0, status[:w - 1], curses.A_BOLD)
        stdscr.refresh()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MotorDebugNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(lambda scr: _run_ui(scr, node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
