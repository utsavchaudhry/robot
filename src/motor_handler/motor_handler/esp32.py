from typing import List, Dict, Optional
from collections import deque

import glob as glob_mod
import serial
import time
import threading


def _probe_port(port, deadline_s=8.0):
    """Try one port. Returns (port, identity, ser) or (port, None, None).

    Self-contained so it can run in a worker thread alongside other probes.
    """
    try:
        ser = serial.Serial(port, 115200, timeout=1.0, write_timeout=1)
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        ser.reset_input_buffer()
        time.sleep(0.2)
        try:
            ser.write(b"identify\r\n")
            ser.flush()
        except Exception:
            ser.close()
            return port, None, None, "write timeout"

        identity = None
        deadline = time.time() + deadline_s
        last_send = time.time()
        while time.time() < deadline:
            try:
                line = ser.readline()
            except Exception:
                break
            if line:
                text = line.decode("utf-8", errors="replace").strip()
                if text in ("tc", "m", "xiaomi"):
                    identity = text
                    break
            # Resend periodically (xiaomi firmware blocks in setup() and
            # only checks serial between retries).
            if time.time() - last_send > 1.0:
                try:
                    ser.write(b"identify\r\n")
                    ser.flush()
                except Exception:
                    break
                last_send = time.time()

        if identity:
            ser.timeout = 0.1
            ser.reset_input_buffer()
            return port, identity, ser, None

        ser.close()
        return port, None, None, "no identity response"
    except Exception as e:
        return port, None, None, str(e)


def discover_devices(logger=None, deadline_s=8.0, settle_time=0.5):
    """Scan serial ports in parallel and return {identity: serial.Serial}.

    Each `/dev/ttyACM*` is probed on its own thread, so the slow xiaomi
    (which can hold off USB CDC responses for ~30 s during its CAN init)
    doesn't block the other ports. Discovery completes in roughly
    `deadline_s` wall-clock seconds, not Σ(per-port times).

    Args:
        deadline_s: Per-port deadline waiting for the "identify" response.
                    8 s comfortably covers xiaomi's worst-case boot.
        settle_time: Seconds to wait before first probe (lets ESP32 boot).
    """
    time.sleep(settle_time)
    ports = sorted(glob_mod.glob("/dev/ttyACM*"))
    if logger:
        logger.info(f"Serial ports found: {ports}")

    found = {}
    results = {}
    threads = []

    def _worker(p):
        results[p] = _probe_port(p, deadline_s=deadline_s)

    for port in ports:
        t = threading.Thread(target=_worker, args=(port,),
                             name=f"probe-{port}", daemon=True)
        t.start()
        threads.append(t)

    # Cap total wall time so a wedged port can't hang startup forever.
    # +2 s slack for thread start / Python overhead.
    for t in threads:
        t.join(timeout=deadline_s + 2.0)

    for port in ports:
        if port not in results:
            if logger:
                logger.warn(f"Probe thread for {port} did not finish in time")
            continue
        _port, identity, ser, err = results[port]
        if identity:
            if logger:
                logger.info(f"Found '{identity}' on {port}")
            found[identity] = ser
        else:
            if logger:
                logger.warn(f"Could not probe {port}: {err}")

    return found


class ESP32:
    """Threaded serial handler for an ESP32 controlling STS/SCS servos."""

    def __init__(self, ser: serial.Serial, identity: str):
        self.ser = ser
        self.identity = identity
        self.ids: List[int] = []

        self._read_buf = b""
        self._positions: Dict[int, int] = {}
        self._pending_reads: deque = deque()
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # --- Threading ---------------------------------------------------------

    def start(self):
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True,
            name=f"esp32-{self.identity}")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _poll_loop(self):
        while self._running:
            with self._lock:
                avail = self.ser.in_waiting
                if avail > 0:
                    self._read_buf += self.ser.read(avail)
                while b"\n" in self._read_buf:
                    line, self._read_buf = self._read_buf.split(b"\n", 1)
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        self._handle_line(text)
            time.sleep(0.005)

    # --- Discovery (blocking — call before start()) -----------------------

    def get_ids(self, prefix="", timeout=10.0):
        """STS-style ping. Blocks until 'Total servos found:' or timeout."""
        self.ids = []
        cmd = f"{prefix}ping\r\n".encode()
        self.ser.write(cmd)
        self.ser.flush()
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.ser.readline()
            if raw:
                text = raw.decode("utf-8", errors="replace").strip()
                if text.startswith("Servo ID "):
                    self.ids.append(int(text.split(" ")[2]))
                elif text.startswith("Total servos found:"):
                    return
                elif text.startswith("Error"):
                    self.ser.write(cmd)
                    self.ser.flush()
            time.sleep(0.01)

    def add_ids(self, extra: List[int]):
        """Manually register servo IDs (e.g. SCS motors not found by STS ping)."""
        for sid in extra:
            if sid not in self.ids:
                self.ids.append(sid)

    # --- Commands ----------------------------------------------------------

    def set_pos(self, servo_id: int, position: int, prefix: str = ""):
        if servo_id not in self.ids:
            return -1
        try:
            position = int(position)
        except Exception:
            return -1
        if position < 0 or position > 4096:
            return -1
        with self._lock:
            self.ser.write(
                f"{prefix}{servo_id},{position}\r\n"
                .encode("utf-8", errors="replace"))
        return 0

    def request_pos(self, servo_id: int, prefix: str = ""):
        if servo_id not in self.ids:
            return
        with self._lock:
            # Drop stale entries if queue grows too large (desync protection)
            while len(self._pending_reads) > 5:
                self._pending_reads.popleft()
            self._pending_reads.append(servo_id)
            self.ser.write(
                f"{prefix}pos {servo_id}\r\n"
                .encode("utf-8", errors="replace"))

    def get_pos(self, servo_id: int) -> int:
        with self._lock:
            return self._positions.get(servo_id, -1)

    # --- Response parsing --------------------------------------------------

    def _handle_line(self, line: str):
        """Parse one response line. Handles both STS and SCS format.

        STS: "Servo ID: 8", "Position: 2048", ...
        SCS: "Position:2048", ...

        Uses a FIFO queue of pending servo IDs. STS responses override from
        the "Servo ID:" line; SCS responses consume from the queue front.
        """
        if line.startswith("Servo ID:"):
            try:
                sid = int(line.split(":")[1].strip())
                # STS response includes the ID — discard the matching queue
                # entry (if any) and use the ID from the response directly.
                if self._pending_reads and self._pending_reads[0] == sid:
                    self._pending_reads.popleft()
                self._current_read = sid
            except (ValueError, IndexError):
                pass
        elif line.startswith("Position"):
            try:
                pos = int(line.split(":")[1].strip())
                # Use _current_read (set by "Servo ID:" for STS) or pop from
                # the queue (SCS responses that don't include a servo ID line).
                sid = getattr(self, '_current_read', None)
                if sid is None and self._pending_reads:
                    sid = self._pending_reads.popleft()
                if sid is not None:
                    self._positions[sid] = pos
                self._current_read = None
            except (ValueError, IndexError):
                pass
        elif line.startswith("Failed to get status") or line.startswith("FeedBack error"):
            # Consume the failed request from the queue
            if getattr(self, '_current_read', None) is None and self._pending_reads:
                self._pending_reads.popleft()
            self._current_read = None


class XiaomiESP32:
    """Threaded serial handler for Xiaomi CyberGear motors."""

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True,
            name="esp32-xiaomi")
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _poll_loop(self):
        """Drain serial to prevent buffer overflow."""
        while self._running:
            with self._lock:
                avail = self.ser.in_waiting
                if avail > 0:
                    self.ser.read(avail)
            time.sleep(0.005)

    def set_speed(self, left: float, right: float):
        with self._lock:
            self.ser.write(
                f"{left},{right}\r\n"
                .encode("utf-8", errors="replace"))
