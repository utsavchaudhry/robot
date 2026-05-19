from typing import Optional

from motor_handler.esp32 import ESP32


class Motor:
    """Maps a named joint to a servo on a specific ESP32."""

    def __init__(self, name: str, esp: ESP32, servo_id: int,
                 angle_min: float, angle_max: float,
                 esp_min: float, esp_max: float,
                 cmd_prefix: str = "",
                 flip: bool = False,
                 offset: float = 0.0,
                 enabled: bool = False):
        self.name = name
        self.esp = esp
        self.esp_id = servo_id
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.esp_min = esp_min
        self.esp_max = esp_max
        self.cmd_prefix = cmd_prefix
        self.flip = flip
        self.offset = offset
        self.enabled = enabled

    def set_angle(self, value: float):
        """Send a position command (only if enabled)."""
        if not self.enabled:
            return
        value = max(self.angle_min, min(self.angle_max, value + self.offset))
        t = (value - self.angle_min) / (self.angle_max - self.angle_min)
        if self.flip:
            t = 1.0 - t
        raw = t * (self.esp_max - self.esp_min) + self.esp_min
        self.esp.set_pos(self.esp_id, int(raw), prefix=self.cmd_prefix)

    def get_angle(self) -> Optional[float]:
        """Read last known position (always works regardless of enabled)."""
        raw = self.esp.get_pos(self.esp_id)
        if raw < 0:
            return None
        t = (raw - self.esp_min) / (self.esp_max - self.esp_min)
        if self.flip:
            t = 1.0 - t
        return t * (self.angle_max - self.angle_min) + self.angle_min - self.offset
