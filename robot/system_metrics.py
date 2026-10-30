"""Host metrics (CPU temperature, load, memory)."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

try:
    import psutil
except ImportError:  # psutil is optional
    psutil = None


def cpu_temperature() -> Optional[float]:
    p = Path("/sys/class/thermal/thermal_zone0/temp")
    try:
        return int(p.read_text().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def throttled_flags() -> Optional[int]:
    """Raspberry Pi firmware throttling bits (under-voltage, thermal)."""
    import shutil
    import subprocess
    if not shutil.which("vcgencmd"):
        return None
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1).stdout
        return int(out.strip().split("=")[1], 16)
    except Exception:
        return None


def snapshot() -> dict:
    data = {"cpu_temperature": cpu_temperature(), "cpu_percent": None, "ram_percent": None,
            "ram_used_mb": None, "disk_percent": None}
    if psutil is not None:
        data["cpu_percent"] = psutil.cpu_percent(interval=None)
        vm = psutil.virtual_memory()
        data["ram_percent"] = vm.percent
        data["ram_used_mb"] = round(vm.used / 1e6)
        try:
            data["disk_percent"] = psutil.disk_usage("/").percent
        except OSError:
            pass
    return data
