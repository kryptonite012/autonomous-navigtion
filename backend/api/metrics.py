"""Prometheus text-format metrics (scrape with Prometheus/Grafana on the LAN)."""

from __future__ import annotations

from fastapi import APIRouter, Depends
from fastapi.responses import PlainTextResponse

from backend.api.deps import AppContainer, container

router = APIRouter(tags=["metrics"])


def _line(lines: list[str], name: str, value, help_: str, kind: str = "gauge", labels: str = "") -> None:
    if value is None:
        return
    if not any(l.startswith(f"# HELP {name} ") for l in lines):
        lines.append(f"# HELP {name} {help_}")
        lines.append(f"# TYPE {name} {kind}")
    lines.append(f"{name}{{{labels}}} {float(value)}" if labels else f"{name} {float(value)}")


@router.get("/metrics", response_class=PlainTextResponse, include_in_schema=False)
def metrics(c: AppContainer = Depends(container)):
    """No secrets are exposed; access is still limited by the network allow-list."""
    r = c.robot
    t = r.telemetry()
    st = r.safety.status
    lines: list[str] = []
    sim = 'mode="simulation"' if r.simulated else 'mode="hardware"'
    _line(lines, "nexus_up", 1, "robot runtime is running", labels=sim)
    _line(lines, "nexus_battery_voltage", t["battery_voltage"], "battery voltage (V)")
    _line(lines, "nexus_cpu_temperature_celsius", t["cpu_temperature"], "CPU temperature")
    _line(lines, "nexus_cpu_percent", t["cpu_percent"], "CPU load")
    _line(lines, "nexus_control_loop_hz", r.loop_hz, "measured control loop rate")
    _line(lines, "nexus_control_loop_overruns_total", r.loop_overruns, "control cycles that overran", "counter")
    _line(lines, "nexus_linear_velocity_mps", t["linear_velocity"], "estimated linear velocity")
    _line(lines, "nexus_estop_active", int(st.estop_hardware or st.estop_software), "emergency stop latched")
    _line(lines, "nexus_safety_faults", len(st.faults), "active safety faults")
    _line(lines, "nexus_stale_sensors", len(st.stale_sensors), "range sensors without recent data")
    vs = r.vision.status()
    _line(lines, "nexus_vision_fps", vs["fps"], "vision pipeline rate")
    _line(lines, "nexus_vision_process_ms", vs["process_ms"], "detection time per frame")
    _line(lines, "nexus_vision_active_tracks", vs["active_tracks"], "objects currently tracked")
    for status, n in r.sorter.status()["statistics"].items():
        _line(lines, "nexus_sort_operations_total", n, "sorting outcomes this session", "counter",
              f'status="{status}"')
    p = r.pose()
    _line(lines, "nexus_pose_std_m", p.get("std_xy"), "position uncertainty (1 sigma)")
    _line(lines, "nexus_pose_valid", int(p["valid"]), "pose usable for navigation")
    _line(lines, "nexus_navigation_state", 1, "current navigation state", labels=f'state="{r.navigator.state.value}"')
    _line(lines, "nexus_websocket_clients", len(c.hub.clients), "connected dashboard clients")
    return "\n".join(lines) + "\n"
