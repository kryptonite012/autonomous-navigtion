"""System, robot, navigation, mapping and sensor endpoints."""

from __future__ import annotations

import os
import signal
import threading
import time

import cv2
import numpy as np
from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import FileResponse, Response
from sqlalchemy import select

from backend.api.deps import (AppContainer, CurrentUser, admin, audit, container, motion_rate_limit,
                              operator, viewer)
from backend.api.schemas import (FaultClear, GoalRequest, MapName, MappingStart, MapRename, NamedLocation,
                                 PoseReset, SafetySettings, Teleop)
from backend.database.models import MapRecord, SystemEvent, Telemetry
from robot import system_metrics
from robot.mapping.occupancy_grid import FREE, OCCUPIED
from robot.runtime import CommandRejected

system = APIRouter(prefix="/api/system", tags=["system"])
robot_r = APIRouter(prefix="/api/robot", tags=["robot"])
nav = APIRouter(prefix="/api/navigation", tags=["navigation"])
mapping = APIRouter(prefix="/api/mapping", tags=["mapping"])
sensors = APIRouter(prefix="/api/sensors", tags=["sensors"])
logs = APIRouter(prefix="/api/logs", tags=["logs"])


def _run(fn, *args, **kwargs):
    try:
        return fn(*args, **kwargs)
    except CommandRejected as exc:
        raise HTTPException(409, str(exc))
    except (KeyError, ValueError) as exc:
        raise HTTPException(422, str(exc).strip("'\""))
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))


# ---------------------------------------------------------------------- system
@system.get("/status")
def system_status(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    r = c.robot
    return {
        "robot": r.cfg["robot"]["name"],
        "hardware_mode": r.hw.mode,
        "mode": r.mode.value,
        "uptime_s": round(time.time() - r.started_at, 1),
        "metrics": system_metrics.snapshot(),
        "throttled": system_metrics.throttled_flags(),
        "control_loop_hz": round(r.loop_hz, 1),
        "control_loop_overruns": r.loop_overruns,
        "websocket_clients": len(c.hub.clients),
        "database_version": c.db.version(),
        "capabilities": r.caps,
    }


@system.get("/health")
def system_health(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    devices = c.robot.health()
    bad = [d for d in devices if d["state"] in ("fault", "missing")]
    return {"overall": "ok" if not bad else ("degraded" if len(bad) < len(devices) else "fault"),
            "devices": devices, "safety": c.robot.safety.status.as_dict()}


@system.post("/shutdown")
def system_shutdown(request: Request, poweroff: bool = False, _: CurrentUser = Depends(admin),
                    c: AppContainer = Depends(container)):
    """Stop the robot safely and end the server process (systemd may restart it)."""
    audit(c, request, "system_shutdown", f"poweroff={poweroff}")
    c.robot.stop()

    def _later():
        time.sleep(0.5)
        if poweroff and c.robot.hw.mode == "hardware":
            os.system("sudo /usr/bin/systemctl poweroff")
        os.kill(os.getpid(), signal.SIGTERM)
    threading.Thread(target=_later, daemon=True).start()
    return {"ok": True, "message": "shutting down"}


@system.post("/backup")
def system_backup(request: Request, _: CurrentUser = Depends(admin), c: AppContainer = Depends(container)):
    try:
        path = c.db.backup()
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    audit(c, request, "database_backup", str(path))
    return {"ok": True, "file": path.name}


@system.get("/config")
def get_config(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    cfg = c.robot.cfg
    return {k: cfg[k] for k in ("robot", "chassis", "safety", "camera", "vision", "mapping", "navigation",
                                "sorting", "hardware")}


@system.put("/safety")
def update_safety(body: SafetySettings, request: Request, _: CurrentUser = Depends(admin),
                  c: AppContainer = Depends(container)):
    sc = c.robot.safety.cfg
    changes = body.model_dump(exclude_none=True)
    stop = changes.get("emergency_stop_distance_m", sc.emergency_stop_distance_m)
    slow = changes.get("slow_down_distance_m", sc.slow_down_distance_m)
    if slow <= stop:
        raise HTTPException(422, "slow-down distance must be larger than the stop distance")
    for k, v in changes.items():
        setattr(sc, k, v)
    audit(c, request, "safety_update", str(changes))
    return {"ok": True, "safety": sc.__dict__}


# ----------------------------------------------------------------------- robot
@robot_r.get("/state")
def robot_state(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    state = c.robot.state()
    state["ground_truth"] = c.robot.ground_truth()
    return state


@robot_r.post("/start")
def robot_start(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    """Switch to manual control mode (teleoperation)."""
    _run(c.robot.set_manual)
    audit(c, request, "robot_manual_mode")
    return {"ok": True, "mode": c.robot.mode.value}


@robot_r.post("/stop")
def robot_stop(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.stop()
    audit(c, request, "robot_stop")
    return {"ok": True}


@robot_r.post("/emergency-stop")
def robot_estop(request: Request, user: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    # Any authenticated user (even a viewer) may stop the robot.
    c.robot.emergency_stop(f"emergency stop from dashboard by {user.username}")
    audit(c, request, "emergency_stop")
    return {"ok": True, "safety": c.robot.safety.status.as_dict()}


@robot_r.post("/emergency-stop/reset")
def robot_estop_reset(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    ok, msg = c.robot.reset_emergency_stop()
    audit(c, request, "emergency_stop_reset", msg, ok)
    if not ok:
        raise HTTPException(409, msg)
    return {"ok": True}


@robot_r.post("/faults/clear")
def clear_fault(body: FaultClear, request: Request, _: CurrentUser = Depends(operator),
                c: AppContainer = Depends(container)):
    ok = c.robot.safety.clear_fault(body.fault)
    audit(c, request, "fault_clear", body.fault, ok)
    if not ok:
        raise HTTPException(409, "fault not active or condition still present")
    return {"ok": True}


@robot_r.post("/teleop")
def robot_teleop(body: Teleop, c: AppContainer = Depends(container), _: CurrentUser = Depends(motion_rate_limit)):
    """Velocity command; must be repeated at >= 2 Hz or the watchdog stops the robot."""
    safety = _run(c.robot.teleop, body.linear, body.angular)
    return {"ok": True, "safety": safety}


@robot_r.post("/pose/reset")
def robot_pose_reset(body: PoseReset, request: Request, _: CurrentUser = Depends(operator),
                     c: AppContainer = Depends(container)):
    _run(c.robot.reset_pose, body.x, body.y, body.theta)
    audit(c, request, "pose_reset", f"{body.x},{body.y},{body.theta}")
    return {"ok": True, "pose": c.robot.pose()}


# ------------------------------------------------------------------ navigation
@nav.get("/status")
def nav_status(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    s = c.robot.navigator.status()
    s["pose"] = c.robot.pose()
    return s


@nav.post("/start")
def nav_start(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    """Start autonomous frontier exploration (maps while it explores)."""
    _run(c.robot.start_mapping, True, False)
    audit(c, request, "navigation_explore")
    return {"ok": True}


@nav.post("/goal")
def nav_goal(body: GoalRequest, request: Request, _: CurrentUser = Depends(operator),
             c: AppContainer = Depends(container)):
    if body.location:
        _run(c.robot.navigate_to_named, body.location)
    elif body.x is not None and body.y is not None:
        _run(c.robot.navigate_to, body.x, body.y, body.theta)
    else:
        raise HTTPException(422, "provide x and y, or a location name")
    audit(c, request, "navigation_goal", body.model_dump_json(exclude_none=True))
    return {"ok": True, "navigation": c.robot.navigator.status()}


@nav.post("/pause")
def nav_pause(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    if not c.robot.pause_navigation():
        raise HTTPException(409, "navigation is not active")
    return {"ok": True}


@nav.post("/resume")
def nav_resume(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    if not _run(c.robot.resume_navigation):
        raise HTTPException(409, "navigation is not paused")
    return {"ok": True}


@nav.post("/cancel")
def nav_cancel(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.navigator.cancel("cancelled by operator")
    c.robot.safety.stop("navigation-cancel")
    return {"ok": True}


@nav.post("/locations")
def nav_add_location(body: NamedLocation, request: Request, _: CurrentUser = Depends(operator),
                     c: AppContainer = Depends(container)):
    c.robot.navigator.named_locations[body.name] = [body.x, body.y, body.theta]
    audit(c, request, "location_add", body.model_dump_json())
    return {"ok": True, "locations": c.robot.navigator.named_locations}


@nav.delete("/locations/{name}")
def nav_delete_location(name: str, request: Request, _: CurrentUser = Depends(operator),
                        c: AppContainer = Depends(container)):
    if c.robot.navigator.named_locations.pop(name, None) is None:
        raise HTTPException(404, "unknown location")
    audit(c, request, "location_delete", name)
    return {"ok": True}


# --------------------------------------------------------------------- mapping
@mapping.post("/start")
def map_start(body: MappingStart, request: Request, _: CurrentUser = Depends(operator),
              c: AppContainer = Depends(container)):
    _run(c.robot.start_mapping, body.explore, body.reset)
    audit(c, request, "mapping_start", body.model_dump_json())
    return {"ok": True, "mapping": c.robot.mapper.status()}


@mapping.post("/stop")
def map_stop(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.stop_mapping()
    return {"ok": True}


@mapping.post("/reset")
def map_reset(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    if c.robot.navigator.active:
        raise HTTPException(409, "stop navigation before resetting the map")
    c.robot.mapper.reset()
    audit(c, request, "mapping_reset")
    return {"ok": True}


@mapping.get("/current")
def map_current(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    snap = c.robot.mapper.snapshot()
    snap["status"] = c.robot.mapper.status()
    snap["pose"] = c.robot.pose()
    snap["path"] = c.robot.navigator.status()["path"]
    loc = c.robot.localizer.status(with_particles=True)
    snap["particles"] = loc.get("particle_sample", [])
    snap["objects"] = c.robot.objects.list()
    snap["ground_truth"] = c.robot.ground_truth()
    return snap


@mapping.get("/current.png")
def map_png(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    t = c.robot.mapper.grid.ternary()
    img = np.full(t.shape, 128, np.uint8)
    img[t == FREE] = 255
    img[t == OCCUPIED] = 0
    ok, buf = cv2.imencode(".png", np.flipud(img))
    return Response(buf.tobytes(), media_type="image/png")


@mapping.get("/maps")
def map_list(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    files = {m["name"]: m for m in c.robot.mapper.store.list()}
    with c.db.session() as s:
        recs = {r.map_name: r for r in s.execute(select(MapRecord)).scalars()}
    out = []
    for name, m in files.items():
        r = recs.get(name)
        out.append({"name": name, "modified": m["modified"],
                    "resolution": r.resolution if r else None,
                    "experimental": r.experimental if r else None,
                    "loaded": c.robot.mapper.loaded_name == name})
    return sorted(out, key=lambda m: -m["modified"])


@mapping.post("/save")
def map_save(body: MapName, request: Request, _: CurrentUser = Depends(operator),
             c: AppContainer = Depends(container)):
    if c.robot.mapper.grid.updates == 0 and c.robot.mapper.loaded_name is None:
        raise HTTPException(409, "map is empty")
    path = _run(c.robot.mapper.save, body.name)
    c.service.register_map(body.name, str(path), c.robot.mapper.grid.resolution, c.robot.mapper.experimental)
    audit(c, request, "map_save", body.name)
    return {"ok": True, "file": path.name}


@mapping.post("/load")
def map_load(body: MapName, request: Request, _: CurrentUser = Depends(operator),
             c: AppContainer = Depends(container)):
    try:
        _run(c.robot.load_map, body.name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    audit(c, request, "map_load", body.name)
    return {"ok": True, "mapping": c.robot.mapper.status()}


@mapping.post("/rename")
def map_rename(body: MapRename, request: Request, _: CurrentUser = Depends(operator),
               c: AppContainer = Depends(container)):
    try:
        path = c.robot.mapper.store.rename(body.old_name, body.new_name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except FileExistsError as exc:
        raise HTTPException(409, str(exc))
    c.service.rename_map(body.old_name, body.new_name, str(path))
    if c.robot.mapper.loaded_name == body.old_name:
        c.robot.mapper.loaded_name = body.new_name
    audit(c, request, "map_rename", f"{body.old_name} -> {body.new_name}")
    return {"ok": True}


@mapping.delete("/maps/{name}")
def map_delete(name: str, request: Request, _: CurrentUser = Depends(operator),
               c: AppContainer = Depends(container)):
    try:
        c.robot.mapper.store.delete(name)
    except FileNotFoundError as exc:
        raise HTTPException(404, str(exc))
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    c.service.delete_map(name)
    audit(c, request, "map_delete", name)
    return {"ok": True}


@mapping.get("/maps/{name}/download")
def map_download(name: str, fmt: str = "yaml", _: CurrentUser = Depends(viewer),
                 c: AppContainer = Depends(container)):
    try:
        p = c.robot.mapper.store.path(name)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    if fmt == "pgm":
        p = p.with_suffix(".pgm")
    if not p.exists():
        raise HTTPException(404, "map not found")
    return FileResponse(p, filename=p.name)


# --------------------------------------------------------------------- sensors
@sensors.get("/status")
def sensors_status(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    return {"devices": c.robot.health(), "stale": c.robot.safety.status.stale_sensors,
            "capabilities": c.robot.caps}


@sensors.get("/readings")
def sensors_readings(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    t = c.robot.telemetry()
    return {"timestamp": t["timestamp"], "ranges": [
        {"sensor": r.sensor, "distance_m": r.distance_m, "angle_rad": r.angle_rad, "valid": r.valid,
         "max_range_m": r.max_range_m, "age_s": round(time.time() - r.timestamp, 3)} for r in c.robot.ranges],
        "imu": t["imu"], "battery_voltage": t["battery_voltage"], "battery_percent": t["battery_percent"],
        "encoders": c.robot.hw.encoders.ticks() if c.robot.hw.encoders else None,
        "simulated": c.robot.simulated}


# ------------------------------------------------------------------------ logs
@logs.get("/events")
def events(limit: int = 200, severity: str | None = None, _: CurrentUser = Depends(viewer),
           c: AppContainer = Depends(container)):
    limit = max(1, min(limit, 2000))
    with c.db.session() as s:
        q = select(SystemEvent).order_by(SystemEvent.id.desc()).limit(limit)
        if severity:
            q = q.where(SystemEvent.severity == severity)
        return [{"id": e.id, "type": e.event_type, "severity": e.severity, "description": e.description,
                 "timestamp": e.timestamp.isoformat()} for e in s.execute(q).scalars()]


@logs.get("/telemetry")
def telemetry_history(minutes: int = 10, _: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    """Recent in-memory telemetry (2 Hz) for charts; older data is in the database."""
    cutoff = time.time() - max(1, min(minutes, 60)) * 60
    return [t for t in list(c.robot.telemetry_history) if t["timestamp"] >= cutoff]


@logs.get("/telemetry/stored")
def telemetry_stored(limit: int = 500, _: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    limit = max(1, min(limit, 5000))
    with c.db.session() as s:
        rows = s.execute(select(Telemetry).order_by(Telemetry.id.desc()).limit(limit)).scalars()
        return [{"battery_voltage": r.battery_voltage, "linear_velocity": r.linear_velocity,
                 "angular_velocity": r.angular_velocity, "cpu_temperature": r.cpu_temperature,
                 "timestamp": r.timestamp.isoformat()} for r in rows]
