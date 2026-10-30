"""Simulation control: fault injection and scenario setup.

Every route refuses with 409 unless the robot runs in simulation mode, so
none of this can affect a physical robot.
"""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request

from backend.api.deps import AppContainer, CurrentUser, audit, container, operator, viewer
from backend.api.schemas import MapName, SimFaults, SimObstacle, SimScatter, SimTeleport

router = APIRouter(prefix="/api/simulation", tags=["simulation"])


def _world(c: AppContainer):
    if not c.robot.simulated or c.robot.hw.world is None:
        raise HTTPException(409, "only available in simulation mode")
    return c.robot.hw.world


@router.get("/state")
def sim_state(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    w = _world(c)
    snap = w.snapshot()
    hw = c.robot.hw
    snap["faults"].update({
        "servo_fault": bool(getattr(hw.servos, "failed", False)),
        "camera_covered": bool(getattr(hw.camera, "covered", False)),
        "disconnected_sensors": [s.name for s in hw.ranges if getattr(s, "disconnected", False)],
    })
    snap["sensors"] = [s.name for s in hw.ranges]
    est = c.robot.pose()
    snap["estimate"] = est
    snap["pose_error_m"] = round(((est["x"] - snap["robot"]["x"]) ** 2 + (est["y"] - snap["robot"]["y"]) ** 2) ** 0.5, 4)
    return snap


@router.post("/faults")
def sim_faults(body: SimFaults, request: Request, _: CurrentUser = Depends(operator),
               c: AppContainer = Depends(container)):
    w = _world(c)
    hw = c.robot.hw
    changes = body.model_dump(exclude_none=True)
    if body.motor_fault is not None:
        w.motor_fault = body.motor_fault
    if body.encoder_fault is not None:
        w.encoder_fault = body.encoder_fault
    if body.estop_pressed is not None:
        w.estop_pressed = body.estop_pressed
    if body.battery_voltage is not None:
        w.battery_v = body.battery_voltage
    if body.servo_fault is not None and hw.servos is not None:
        hw.servos.failed = body.servo_fault
    if body.camera_covered is not None and hw.camera is not None:
        hw.camera.covered = body.camera_covered
    if body.disconnected_sensors is not None:
        names = {s.name for s in hw.ranges}
        unknown = set(body.disconnected_sensors) - names
        if unknown:
            raise HTTPException(422, f"unknown sensors: {sorted(unknown)}")
        for s in hw.ranges:
            s.disconnected = s.name in body.disconnected_sensors
    audit(c, request, "sim_faults", str(changes))
    c.robot.events.publish("sim_fault_injected", "warning", f"simulation faults set: {changes}")
    return {"ok": True}


@router.post("/obstacles")
def sim_add_obstacle(body: SimObstacle, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    _world(c).add_dynamic_obstacle(body.x, body.y, body.radius, body.vx, body.vy)
    return {"ok": True}


@router.delete("/obstacles")
def sim_clear_obstacles(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    _world(c).clear_dynamic()
    return {"ok": True}


@router.post("/objects/scatter")
def sim_scatter(body: SimScatter, request: Request, _: CurrentUser = Depends(operator),
                c: AppContainer = Depends(container)):
    if c.robot.sorter.running:
        raise HTTPException(409, "stop sorting first")
    try:
        objs = _world(c).scatter_objects(body.count, body.colors)
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    c.robot.objects.clear()
    audit(c, request, "sim_scatter", f"{body.count} objects")
    return {"ok": True, "objects": len(objs)}


@router.post("/teleport")
def sim_teleport(body: SimTeleport, request: Request, _: CurrentUser = Depends(operator),
                 c: AppContainer = Depends(container)):
    """Kidnapped-robot test: move the robot without updating odometry."""
    if c.robot.navigator.active or c.robot.sorter.running:
        raise HTTPException(409, "stop the active task first")
    try:
        _world(c).teleport(body.x, body.y, body.theta)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    audit(c, request, "sim_teleport", body.model_dump_json())
    c.robot.events.publish("sim_teleport", "warning", "robot moved without odometry (kidnapped robot test)")
    return {"ok": True}


@router.post("/reset")
def sim_reset(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.stop()
    w = _world(c)
    w.reset()
    w.clear_dynamic()
    w.motor_fault = w.encoder_fault = w.estop_pressed = False
    w.battery_v = 8.2
    for s in c.robot.hw.ranges:
        s.disconnected = False
    if c.robot.hw.servos is not None:
        c.robot.hw.servos.failed = False
    if c.robot.hw.camera is not None:
        c.robot.hw.camera.covered = False
    c.robot.reset_pose(*w.start_pose)
    audit(c, request, "sim_reset")
    return {"ok": True}


@router.post("/reference-map")
def sim_reference_map(body: MapName, request: Request, _: CurrentUser = Depends(operator),
                      c: AppContainer = Depends(container)):
    """Save a ground-truth map of the simulated room (no exploration needed)."""
    from robot.mapping.occupancy_grid import OccupancyGrid
    w = _world(c)
    g = OccupancyGrid.from_config(c.robot.cfg["mapping"])
    w.reference_grid(g)
    path = c.robot.mapper.store.save(g, body.name, {"experimental": False, "source": "simulation_ground_truth"})
    c.service.register_map(body.name, str(path), g.resolution, False)
    audit(c, request, "sim_reference_map", body.name)
    return {"ok": True, "file": path.name}
