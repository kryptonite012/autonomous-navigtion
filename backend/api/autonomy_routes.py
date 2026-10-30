"""Localisation, missions and the object map."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request

from backend.api.deps import AppContainer, CurrentUser, audit, container, operator, viewer
from backend.api.schemas import CoverageRequest, InitialPose, MissionIn, ScheduleIn
from backend.api.robot_routes import _run
from robot.navigation.coverage import coverage_waypoints
from robot.navigation.missions import Mission, Waypoint
from robot.navigation.scheduler import Schedule

localization = APIRouter(prefix="/api/localization", tags=["localization"])
missions = APIRouter(prefix="/api/missions", tags=["missions"])
objects = APIRouter(prefix="/api/vision/objects", tags=["vision"])
schedules = APIRouter(prefix="/api/schedules", tags=["missions"])


# ---------------------------------------------------------------- localisation
@localization.get("/status")
def loc_status(particles: bool = False, _: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    s = c.robot.localizer.status(with_particles=particles)
    s["pose"] = c.robot.pose()
    return s


@localization.post("/initial-pose")
def loc_initial_pose(body: InitialPose, request: Request, _: CurrentUser = Depends(operator),
                     c: AppContainer = Depends(container)):
    """Tell the robot where it is on the map (like '2D Pose Estimate' in RViz)."""
    _run(c.robot.reset_pose, body.x, body.y, body.theta)
    audit(c, request, "initial_pose", body.model_dump_json())
    return {"ok": True, "pose": c.robot.pose()}


@localization.post("/global")
def loc_global(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    _run(c.robot.global_localization)
    audit(c, request, "global_localization")
    return {"ok": True}


# -------------------------------------------------------------------- missions
@missions.get("")
def mission_list(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    return [m.as_dict() for m in sorted(c.robot.mission_library.values(), key=lambda m: m.name)]


@missions.get("/status")
def mission_status(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    return c.robot.missions.status()


@missions.post("/coverage")
def mission_coverage(body: CoverageRequest, request: Request, _: CurrentUser = Depends(operator),
                     c: AppContainer = Depends(container)):
    """Generate a lawnmower sweep over the free space of the current map."""
    r = c.robot
    if r.mapper.grid.updates == 0 and r.mapper.loaded_name is None:
        raise HTTPException(409, "no map: load or build one first")
    try:
        pts = coverage_waypoints(r.mapper.grid, body.spacing, r.navigator.inflation + 0.02)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    m = Mission(name=body.name, waypoints=[Waypoint(x=x, y=y, dwell_s=body.dwell_s) for x, y in pts],
                loop=body.loop, return_home=body.return_home, on_failure="skip")
    try:
        c.service.save_mission(m)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    audit(c, request, "mission_coverage", f"{body.name}: {len(pts)} waypoints, spacing {body.spacing}")
    return {"ok": True, "mission": m.as_dict()}


@missions.put("/{name}")
def mission_save(name: str, body: MissionIn, request: Request, _: CurrentUser = Depends(operator),
                 c: AppContainer = Depends(container)):
    if c.robot.missions.running and c.robot.missions.p.mission and c.robot.missions.p.mission.name == name:
        raise HTTPException(409, "cannot edit the running mission")
    m = Mission(name=name, waypoints=[Waypoint(**w.model_dump()) for w in body.waypoints], loop=body.loop,
                max_loops=body.max_loops, return_home=body.return_home, on_failure=body.on_failure)
    try:
        c.service.save_mission(m)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    audit(c, request, "mission_save", name)
    return {"ok": True, "mission": m.as_dict()}


@missions.delete("/{name}")
def mission_delete(name: str, request: Request, _: CurrentUser = Depends(operator),
                   c: AppContainer = Depends(container)):
    if c.robot.missions.running and c.robot.missions.p.mission and c.robot.missions.p.mission.name == name:
        raise HTTPException(409, "stop the mission first")
    if not c.service.delete_mission(name):
        raise HTTPException(404, "unknown mission")
    audit(c, request, "mission_delete", name)
    return {"ok": True}


@missions.post("/{name}/start")
def mission_start(name: str, request: Request, _: CurrentUser = Depends(operator),
                  c: AppContainer = Depends(container)):
    _run(c.robot.start_mission, name)
    audit(c, request, "mission_start", name)
    return {"ok": True, "status": c.robot.missions.status()}


@missions.post("/stop")
def mission_stop(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.stop_mission()
    audit(c, request, "mission_stop")
    return {"ok": True}


# ------------------------------------------------------------------ object map
@objects.get("")
def objects_list(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    return {"calibrated": c.robot.vision.detector.calibrated, "objects": c.robot.objects.list()}


@objects.delete("")
def objects_clear(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.objects.clear()
    audit(c, request, "objects_clear")
    return {"ok": True}


# ------------------------------------------------------------------- schedules
@schedules.get("")
def schedule_list(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    return [s.as_dict() for s in sorted(c.robot.scheduler.schedules.values(), key=lambda s: s.name)]


@schedules.put("/{name}")
def schedule_save(name: str, body: ScheduleIn, request: Request, _: CurrentUser = Depends(operator),
                  c: AppContainer = Depends(container)):
    old = c.robot.scheduler.schedules.get(name)
    sched = Schedule(name=name, mission=body.mission, time=body.time, days=body.days, enabled=body.enabled,
                     last_run=old.last_run if old else None, last_result=old.last_result if old else None)
    try:
        c.service.save_schedule(sched)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    audit(c, request, "schedule_save", f"{name}: {body.mission} at {body.time} {body.days}")
    return {"ok": True, "schedule": sched.as_dict()}


@schedules.delete("/{name}")
def schedule_delete(name: str, request: Request, _: CurrentUser = Depends(operator),
                    c: AppContainer = Depends(container)):
    if not c.service.delete_schedule(name):
        raise HTTPException(404, "unknown schedule")
    audit(c, request, "schedule_delete", name)
    return {"ok": True}
