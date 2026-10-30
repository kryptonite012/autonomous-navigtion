"""Vision and sorting endpoints, including the MJPEG camera stream."""

from __future__ import annotations

import asyncio
import time

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import Response, StreamingResponse
from sqlalchemy import func, select

from backend.api.deps import AppContainer, CurrentUser, audit, container, operator, viewer
from backend.api.schemas import (CalibrationRequest, ServoCalibration, SortingConfig, VisionSettings,
                                 WhiteBalanceRequest)
from backend.database.models import SortingRecord
from robot.config import save_section
from robot.runtime import CommandRejected
from robot.vision.calibration import suggest_range, validate_range
from robot.vision.detector import ColorDetector

vision = APIRouter(prefix="/api/vision", tags=["vision"])
sorting = APIRouter(prefix="/api/sorting", tags=["sorting"])


# ---------------------------------------------------------------------- vision
@vision.get("/status")
def vision_status(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    s = c.robot.vision.status()
    s["colors"] = c.robot.vision.params.colors_as_config()
    s["enabled_colors"] = c.robot.vision.params.enabled_colors
    s["white_balance"] = {"mode": c.robot.vision.params.white_balance, "gains": list(c.robot.vision.params.wb_gains)}
    s["settings"] = {"min_area_px": c.robot.vision.params.min_area_px,
                     "min_confidence": c.robot.vision.params.min_confidence,
                     "blur_kernel": c.robot.vision.params.blur_kernel,
                     "morph_kernel": c.robot.vision.params.morph_kernel}
    return s


@vision.post("/start")
def vision_start(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    try:
        c.robot.vision.start()
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    return {"ok": True}


@vision.post("/stop")
def vision_stop(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    if c.robot.sorter.running:
        raise HTTPException(409, "sorting needs the camera; stop sorting first")
    c.robot.vision.stop()
    return {"ok": True}


@vision.get("/detections")
def vision_detections(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    dets, ts = c.robot.vision.latest()
    return {"timestamp": ts, "age_s": round(time.time() - ts, 3) if ts else None,
            "items": [d.as_dict() for d in dets]}


@vision.post("/calibrate")
def vision_calibrate(body: CalibrationRequest, request: Request, _: CurrentUser = Depends(operator),
                     c: AppContainer = Depends(container)):
    params = c.robot.vision.params
    if body.ranges:
        try:
            ranges = [validate_range(r.lower, r.upper) for r in body.ranges]
        except ValueError as exc:
            raise HTTPException(422, str(exc))
    elif body.roi:
        frame = c.robot.vision.frame_copy()
        if frame is None:
            raise HTTPException(409, "no camera frame available")
        try:
            ranges = suggest_range(frame, tuple(body.roi))
        except ValueError as exc:
            raise HTTPException(422, str(exc))
    else:
        raise HTTPException(422, "provide either ranges or roi")
    params.colors[body.color] = ranges
    if body.persist:
        vc = dict(c.robot.cfg["vision"])
        vc["colors"] = params.colors_as_config()
        save_section(c.config_path, "vision", vc)
    audit(c, request, "vision_calibrate", f"{body.color}: {[(r.lower, r.upper) for r in ranges]}")
    return {"ok": True, "color": body.color,
            "ranges": [{"lower": list(r.lower), "upper": list(r.upper)} for r in ranges]}


@vision.post("/white-balance")
def vision_white_balance(body: WhiteBalanceRequest, request: Request, _: CurrentUser = Depends(operator),
                         c: AppContainer = Depends(container)):
    """Correct coloured lighting. 'reference': hold a white or light grey sheet
    in the ROI; gains making it neutral are stored. Recalibrate HSV ranges
    afterwards if they were tuned without white balance."""
    p = c.robot.vision.params
    if body.mode == "reference":
        if body.roi is None:
            raise HTTPException(422, "roi required for reference white balance")
        frame = c.robot.vision.frame_copy()
        if frame is None:
            raise HTTPException(409, "no camera frame available")
        try:
            p.wb_gains = ColorDetector.reference_gains(frame, tuple(body.roi))
        except ValueError as exc:
            raise HTTPException(422, str(exc))
    p.white_balance = body.mode
    if body.persist:
        vc = dict(c.robot.cfg["vision"])
        vc["white_balance"] = p.white_balance
        vc["wb_gains"] = list(p.wb_gains)
        save_section(c.config_path, "vision", vc)
    audit(c, request, "vision_white_balance", f"{body.mode} gains={p.wb_gains}")
    return {"ok": True, "mode": p.white_balance, "gains": list(p.wb_gains)}


@vision.put("/settings")
def vision_settings(body: VisionSettings, request: Request, _: CurrentUser = Depends(operator),
                    c: AppContainer = Depends(container)):
    p = c.robot.vision.params
    changes = body.model_dump(exclude_none=True)
    if "enabled_colors" in changes:
        unknown = [col for col in changes["enabled_colors"] if col not in p.colors]
        if unknown:
            raise HTTPException(422, f"unknown colours: {unknown}")
    for k, v in changes.items():
        setattr(p, k, v)
    audit(c, request, "vision_settings", str(changes))
    return {"ok": True}


@vision.get("/snapshot.jpg")
def vision_snapshot(kind: str = "annotated", _: CurrentUser = Depends(viewer),
                    c: AppContainer = Depends(container)):
    data = c.robot.vision.jpeg(kind)
    if data is None:
        raise HTTPException(404, "no frame available")
    return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@vision.get("/stream.mjpg")
async def vision_stream(kind: str = "annotated", fps: int = 10, _: CurrentUser = Depends(viewer),
                        c: AppContainer = Depends(container)):
    if kind not in ("raw", "annotated", "mask"):
        raise HTTPException(422, "kind must be raw, annotated or mask")
    if not c.robot.vision.available:
        raise HTTPException(409, "no camera available")
    period = 1.0 / max(1, min(fps, 30))

    async def gen():
        last = None
        while True:
            data = c.robot.vision.jpeg(kind)
            if data is not None and data is not last:
                last = data
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(data)).encode()
                       + b"\r\n\r\n" + data + b"\r\n")
            await asyncio.sleep(period)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame",
                             headers={"Cache-Control": "no-store"})


# --------------------------------------------------------------------- sorting
def _sorting_cmd(fn):
    try:
        fn()
    except CommandRejected as exc:
        raise HTTPException(409, str(exc))
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))


@sorting.post("/start")
def sorting_start(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    _sorting_cmd(c.robot.start_sorting)
    audit(c, request, "sorting_start")
    return {"ok": True, "status": c.robot.sorter.status()}


@sorting.post("/pause")
def sorting_pause(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    if not c.robot.sorter.running:
        raise HTTPException(409, "sorting is not running")
    c.robot.sorter.pause()
    return {"ok": True}


@sorting.post("/resume")
def sorting_resume(_: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    if not c.robot.sorter.running:
        raise HTTPException(409, "sorting is not running")
    c.robot.sorter.resume()
    return {"ok": True}


@sorting.post("/stop")
def sorting_stop(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.stop_sorting()
    audit(c, request, "sorting_stop")
    return {"ok": True}


@sorting.get("/status")
def sorting_status(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    return c.robot.sorter.status()


@sorting.get("/statistics")
def sorting_statistics(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    with c.db.session() as s:
        by_status = dict(s.execute(select(SortingRecord.sorting_status, func.count())
                                   .group_by(SortingRecord.sorting_status)).all())
        by_color = {}
        for color, st, n in s.execute(select(SortingRecord.detected_color, SortingRecord.sorting_status,
                                             func.count()).group_by(SortingRecord.detected_color,
                                                                    SortingRecord.sorting_status)).all():
            by_color.setdefault(color, {})[st] = n
    total = sum(by_status.values())
    return {"session": c.robot.sorter.status()["statistics"], "all_time": by_status, "by_color": by_color,
            "all_time_total": total,
            "all_time_success_rate": round(by_status.get("success", 0) / total, 3) if total else None}


@sorting.get("/history")
def sorting_history(limit: int = 100, _: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    limit = max(1, min(limit, 1000))
    with c.db.session() as s:
        rows = s.execute(select(SortingRecord).order_by(SortingRecord.id.desc()).limit(limit)).scalars()
        return [{"id": r.id, "object_id": r.object_id, "detected_color": r.detected_color,
                 "target_bin": r.target_bin, "sorting_status": r.sorting_status, "attempts": r.attempts,
                 "message": r.message, "timestamp": r.timestamp.isoformat()} for r in rows]


@sorting.put("/config")
def sorting_config(body: SortingConfig, request: Request, _: CurrentUser = Depends(operator),
                   c: AppContainer = Depends(container)):
    if c.robot.sorter.running:
        raise HTTPException(409, "stop sorting before changing the configuration")
    try:
        c.robot.sorter.set_config(body.bins, body.unclassified_bin, body.bin_capacity)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    if body.persist:
        sc = dict(c.robot.cfg["sorting"])
        sc["bins"] = c.robot.sorter.bins
        sc["unclassified_bin"] = c.robot.sorter.unclassified_bin
        sc["bin_capacity"] = c.robot.sorter.capacity
        save_section(c.config_path, "sorting", sc)
    audit(c, request, "sorting_config", body.model_dump_json())
    return {"ok": True, "status": c.robot.sorter.status()}


@sorting.post("/bins/empty")
def sorting_bins_empty(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    c.robot.sorter.empty_bins()
    audit(c, request, "bins_emptied")
    return {"ok": True}


@sorting.post("/servo")
def sorting_servo(body: ServoCalibration, request: Request, _: CurrentUser = Depends(operator),
                  c: AppContainer = Depends(container)):
    arm = c.robot.sorter.arm
    if arm is None:
        raise HTTPException(409, "servo controller unavailable")
    if c.robot.sorter.running:
        raise HTTPException(409, "stop sorting before calibrating servos")
    try:
        arm.calibrate(body.channel, body.angle)
    except (ValueError, IOError) as exc:
        raise HTTPException(409, str(exc))
    audit(c, request, "servo_calibrate", f"{body.channel}={body.angle}")
    return {"ok": True}
