"""Flight recorder: start/stop raw sensor logging, list, download, delete."""

from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import FileResponse

from backend.api.deps import AppContainer, CurrentUser, audit, container, operator, viewer
from backend.api.robot_routes import _run
from backend.api.schemas import RecordingName

router = APIRouter(prefix="/api/recordings", tags=["recordings"])


@router.get("")
def recordings_list(_: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    return {"status": c.robot.recorder.status(), "recordings": c.robot.recorder.list()}


@router.post("/start")
def recording_start(body: RecordingName, request: Request, _: CurrentUser = Depends(operator),
                    c: AppContainer = Depends(container)):
    path = _run(c.robot.start_recording, body.name)
    audit(c, request, "recording_start", body.name)
    return {"ok": True, "file": path}


@router.post("/stop")
def recording_stop(request: Request, _: CurrentUser = Depends(operator), c: AppContainer = Depends(container)):
    name = c.robot.recorder.stop()
    if name is None:
        raise HTTPException(409, "not recording")
    audit(c, request, "recording_stop", name)
    return {"ok": True, "name": name}


@router.get("/{name}/download")
def recording_download(name: str, _: CurrentUser = Depends(viewer), c: AppContainer = Depends(container)):
    try:
        p = c.robot.recorder.path(name)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    if not p.exists():
        raise HTTPException(404, "recording not found")
    if name == c.robot.recorder.name:
        raise HTTPException(409, "stop the recording first")
    return FileResponse(p, filename=p.name, media_type="application/gzip")


@router.delete("/{name}")
def recording_delete(name: str, request: Request, _: CurrentUser = Depends(operator),
                     c: AppContainer = Depends(container)):
    try:
        c.robot.recorder.delete(name)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    except FileNotFoundError:
        raise HTTPException(404, "recording not found")
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    audit(c, request, "recording_delete", name)
    return {"ok": True}
