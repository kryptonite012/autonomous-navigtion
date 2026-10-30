"""PROJECT NEXUS backend: FastAPI application entry point.

Run with:  python3 -m backend.main          (or uvicorn backend.main:app)
"""

from __future__ import annotations

import asyncio
import logging
import os
import secrets
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, Request, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from sqlalchemy import func, select

from backend.api import (auth_routes, autonomy_routes, metrics, recording_routes, robot_routes, sim_routes,
                         vision_routes)
from backend.api.deps import AppContainer, resolve_token
from backend.authentication.ratelimit import RateLimiter
from backend.authentication.security import NetworkPolicy, TokenService, hash_password
from backend.database.db import Database
from backend.database.models import User
from backend.services.robot_service import RobotService
from backend.websocket.hub import Client, Hub
from robot.config import ROOT_DIR, load_config
from robot.runtime import Robot

log = logging.getLogger("nexus")

DATA_DIR = ROOT_DIR / "data"


def _load_dotenv(path: Path = ROOT_DIR / ".env") -> None:
    """Minimal .env reader so the app has no python-dotenv dependency."""
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def _secret_file(name: str, nbytes: int = 48) -> str:
    """Read or create a secret stored with 0600 permissions in data/."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    p = DATA_DIR / name
    if p.exists():
        return p.read_text().strip()
    value = secrets.token_urlsafe(nbytes)
    fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as fh:
        fh.write(value)
    return value


def bootstrap_admin(db: Database) -> Optional[str]:
    """Create the first admin account when the users table is empty.

    The password comes from NEXUS_ADMIN_PASSWORD; if unset, a random one is
    generated and written to data/initial_admin_password (mode 0600).
    """
    with db.session() as s:
        if s.execute(select(func.count()).select_from(User)).scalar_one() > 0:
            return None
        username = os.environ.get("NEXUS_ADMIN_USERNAME", "admin")
        password = os.environ.get("NEXUS_ADMIN_PASSWORD") or ""
        note = None
        if not password:
            password = secrets.token_urlsafe(12) + "A1!"
            DATA_DIR.mkdir(parents=True, exist_ok=True)
            p = DATA_DIR / "initial_admin_password"
            fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w") as fh:
                fh.write(f"{username}\n{password}\n")
            note = str(p)
        s.add(User(username=username, password_hash=hash_password(password), role="admin"))
    if note:
        log.warning("created admin user %r; password stored in %s (change it after first login)", username, note)
    return username


def create_app(cfg: Optional[dict] = None, db_url: Optional[str] = None, secret: Optional[str] = None,
               robot: Optional[Robot] = None, start_robot: bool = True) -> FastAPI:
    _load_dotenv()
    cfg = cfg or load_config()
    db = Database(db_url)
    db.migrate()
    bootstrap_admin(db)
    secret = secret or os.environ.get("NEXUS_SECRET_KEY") or _secret_file("secret_key")
    hub = Hub()
    robot = robot or Robot(cfg)
    service = RobotService(robot, db, hub)
    srv = cfg.get("server", {})
    c = AppContainer(
        cfg=cfg, db=db, hub=hub, robot=robot, service=service,
        tokens=TokenService(secret, int(os.environ.get("NEXUS_SESSION_MINUTES", "480"))),
        network=NetworkPolicy(os.environ.get("NEXUS_ALLOWED_NETWORKS",
                                             "127.0.0.0/8,10.0.0.0/8,172.16.0.0/12,192.168.0.0/16,::1/128")),
        limiter=RateLimiter(srv.get("rate_limit_per_minute", 240), 60.0),
        motion_limiter=RateLimiter(srv.get("motion_rate_limit_per_second", 20), 1.0),
        login_limiter=RateLimiter(10, 60.0),
        config_path=os.environ.get("NEXUS_CONFIG"),
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        hub.bind_loop(asyncio.get_running_loop())
        if start_robot:
            service.start()
        try:
            yield
        finally:
            if start_robot:
                service.stop()

    app = FastAPI(title="PROJECT NEXUS API", version="1.0.0", lifespan=lifespan,
                  description="Control and telemetry API for the NEXUS autonomous mobile robot.")
    app.state.container = c

    app.add_middleware(CORSMiddleware, allow_origins=srv.get("cors_origins", []), allow_credentials=False,
                       allow_methods=["GET", "POST", "PUT", "PATCH", "DELETE"],
                       allow_headers=["Authorization", "Content-Type"])

    @app.middleware("http")
    async def guard(request: Request, call_next):
        host = request.client.host if request.client else None
        if not c.network.allowed(host):
            return JSONResponse({"detail": "client network not allowed"}, status_code=403)
        path = request.url.path
        # the stream and the e-stop are never rate limited
        if (path.startswith("/api/") or path == "/metrics") and not path.endswith("stream.mjpg") \
                and "emergency-stop" not in path:
            if not c.limiter.allow(f"api:{host}"):
                return JSONResponse({"detail": "rate limit exceeded"}, status_code=429)
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["Referrer-Policy"] = "no-referrer"
        return response

    for r in (auth_routes.router, auth_routes.users_router, robot_routes.system, robot_routes.robot_r,
              robot_routes.nav, robot_routes.mapping, robot_routes.sensors, robot_routes.logs,
              vision_routes.vision, vision_routes.sorting, autonomy_routes.localization,
              autonomy_routes.missions, autonomy_routes.objects, autonomy_routes.schedules,
              sim_routes.router, metrics.router,
              recording_routes.router):
        app.include_router(r)

    @app.get("/api/ping")
    def ping():
        return {"ok": True}

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        host = ws.client.host if ws.client else None
        if not c.network.allowed(host):
            await ws.close(code=1008)
            return
        user = resolve_token(c, ws.query_params.get("token"))
        if user is None:
            await ws.close(code=1008)
            return
        await ws.accept()
        client = Client(ws, user.username, user.role)
        await ws.send_json({"topic": "hello", "data": {"user": user.username, "role": user.role,
                                                        "state": robot.state()}})
        await hub.serve(client)

    dist = ROOT_DIR / "frontend" / "dist"
    if dist.exists():
        app.mount("/assets", StaticFiles(directory=dist / "assets"), name="assets")

        @app.get("/{path:path}", include_in_schema=False)
        def spa(path: str):
            f = (dist / path).resolve()
            if path and f.is_file() and dist.resolve() in f.parents:
                return FileResponse(f)
            return FileResponse(dist / "index.html")

    return app


def main() -> None:
    import uvicorn
    _load_dotenv()
    logging.basicConfig(level=os.environ.get("NEXUS_LOG_LEVEL", "INFO"),
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    uvicorn.run(create_app(), host=os.environ.get("NEXUS_HOST", "0.0.0.0"),
                port=int(os.environ.get("NEXUS_PORT", "8000")), log_level="info")


if __name__ == "__main__":
    main()
