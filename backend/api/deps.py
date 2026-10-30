"""Application container and FastAPI dependencies (auth, roles, audit)."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

from fastapi import Depends, HTTPException, Request, status
from sqlalchemy import select

from backend.authentication.ratelimit import RateLimiter
from backend.authentication.security import NetworkPolicy, TokenService, role_allows
from backend.database.db import Database
from backend.database.models import AuditLog, AuthSession, User
from backend.services.robot_service import RobotService
from backend.websocket.hub import Hub
from robot.runtime import Robot

log = logging.getLogger(__name__)


@dataclass
class AppContainer:
    cfg: dict
    db: Database
    hub: Hub
    robot: Robot
    service: RobotService
    tokens: TokenService
    network: NetworkPolicy
    limiter: RateLimiter
    motion_limiter: RateLimiter
    login_limiter: RateLimiter
    config_path: Optional[str] = None


@dataclass
class CurrentUser:
    id: int
    username: str
    role: str
    session_id: str


def container(request: Request) -> AppContainer:
    return request.app.state.container


def client_ip(request: Request) -> str:
    return request.client.host if request.client else "unknown"


def _extract_token(request: Request) -> Optional[str]:
    auth = request.headers.get("authorization", "")
    if auth.lower().startswith("bearer "):
        return auth[7:].strip()
    # <img src> for the MJPEG stream cannot send headers
    return request.query_params.get("token")


def resolve_token(c: AppContainer, token: Optional[str]) -> Optional[CurrentUser]:
    if not token:
        return None
    claims = c.tokens.decode(token)
    if not claims:
        return None
    with c.db.session() as s:
        sess = s.get(AuthSession, claims.get("sid"))
        if sess is None or sess.revoked:
            return None
        exp = sess.expires_at if sess.expires_at.tzinfo else sess.expires_at.replace(tzinfo=timezone.utc)
        if exp < datetime.now(timezone.utc):
            return None
        user = s.get(User, sess.user_id)
        if user is None or not user.is_active:
            return None
        return CurrentUser(user.id, user.username, user.role, sess.id)


def current_user(request: Request, c: AppContainer = Depends(container)) -> CurrentUser:
    user = resolve_token(c, _extract_token(request))
    if user is None:
        raise HTTPException(status.HTTP_401_UNAUTHORIZED, "not authenticated",
                            headers={"WWW-Authenticate": "Bearer"})
    request.state.user = user
    return user


def require_role(role: str):
    def checker(user: CurrentUser = Depends(current_user)) -> CurrentUser:
        if not role_allows(user.role, role):
            raise HTTPException(status.HTTP_403_FORBIDDEN, f"requires {role} role")
        return user
    return checker


viewer = require_role("viewer")
operator = require_role("operator")
admin = require_role("admin")


def motion_rate_limit(request: Request, c: AppContainer = Depends(container),
                      user: CurrentUser = Depends(operator)) -> CurrentUser:
    if not c.motion_limiter.allow(f"motion:{user.username}"):
        raise HTTPException(status.HTTP_429_TOO_MANY_REQUESTS, "motion command rate exceeded")
    return user


def audit(c: AppContainer, request: Request, action: str, detail: str = "", success: bool = True,
          username: Optional[str] = None) -> None:
    user = getattr(request.state, "user", None)
    try:
        with c.db.session() as s:
            s.add(AuditLog(username=username or (user.username if user else None), action=action,
                           detail=detail[:2000], client_ip=client_ip(request), success=success))
    except Exception:
        log.exception("audit write failed")


def find_user(c: AppContainer, username: str) -> Optional[User]:
    with c.db.session() as s:
        return s.execute(select(User).where(User.username == username)).scalar_one_or_none()
