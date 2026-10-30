"""Authentication and user management endpoints."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone

from fastapi import APIRouter, Depends, HTTPException, Request, status
from sqlalchemy import select

from backend.api.deps import (AppContainer, CurrentUser, admin, audit, client_ip, container,
                              current_user)
from backend.api.schemas import LoginRequest, PasswordChange, TokenResponse, UserCreate, UserUpdate
from backend.authentication.security import (LOCKOUT_MINUTES, LOCKOUT_THRESHOLD, check_password_policy,
                                             hash_password, verify_password)
from backend.database.models import AuditLog, AuthSession, User

router = APIRouter(prefix="/api/auth", tags=["auth"])
users_router = APIRouter(prefix="/api/users", tags=["users"])

# Used to make login timing similar whether or not the user exists
_DUMMY_HASH = hash_password("timing-equaliser-Aa1!")


def _aware(dt):
    return dt if dt is None or dt.tzinfo else dt.replace(tzinfo=timezone.utc)


@router.post("/login", response_model=TokenResponse)
def login(body: LoginRequest, request: Request, c: AppContainer = Depends(container)):
    ip = client_ip(request)
    if not c.login_limiter.allow(f"login:{ip}"):
        raise HTTPException(status.HTTP_429_TOO_MANY_REQUESTS, "too many login attempts, wait a minute")
    now = datetime.now(timezone.utc)
    # decide inside the transaction, raise after it commits so the failed
    # login counter is not rolled back by the exception
    error = None
    with c.db.session() as s:
        user = s.execute(select(User).where(User.username == body.username)).scalar_one_or_none()
        if user is None:
            verify_password(body.password, _DUMMY_HASH)
            error = (status.HTTP_401_UNAUTHORIZED, "invalid username or password", "unknown user")
        elif _aware(user.locked_until) and _aware(user.locked_until) > now:
            error = (status.HTTP_423_LOCKED, "account temporarily locked", "account locked")
        elif not user.is_active or not verify_password(body.password, user.password_hash):
            user.failed_logins += 1
            if user.failed_logins >= LOCKOUT_THRESHOLD:
                user.locked_until = now + timedelta(minutes=LOCKOUT_MINUTES)
                user.failed_logins = 0
            error = (status.HTTP_401_UNAUTHORIZED, "invalid username or password", "bad password or inactive")
        else:
            user.failed_logins = 0
            user.locked_until = None
            sid = c.tokens.new_session_id()
            token, exp = c.tokens.issue(sid, user.username, user.role)
            s.add(AuthSession(id=sid, user_id=user.id, expires_at=exp, client_ip=ip))
            username, role = user.username, user.role
    if error:
        audit(c, request, "login", error[2], False, username=body.username)
        raise HTTPException(error[0], error[1])
    audit(c, request, "login", "ok", True, username=username)
    return TokenResponse(access_token=token, expires_at=exp.isoformat(), username=username, role=role)


@router.post("/logout")
def logout(request: Request, user: CurrentUser = Depends(current_user), c: AppContainer = Depends(container)):
    with c.db.session() as s:
        sess = s.get(AuthSession, user.session_id)
        if sess:
            sess.revoked = True
    audit(c, request, "logout")
    return {"ok": True}


@router.get("/me")
def me(user: CurrentUser = Depends(current_user)):
    return {"id": user.id, "username": user.username, "role": user.role}


@router.post("/password")
def change_password(body: PasswordChange, request: Request, user: CurrentUser = Depends(current_user),
                    c: AppContainer = Depends(container)):
    try:
        check_password_policy(body.new_password)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    with c.db.session() as s:
        u = s.get(User, user.id)
        if not verify_password(body.current_password, u.password_hash):
            wrong = True
        else:
            wrong = False
            u.password_hash = hash_password(body.new_password)
            # revoke every other session of this user
            for sess in s.execute(select(AuthSession).where(AuthSession.user_id == u.id)).scalars():
                if sess.id != user.session_id:
                    sess.revoked = True
    if wrong:
        audit(c, request, "password_change", "wrong current password", False)
        raise HTTPException(status.HTTP_403_FORBIDDEN, "current password is incorrect")
    audit(c, request, "password_change")
    return {"ok": True}


# ------------------------------------------------------------------ users (admin)
@users_router.get("")
def list_users(_: CurrentUser = Depends(admin), c: AppContainer = Depends(container)):
    with c.db.session() as s:
        return [{"id": u.id, "username": u.username, "role": u.role, "is_active": u.is_active,
                 "created_at": u.created_at.isoformat()} for u in s.execute(select(User)).scalars()]


@users_router.post("", status_code=201)
def create_user(body: UserCreate, request: Request, _: CurrentUser = Depends(admin),
                c: AppContainer = Depends(container)):
    try:
        check_password_policy(body.password)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    with c.db.session() as s:
        if s.execute(select(User).where(User.username == body.username)).scalar_one_or_none():
            raise HTTPException(409, "username already exists")
        u = User(username=body.username, password_hash=hash_password(body.password), role=body.role)
        s.add(u)
        s.flush()
        uid = u.id
    audit(c, request, "user_create", f"{body.username} ({body.role})")
    return {"id": uid, "username": body.username, "role": body.role}


@users_router.patch("/{user_id}")
def update_user(user_id: int, body: UserUpdate, request: Request, me_: CurrentUser = Depends(admin),
                c: AppContainer = Depends(container)):
    with c.db.session() as s:
        u = s.get(User, user_id)
        if u is None:
            raise HTTPException(404, "user not found")
        if u.id == me_.id and (body.role not in (None, "admin") or body.is_active is False):
            raise HTTPException(400, "you cannot demote or deactivate your own account")
        if body.role:
            u.role = body.role
        if body.is_active is not None:
            u.is_active = body.is_active
        if body.password:
            try:
                check_password_policy(body.password)
            except ValueError as exc:
                raise HTTPException(422, str(exc))
            u.password_hash = hash_password(body.password)
        if body.is_active is False or body.password or body.role:
            for sess in s.execute(select(AuthSession).where(AuthSession.user_id == u.id)).scalars():
                sess.revoked = True
        name = u.username
    audit(c, request, "user_update", f"{name}: {body.model_dump(exclude={'password'}, exclude_none=True)}")
    return {"ok": True}


@users_router.delete("/{user_id}")
def delete_user(user_id: int, request: Request, me_: CurrentUser = Depends(admin),
                c: AppContainer = Depends(container)):
    if user_id == me_.id:
        raise HTTPException(400, "you cannot delete your own account")
    with c.db.session() as s:
        u = s.get(User, user_id)
        if u is None:
            raise HTTPException(404, "user not found")
        name = u.username
        s.delete(u)
    audit(c, request, "user_delete", name)
    return {"ok": True}


@users_router.get("/audit")
def audit_log(limit: int = 200, _: CurrentUser = Depends(admin), c: AppContainer = Depends(container)):
    limit = max(1, min(limit, 1000))
    with c.db.session() as s:
        rows = s.execute(select(AuditLog).order_by(AuditLog.id.desc()).limit(limit)).scalars()
        return [{"id": r.id, "username": r.username, "action": r.action, "detail": r.detail,
                 "client_ip": r.client_ip, "success": r.success, "timestamp": r.timestamp.isoformat()}
                for r in rows]
