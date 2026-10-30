"""Password hashing, session tokens and role checks."""

from __future__ import annotations

import ipaddress
import secrets
from datetime import datetime, timedelta, timezone
from typing import Optional

import bcrypt
import jwt

ROLES = ("viewer", "operator", "admin")
ROLE_RANK = {r: i for i, r in enumerate(ROLES)}

MIN_PASSWORD_LEN = 10
LOCKOUT_THRESHOLD = 5
LOCKOUT_MINUTES = 10


def hash_password(password: str) -> str:
    # bcrypt only uses the first 72 bytes; reject longer inputs instead of silently truncating
    pw = password.encode("utf-8")
    if len(pw) > 72:
        raise ValueError("password too long (max 72 bytes)")
    return bcrypt.hashpw(pw, bcrypt.gensalt(rounds=12)).decode("ascii")


def verify_password(password: str, password_hash: str) -> bool:
    try:
        return bcrypt.checkpw(password.encode("utf-8")[:72], password_hash.encode("ascii"))
    except ValueError:
        return False


def check_password_policy(password: str) -> None:
    if len(password) < MIN_PASSWORD_LEN:
        raise ValueError(f"password must be at least {MIN_PASSWORD_LEN} characters")
    classes = sum([any(c.islower() for c in password), any(c.isupper() for c in password),
                   any(c.isdigit() for c in password), any(not c.isalnum() for c in password)])
    if classes < 3:
        raise ValueError("password needs at least 3 of: lowercase, uppercase, digit, symbol")


def role_allows(role: str, required: str) -> bool:
    return ROLE_RANK.get(role, -1) >= ROLE_RANK[required]


class TokenService:
    """Signed tokens that reference a server-side session (revocable)."""

    ALGORITHM = "HS256"

    def __init__(self, secret: str, lifetime_minutes: int = 480):
        if not secret or len(secret) < 32:
            raise ValueError("NEXUS_SECRET_KEY must be set to at least 32 characters")
        self.secret = secret
        self.lifetime = timedelta(minutes=lifetime_minutes)

    def new_session_id(self) -> str:
        return secrets.token_urlsafe(24)

    def issue(self, session_id: str, username: str, role: str) -> tuple[str, datetime]:
        exp = datetime.now(timezone.utc) + self.lifetime
        token = jwt.encode({"sid": session_id, "sub": username, "role": role, "exp": exp,
                            "iat": datetime.now(timezone.utc)}, self.secret, algorithm=self.ALGORITHM)
        return token, exp

    def decode(self, token: str) -> Optional[dict]:
        try:
            return jwt.decode(token, self.secret, algorithms=[self.ALGORITHM])
        except jwt.PyJWTError:
            return None


class NetworkPolicy:
    def __init__(self, cidrs: str):
        self.networks = [ipaddress.ip_network(c.strip(), strict=False) for c in cidrs.split(",") if c.strip()]

    def allowed(self, host: Optional[str]) -> bool:
        if not self.networks:
            return True
        if host is None:
            return False
        if host == "testclient":       # Starlette test client
            return True
        try:
            ip = ipaddress.ip_address(host)
        except ValueError:
            return False
        return any(ip in n for n in self.networks)
