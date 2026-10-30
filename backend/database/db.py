"""Engine/session setup, versioned migrations and backups."""

from __future__ import annotations

import logging
import os
import sqlite3
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterator

from sqlalchemy import create_engine, event, inspect, select, text
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session, sessionmaker

from backend.database.models import Base, SchemaVersion
from robot.config import ROOT_DIR

log = logging.getLogger(__name__)


def default_url() -> str:
    return os.environ.get("NEXUS_DATABASE_URL", f"sqlite:///{ROOT_DIR / 'data' / 'nexus.db'}")


def _sqlite_path(url: str) -> Path | None:
    if not url.startswith("sqlite:///"):
        return None
    p = url[len("sqlite:///"):]
    return None if p in ("", ":memory:") else Path(p)


# --------------------------------------------------------------------- migrations
# Each migration is (version, description, fn(engine)). Version 1 creates the
# initial schema from the models; later versions alter it in place. Never edit
# a released migration, add a new one.

def _m1_initial(engine: Engine) -> None:
    Base.metadata.create_all(engine)


def _m2_telemetry_pose(engine: Engine) -> None:
    cols = {c["name"] for c in inspect(engine).get_columns("telemetry")}
    with engine.begin() as conn:
        for name in ("pose_x", "pose_y", "pose_theta"):
            if name not in cols:
                conn.execute(text(f"ALTER TABLE telemetry ADD COLUMN {name} FLOAT"))


def _m3_missions(engine: Engine) -> None:
    from backend.database.models import MissionRecord
    MissionRecord.__table__.create(engine, checkfirst=True)


def _m4_schedules(engine: Engine) -> None:
    from backend.database.models import ScheduleRecord
    ScheduleRecord.__table__.create(engine, checkfirst=True)


MIGRATIONS: list[tuple[int, str, Callable[[Engine], None]]] = [
    (1, "initial schema", _m1_initial),
    (2, "pose columns on telemetry", _m2_telemetry_pose),
    (3, "missions", _m3_missions),
    (4, "mission schedules", _m4_schedules),
]


class Database:
    def __init__(self, url: str | None = None):
        self.url = url or default_url()
        path = _sqlite_path(self.url)
        if path:
            path.parent.mkdir(parents=True, exist_ok=True)
        kwargs = {"connect_args": {"check_same_thread": False}} if self.url.startswith("sqlite") else {"pool_pre_ping": True}
        self.engine = create_engine(self.url, future=True, **kwargs)
        if self.url.startswith("sqlite"):
            @event.listens_for(self.engine, "connect")
            def _pragmas(dbapi_conn, _):
                cur = dbapi_conn.cursor()
                cur.execute("PRAGMA journal_mode=WAL")
                cur.execute("PRAGMA foreign_keys=ON")
                cur.close()
        self.SessionLocal = sessionmaker(bind=self.engine, expire_on_commit=False, future=True)

    def migrate(self) -> int:
        SchemaVersion.__table__.create(self.engine, checkfirst=True)
        with self.session() as s:
            done = {v for (v,) in s.execute(select(SchemaVersion.version))}
        applied = 0
        for version, desc, fn in MIGRATIONS:
            if version in done:
                continue
            log.info("applying migration %d: %s", version, desc)
            fn(self.engine)
            with self.session() as s:
                s.add(SchemaVersion(version=version, description=desc))
            applied += 1
        return applied

    def version(self) -> int:
        with self.session() as s:
            v = s.execute(select(SchemaVersion.version).order_by(SchemaVersion.version.desc())).first()
            return v[0] if v else 0

    @contextmanager
    def session(self) -> Iterator[Session]:
        s = self.SessionLocal()
        try:
            yield s
            s.commit()
        except Exception:
            s.rollback()
            raise
        finally:
            s.close()

    def backup(self, dest_dir: str | Path | None = None) -> Path:
        """Online backup of the SQLite database (safe while the app runs).

        For PostgreSQL use pg_dump (see scripts/backup_db.sh).
        """
        src = _sqlite_path(self.url)
        if src is None:
            raise RuntimeError("built-in backup supports SQLite only; use pg_dump for PostgreSQL")
        dest_dir = Path(dest_dir or ROOT_DIR / "backups")
        dest_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        dest = dest_dir / f"nexus-{stamp}.db"
        with sqlite3.connect(src) as source, sqlite3.connect(dest) as target:
            source.backup(target)
        return dest

    def restore(self, backup_file: str | Path) -> None:
        dst = _sqlite_path(self.url)
        if dst is None:
            raise RuntimeError("restore supports SQLite only")
        self.engine.dispose()
        # copy page by page through SQLite itself so the WAL of the live
        # database is replaced too (a plain file copy would be overlaid by it)
        with sqlite3.connect(backup_file) as source, sqlite3.connect(dst) as target:
            source.backup(target)
