from sqlalchemy import inspect, select

from backend.database.db import Database
from backend.database.models import SystemEvent


def test_migrations_create_all_tables(tmp_path):
    db = Database(f"sqlite:///{tmp_path / 'm.db'}")
    assert db.migrate() == 2
    assert db.migrate() == 0          # idempotent
    tables = set(inspect(db.engine).get_table_names())
    for t in ("users", "robot_sessions", "maps", "object_detections", "sorting_records",
              "telemetry", "system_events", "audit_log", "schema_version"):
        assert t in tables
    cols = {c["name"] for c in inspect(db.engine).get_columns("telemetry")}
    assert {"pose_x", "pose_y", "pose_theta"} <= cols
    assert db.version() == 2


def test_backup_and_restore(tmp_path):
    db = Database(f"sqlite:///{tmp_path / 'b.db'}")
    db.migrate()
    with db.session() as s:
        s.add(SystemEvent(event_type="t", severity="info", description="before backup"))
    backup = db.backup(tmp_path / "backups")
    assert backup.exists()
    with db.session() as s:
        s.add(SystemEvent(event_type="t", severity="info", description="after backup"))
    db.restore(backup)
    db2 = Database(f"sqlite:///{tmp_path / 'b.db'}")
    with db2.session() as s:
        rows = [e.description for e in s.execute(select(SystemEvent)).scalars()]
    assert rows == ["before backup"]


def test_map_store_rejects_bad_names(tmp_path):
    import pytest
    from robot.mapping.mapper import MapStore
    st = MapStore(str(tmp_path))
    for bad in ("../x", "", "a/b", "x" * 80, ".hidden"):
        with pytest.raises(ValueError):
            st.path(bad)
