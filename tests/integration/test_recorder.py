"""Flight recorder: record a simulated mapping run, then replay it offline."""

import subprocess
import sys
import time
from pathlib import Path

from fastapi.testclient import TestClient

from robot.config import load_config
from robot.recorder import read
from robot.runtime import Robot

ROOT = Path(__file__).resolve().parents[2]


def test_record_and_replay(tmp_path):
    cfg = load_config(overrides={
        "robot": {"mode": "simulation"}, "hardware": {"lidar": {"enabled": True}},
        "simulation": {"odom_noise": 0.05, "seed": 11},
        "mapping": {"maps_dir": str(tmp_path / "maps")}, "recording": {"dir": str(tmp_path / "rec")}})
    r = Robot(cfg)
    r.start()
    time.sleep(0.3)
    try:
        r.start_recording("run1")
        r.start_mapping(explore=False, reset=True)
        r.events.publish("test_marker", "info", "recorded with the run")
        for v, w, s in [(0.2, 0, 3), (0, -1.0, 1.6), (0.2, 0, 3)]:
            end = time.time() + s
            while time.time() < end:
                r.teleop(v, w)
                time.sleep(0.1)
        assert r.recorder.status()["records"] > 100
        assert r.recorder.stop() == "run1"
    finally:
        r.shutdown()
    path = tmp_path / "rec" / "run1.jsonl.gz"
    recs = list(read(path))
    assert recs[0]["type"] == "meta" and recs[0]["lidar"] is True
    assert recs[-1]["type"] == "end"
    steps = [x for x in recs if x["type"] == "step"]
    assert all("gt" in x and "ticks" in x for x in steps)
    assert sum(1 for x in steps if "scan" in x) > 50
    assert any(x["type"] == "event" and x["event"] == "test_marker" for x in recs)

    def replay(*extra):
        out = subprocess.run([sys.executable, str(ROOT / "scripts" / "replay.py"), str(path), *extra],
                             capture_output=True, text=True, timeout=120, cwd=ROOT)
        assert out.returncode == 0, out.stderr
        line = [l for l in out.stdout.splitlines() if "replayed estimate" in l][0]
        return float(line.split(":")[1].split("m")[0]), out.stdout

    rms_on, text = replay()
    rms_off, _ = replay("--no-scan-matching")
    assert "raw odometry" in text
    assert rms_on < rms_off
    assert rms_on < 0.05


def test_recording_api(cfg, tmp_path, monkeypatch):
    monkeypatch.setenv("NEXUS_ADMIN_PASSWORD", "Adm1n-Password!")
    from backend.main import create_app
    app = create_app(cfg, db_url=f"sqlite:///{tmp_path / 'r.db'}", secret="s" * 48)
    with TestClient(app) as c:
        tok = c.post("/api/auth/login", json={"username": "admin", "password": "Adm1n-Password!"}).json()["access_token"]
        c.headers["Authorization"] = f"Bearer {tok}"
        assert c.post("/api/recordings/start", json={"name": "../x"}).status_code == 422
        assert c.post("/api/recordings/start", json={"name": "t1"}).status_code == 200
        assert c.post("/api/recordings/start", json={"name": "t2"}).status_code == 409   # already recording
        time.sleep(0.5)
        assert c.get("/api/recordings/t1/download").status_code == 409                 # still open
        assert c.post("/api/recordings/stop").json()["name"] == "t1"
        listing = c.get("/api/recordings").json()
        assert [x["name"] for x in listing["recordings"]] == ["t1"] and not listing["status"]["active"]
        r = c.get("/api/recordings/t1/download")
        assert r.status_code == 200 and r.content[:2] == b"\x1f\x8b"
        assert c.delete("/api/recordings/t1").status_code == 200
        assert c.post("/api/recordings/stop").status_code == 409
