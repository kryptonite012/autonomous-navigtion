"""API tests for missions, localisation, object map, simulation control and metrics."""

import time

import pytest
from fastapi.testclient import TestClient

ADMIN_PW = "Adm1n-Password!"


@pytest.fixture
def client(cfg, tmp_path, monkeypatch):
    monkeypatch.setenv("NEXUS_ADMIN_PASSWORD", ADMIN_PW)
    from backend.main import create_app
    app = create_app(cfg, db_url=f"sqlite:///{tmp_path / 'v2.db'}", secret="s" * 48)
    with TestClient(app) as c:
        r = c.post("/api/auth/login", json={"username": "admin", "password": ADMIN_PW})
        c.headers["Authorization"] = "Bearer " + r.json()["access_token"]
        yield c


def test_mission_crud_and_validation(client):
    body = {"waypoints": [{"location": "home"}, {"x": 0.5, "y": -0.5, "dwell_s": 2}], "loop": True}
    r = client.put("/api/missions/patrol", json=body)
    assert r.status_code == 200, r.text
    assert [m["name"] for m in client.get("/api/missions").json()] == ["patrol"]
    bad = client.put("/api/missions/bad", json={"waypoints": [{"location": "atlantis"}]})
    assert bad.status_code == 422
    assert client.put("/api/missions/x", json={"waypoints": []}).status_code == 422
    assert client.put("/api/missions/x", json={"waypoints": [{"x": 1}]}).status_code == 422
    # no map loaded yet: starting is refused with a reason
    r = client.post("/api/missions/patrol/start")
    assert r.status_code == 409 and "map" in r.json()["detail"]
    assert client.delete("/api/missions/patrol").status_code == 200
    assert client.delete("/api/missions/patrol").status_code == 404


def test_missions_survive_restart(cfg, tmp_path, monkeypatch):
    monkeypatch.setenv("NEXUS_ADMIN_PASSWORD", ADMIN_PW)
    from backend.main import create_app
    url = f"sqlite:///{tmp_path / 'persist.db'}"
    for attempt in range(2):
        app = create_app(cfg, db_url=url, secret="s" * 48, start_robot=False)
        with TestClient(app) as c:
            tok = c.post("/api/auth/login", json={"username": "admin", "password": ADMIN_PW}).json()["access_token"]
            h = {"Authorization": f"Bearer {tok}"}
            if attempt == 0:
                c.put("/api/missions/tour", json={"waypoints": [{"location": "dock"}]}, headers=h)
            else:
                assert [m["name"] for m in c.get("/api/missions", headers=h).json()] == ["tour"]


def test_localization_endpoints(client):
    s = client.get("/api/localization/status").json()
    assert s["method"] == "odometry" and s["enabled"] is False      # no LiDAR in the default sim
    r = client.post("/api/localization/initial-pose", json={"x": 1.0, "y": 0.5, "theta": 0.0})
    assert r.status_code == 200
    assert r.json()["pose"]["x"] == pytest.approx(1.0, abs=0.02)
    assert client.post("/api/localization/global").status_code == 409


def test_simulation_controls(client):
    st = client.get("/api/simulation/state").json()
    assert "robot" in st and "pose_error_m" in st and st["sensors"]
    r = client.post("/api/simulation/faults", json={"disconnected_sensors": ["us_front"]})
    assert r.status_code == 200
    time.sleep(0.8)
    assert "us_front" in client.get("/api/robot/state").json()["safety"]["stale_sensors"]
    assert client.post("/api/simulation/faults", json={"disconnected_sensors": ["nope"]}).status_code == 422
    assert client.post("/api/simulation/obstacles", json={"x": 0.5, "y": 0.0}).status_code == 200
    assert len(client.get("/api/simulation/state").json()["dynamic"]) == 1
    assert client.delete("/api/simulation/obstacles").status_code == 200
    assert client.post("/api/simulation/objects/scatter", json={"count": 4}).status_code == 200
    assert len(client.get("/api/simulation/state").json()["objects"]) == 4
    assert client.post("/api/simulation/teleport", json={"x": -1.0, "y": 0.0, "theta": 1.0}).status_code == 200
    assert client.post("/api/simulation/teleport", json={"x": 1.0, "y": 0.7}).status_code == 422   # in the table
    assert client.post("/api/simulation/reference-map", json={"name": "groundtruth"}).status_code == 200
    assert "groundtruth" in [m["name"] for m in client.get("/api/mapping/maps").json()]
    assert client.post("/api/simulation/reset").status_code == 200
    st = client.get("/api/simulation/state").json()
    assert st["faults"]["disconnected_sensors"] == [] and st["robot"]["x"] == 0.0


def test_simulation_refused_in_hardware_mode(cfg, tmp_path, monkeypatch):
    monkeypatch.setenv("NEXUS_ADMIN_PASSWORD", ADMIN_PW)
    from backend.main import create_app
    from robot.runtime import Robot
    cfg["robot"]["mode"] = "hardware"
    app = create_app(cfg, db_url=f"sqlite:///{tmp_path / 'hw.db'}", secret="s" * 48, robot=Robot(cfg),
                     start_robot=False)
    with TestClient(app) as c:
        tok = c.post("/api/auth/login", json={"username": "admin", "password": ADMIN_PW}).json()["access_token"]
        h = {"Authorization": f"Bearer {tok}"}
        assert c.get("/api/simulation/state", headers=h).status_code == 409
        assert c.post("/api/simulation/faults", json={"motor_fault": True}, headers=h).status_code == 409


def test_object_map_endpoint(client):
    deadline = time.time() + 5
    objs = []
    while time.time() < deadline and not objs:
        time.sleep(0.3)
        objs = client.get("/api/vision/objects").json()["objects"]
    assert any(o["color"] == "red" for o in objs)
    assert client.delete("/api/vision/objects").status_code == 200


def test_map_snapshot_layers(client):
    snap = client.get("/api/mapping/current").json()
    assert {"particles", "objects", "ground_truth"} <= snap.keys()
    assert snap["ground_truth"] is not None        # simulation exposes ground truth


def test_metrics(client):
    text = client.get("/metrics").text
    assert 'nexus_up{mode="simulation"} 1.0' in text
    assert "nexus_control_loop_hz" in text and "nexus_estop_active 0.0" in text
    assert "# TYPE nexus_sort_operations_total counter" in text


def test_white_balance_endpoint(client):
    time.sleep(0.3)
    r = client.post("/api/vision/white-balance", json={"mode": "reference", "roi": [0, 0, 640, 100]})
    assert r.status_code == 200, r.text        # top of the simulated view is the light wall
    assert len(r.json()["gains"]) == 3
    assert client.post("/api/vision/white-balance", json={"mode": "reference"}).status_code == 422
    assert client.post("/api/vision/white-balance", json={"mode": "off"}).json()["mode"] == "off"
    assert client.get("/api/vision/status").json()["white_balance"]["mode"] == "off"


def test_coverage_mission_and_schedules(client):
    assert client.post("/api/missions/coverage", json={"name": "sweep"}).status_code == 409    # no map yet
    assert client.post("/api/simulation/reference-map", json={"name": "ref"}).status_code == 200
    assert client.post("/api/mapping/load", json={"name": "ref"}).status_code == 200
    r = client.post("/api/missions/coverage", json={"name": "sweep", "spacing": 0.6})
    assert r.status_code == 200, r.text
    assert len(r.json()["mission"]["waypoints"]) >= 6
    assert client.post("/api/missions/coverage", json={"name": "x", "spacing": 0.1}).status_code == 422
    r = client.put("/api/schedules/nightly", json={"mission": "sweep", "time": "21:00", "days": ["mon", "fri"]})
    assert r.status_code == 200, r.text
    assert client.put("/api/schedules/bad", json={"mission": "nope", "time": "21:00"}).status_code == 422
    assert client.put("/api/schedules/bad", json={"mission": "sweep", "time": "9pm"}).status_code == 422
    assert [s["name"] for s in client.get("/api/schedules").json()] == ["nightly"]
    assert client.delete("/api/schedules/nightly").status_code == 200
    assert client.delete("/api/schedules/nightly").status_code == 404
