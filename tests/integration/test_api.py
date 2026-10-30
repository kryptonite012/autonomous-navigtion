"""API tests: authentication, roles, validation, safety gating and WebSocket."""

import os

import pytest
from fastapi.testclient import TestClient

ADMIN_PW = "Adm1n-Password!"


@pytest.fixture
def client(cfg, tmp_path, monkeypatch):
    monkeypatch.setenv("NEXUS_ADMIN_USERNAME", "admin")
    monkeypatch.setenv("NEXUS_ADMIN_PASSWORD", ADMIN_PW)
    from backend.main import create_app
    app = create_app(cfg, db_url=f"sqlite:///{tmp_path / 'test.db'}", secret="s" * 48)
    with TestClient(app) as c:
        yield c


def login(client, user="admin", pw=ADMIN_PW):
    r = client.post("/api/auth/login", json={"username": user, "password": pw})
    assert r.status_code == 200, r.text
    return {"Authorization": "Bearer " + r.json()["access_token"]}


def make_user(client, h, name, role, pw="Op3rator-pass!"):
    r = client.post("/api/users", json={"username": name, "password": pw, "role": role}, headers=h)
    assert r.status_code == 201, r.text
    return login(client, name, pw)


def test_requires_authentication(client):
    for path in ("/api/system/status", "/api/robot/state", "/api/sensors/readings", "/api/vision/detections"):
        assert client.get(path).status_code == 401
    assert client.post("/api/robot/teleop", json={"linear": 0.1, "angular": 0}).status_code == 401


def test_login_failure_and_lockout(client):
    for _ in range(5):
        assert client.post("/api/auth/login", json={"username": "admin", "password": "nope"}).status_code == 401
    r = client.post("/api/auth/login", json={"username": "admin", "password": ADMIN_PW})
    assert r.status_code == 423


def test_logout_revokes_token(client):
    h = login(client)
    assert client.get("/api/auth/me", headers=h).status_code == 200
    client.post("/api/auth/logout", headers=h)
    assert client.get("/api/auth/me", headers=h).status_code == 401


def test_roles(client):
    admin = login(client)
    viewer = make_user(client, admin, "watcher", "viewer")
    operator = make_user(client, admin, "driver", "operator")
    assert client.get("/api/robot/state", headers=viewer).status_code == 200
    assert client.post("/api/robot/teleop", json={"linear": 0.1, "angular": 0}, headers=viewer).status_code == 403
    assert client.post("/api/robot/teleop", json={"linear": 0.1, "angular": 0}, headers=operator).status_code == 200
    assert client.get("/api/users", headers=operator).status_code == 403
    assert client.post("/api/system/shutdown", headers=operator).status_code == 403
    # anybody logged in can hit the emergency stop
    assert client.post("/api/robot/emergency-stop", headers=viewer).status_code == 200


def test_password_policy(client):
    h = login(client)
    r = client.post("/api/users", json={"username": "weak", "password": "aaaaaaaaaa", "role": "operator"}, headers=h)
    assert r.status_code == 422


def test_input_validation(client):
    h = login(client)
    assert client.post("/api/robot/teleop", json={"linear": 9, "angular": 0}, headers=h).status_code == 422
    assert client.post("/api/robot/teleop", json={"linear": "fast"}, headers=h).status_code == 422
    assert client.post("/api/mapping/save", json={"name": "../etc/passwd"}, headers=h).status_code == 422
    assert client.post("/api/vision/calibrate", json={"color": "red", "ranges": [
        {"lower": [10, 0, 0], "upper": [5, 255, 255]}]}, headers=h).status_code == 422
    assert client.put("/api/sorting/config", json={"bins": {"red": 42}, "unclassified_bin": 5},
                      headers=h).status_code == 422


def test_emergency_stop_blocks_motion_until_reset(client):
    h = login(client)
    client.post("/api/robot/emergency-stop", headers=h)
    r = client.post("/api/robot/teleop", json={"linear": 0.1, "angular": 0}, headers=h)
    assert r.status_code == 409 and "emergency" in r.json()["detail"]
    assert client.post("/api/sorting/start", headers=h).status_code == 409
    assert client.post("/api/robot/emergency-stop/reset", headers=h).status_code == 200
    assert client.post("/api/robot/teleop", json={"linear": 0.1, "angular": 0}, headers=h).status_code == 200


def test_navigation_rejected_without_map(client):
    h = login(client)
    r = client.post("/api/navigation/goal", json={"x": 1.0, "y": 0.0}, headers=h)
    assert r.status_code == 409
    r = client.post("/api/navigation/goal", json={"location": "nowhere"}, headers=h)
    assert r.status_code in (409, 422)


def test_endpoints_return_data(client):
    h = login(client)
    st = client.get("/api/system/status", headers=h).json()
    assert st["hardware_mode"] == "simulation"
    health = client.get("/api/system/health", headers=h).json()
    assert all(d["simulated"] for d in health["devices"] if d["state"] == "ok")
    readings = client.get("/api/sensors/readings", headers=h).json()
    assert readings["simulated"] is True and len(readings["ranges"]) == 4
    assert client.get("/api/vision/status", headers=h).json()["available"] is True
    assert client.get("/api/navigation/status", headers=h).json()["state"] == "idle"
    assert client.get("/api/sorting/statistics", headers=h).status_code == 200
    assert client.get("/api/sorting/history", headers=h).json() == []
    assert client.get("/api/logs/events", headers=h).status_code == 200


def test_map_lifecycle(client):
    h = login(client)
    assert client.post("/api/mapping/start", json={"explore": False, "reset": True}, headers=h).status_code == 200
    import time
    time.sleep(0.5)
    assert client.post("/api/mapping/save", json={"name": "lab"}, headers=h).status_code == 200
    client.post("/api/mapping/stop", headers=h)
    names = [m["name"] for m in client.get("/api/mapping/maps", headers=h).json()]
    assert "lab" in names
    assert client.post("/api/mapping/rename", json={"old_name": "lab", "new_name": "lab2"},
                       headers=h).status_code == 200
    assert client.post("/api/mapping/load", json={"name": "lab2"}, headers=h).status_code == 200
    assert client.get("/api/mapping/maps/lab2/download", headers=h).status_code == 200
    assert client.delete("/api/mapping/maps/lab2", headers=h).status_code == 200
    assert client.post("/api/mapping/load", json={"name": "lab2"}, headers=h).status_code == 404


def test_websocket_requires_token_and_streams(client):
    h = login(client)
    token = h["Authorization"][7:]
    with pytest.raises(Exception):
        with client.websocket_connect("/ws") as ws:
            ws.receive_json()
    with client.websocket_connect(f"/ws?token={token}") as ws:
        assert ws.receive_json()["topic"] == "hello"
        topics = {ws.receive_json()["topic"] for _ in range(8)}
        assert "telemetry" in topics


def test_snapshot_jpeg(client):
    import time
    h = login(client)
    time.sleep(0.4)
    r = client.get("/api/vision/snapshot.jpg", headers=h)
    assert r.status_code == 200 and r.content[:2] == b"\xff\xd8"


def test_audit_log_records_actions(client):
    h = login(client)
    client.post("/api/robot/emergency-stop", headers=h)
    log = client.get("/api/users/audit", headers=h).json()
    actions = [e["action"] for e in log]
    assert "emergency_stop" in actions and "login" in actions
