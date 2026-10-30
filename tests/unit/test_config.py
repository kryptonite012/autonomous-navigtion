import pytest
import yaml

from robot.config import load_config, save_section


def test_load_with_overrides(monkeypatch):
    monkeypatch.delenv("NEXUS_MODE", raising=False)
    cfg = load_config(overrides={"chassis": {"max_linear_mps": 0.1}})
    assert cfg["chassis"]["max_linear_mps"] == 0.1
    assert cfg["chassis"]["wheel_base_m"] > 0          # untouched keys kept


def test_env_mode_and_validation(monkeypatch):
    monkeypatch.setenv("NEXUS_MODE", "hardware")
    assert load_config()["robot"]["mode"] == "hardware"
    monkeypatch.setenv("NEXUS_MODE", "turbo")
    with pytest.raises(ValueError):
        load_config()


def test_save_section_keeps_backup(tmp_path):
    p = tmp_path / "c.yaml"
    p.write_text("# comment\nrobot: {mode: simulation}\nsorting: {bins: {red: 1}}\n")
    save_section(p, "sorting", {"bins": {"red": 2}})
    assert yaml.safe_load(p.read_text())["sorting"]["bins"]["red"] == 2
    assert "# comment" in (tmp_path / "c.yaml.bak").read_text()
