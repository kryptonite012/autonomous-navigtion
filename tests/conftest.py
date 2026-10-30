import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robot.config import load_config  # noqa: E402


@pytest.fixture
def cfg(tmp_path):
    return load_config(overrides={
        "robot": {"mode": "simulation"},
        "mapping": {"maps_dir": str(tmp_path / "maps")},
        "simulation": {"pickup_failure_rate": 0.0},
    })


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: real-time simulation tests (seconds to minutes)")
    config.addinivalue_line("markers", "hardware: needs the physical robot (NEXUS_HIL=1)")


def pytest_collection_modifyitems(config, items):
    if os.environ.get("NEXUS_HIL") == "1":
        return
    skip = pytest.mark.skip(reason="hardware-in-the-loop test, set NEXUS_HIL=1 on the robot")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)
