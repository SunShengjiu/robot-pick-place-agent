"""Physical acceptance and a no-friction counterexample for the isolated rig."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

pytest.importorskip("mujoco")
spec = importlib.util.spec_from_file_location(
    "minimal_grasp", Path(__file__).parents[1] / "examples/minimal_grasp_physics.py"
)
experiment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(experiment)


def test_dynamic_grasp_and_release(tmp_path):
    result = experiment.run(tmp_path, video=False)
    assert result["passed"], result


def test_zero_friction_cannot_transport_cube(tmp_path):
    root = ET.fromstring(experiment.XML)
    for geom in root.iter("geom"):
        geom.set("friction", "0 0 0")
        geom.set("condim", "1")
    result = experiment.run(tmp_path, video=False, xml=ET.tostring(root, encoding="unicode"))
    assert not result["passed"]
    assert not result["checks"]["sustained_lift_at_least_5cm"]
    assert not result["checks"]["horizontal_transport_at_least_10cm"]
    assert result["minimum_sustained_lift_m"] < .001
    assert abs(result["horizontal_transport_m"]) < .001
    assert result["checks"]["initially_stationary"]
    assert result["checks"]["no_closure_launch"]
    assert result["checks"]["stable_after_fall"]
