import hashlib
import json
import math
import xml.etree.ElementTree as ET

import pytest

mujoco = pytest.importorskip("mujoco")
import numpy as np

from tools.vendor_piper import canonical_bytes

from robot_pick_place_agent.adapters.simulation.piper import PiperMujocoRobot
from robot_pick_place_agent.adapters.simulation.piper_m1 import run_m1
from robot_pick_place_agent.adapters.simulation.piper_model import ASSETS, HOME, TCP_OFFSET
from robot_pick_place_agent.core.models import Pose


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    x, y, z = axis / np.linalg.norm(axis)
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross


def urdf_fk(q):
    """Independent URDF transform chain, including the fixed gripper mount."""
    root = ET.parse(ASSETS / "upstream/piper_description.urdf").getroot()
    transforms = {"base_link": np.eye(4)}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        r, p, y = map(float, origin.get("rpy").split())
        local = np.eye(4)
        local[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
        local[:3, 3] = np.fromstring(origin.get("xyz"), sep=" ")
        motion = np.eye(4)
        if joint.get("type") != "fixed":
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            value = q[int(joint.get("name")[5:]) - 1]
            if joint.get("type") == "revolute":
                motion[:3, :3] = rotation(axis, value)
            else:
                motion[:3, 3] = axis * value
        transforms[joint.find("child").get("link")] = transforms[joint.find("parent").get("link")] @ local @ motion
    return transforms


def test_pinned_assets_are_unchanged():
    manifest = json.loads((ASSETS / "source.json").read_text())
    for entry in manifest["files"]:
        data = (ASSETS / entry["local"]).read_bytes()
        assert hashlib.sha256(canonical_bytes(data, entry["local"])).hexdigest() == entry["sha256"]


@pytest.mark.parametrize("q", [np.zeros(8), np.array([*HOME, .035, -.035]), np.array([-.4, 1.2, -1.4, .6, -.5, .8, .02, -.02])])
def test_mujoco_transforms_match_current_official_urdf(q):
    robot = PiperMujocoRobot()
    # This fixture assignment is solely a static FK audit, never execution.
    robot.data.qpos[:] = q
    mujoco.mj_forward(robot.model, robot.data)
    transforms = urdf_fk(q)
    for name, reference in transforms.items():
        np.testing.assert_allclose(robot.data.body(name).xpos, reference[:3, 3], atol=1e-10)
        np.testing.assert_allclose(robot.data.body(name).xmat.reshape(3, 3), reference[:3, :3], atol=1e-10)
    tcp = transforms["link6"] @ [*TCP_OFFSET, 1]
    np.testing.assert_allclose(robot.data.site("tcp").xpos, tcp[:3], atol=1e-10)


def test_structure_limits_and_inertia_match_urdf():
    robot = PiperMujocoRobot()
    model = robot.model
    assert model.nmocap == model.neq == 0
    assert model.nq == model.nv == model.nu == 8
    assert model.body("base_link").jntnum[0] == 0
    assert model.body("link7").parentid[0] == model.body("gripper_base").id
    assert model.body("gripper_base").parentid[0] == model.body("link6").id
    root = ET.parse(ASSETS / "upstream/piper_description.urdf").getroot()
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        actual = model.joint(joint.get("name"))
        limit = joint.find("limit")
        np.testing.assert_allclose(actual.range, [float(limit.get("lower")), float(limit.get("upper"))])
        np.testing.assert_allclose(actual.axis, np.fromstring(joint.find("axis").get("xyz"), sep=" "))
    for link in root.findall("link"):
        body = model.body(link.get("name"))
        inertial = link.find("inertial")
        assert body.mass[0] == pytest.approx(float(inertial.find("mass").get("value")))
        original = inertial.find("inertia")
        tensor = np.array([[float(original.get("ixx")), float(original.get("ixy")), float(original.get("ixz"))],
                           [float(original.get("ixy")), float(original.get("iyy")), float(original.get("iyz"))],
                           [float(original.get("ixz")), float(original.get("iyz")), float(original.get("izz"))]])
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, body.iquat)
        rot = rot.reshape(3, 3)
        # MuJoCo's principal-axis decomposition has finite eigensolver tolerance.
        assert np.linalg.norm(rot @ np.diag(body.inertia) @ rot.T - tensor) < 1e-6 * np.linalg.norm(tensor) + 1e-10
        assert np.linalg.eigvalsh(tensor).min() > 0


def test_m1_full_dynamic_sequence_and_rejections(tmp_path):
    result = run_m1(tmp_path, video=False)
    assert result["passed"], result
    assert len(result["motions"]) == 18
    assert not result["physical_pick_place_verified"]
    assert not result["cap_api_used"]


def test_invalid_cartesian_frame_and_cancel_do_not_teleport():
    robot = PiperMujocoRobot()
    before = robot.data.qpos.copy()
    assert not robot.move_to(Pose("base", .3, .1, .2))
    robot.cancel()
    np.testing.assert_array_equal(before, robot.data.qpos)
    assert robot.data.time == 0
    assert robot.get_state()["capabilities"]["cartesian_motion"]
