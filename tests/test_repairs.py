import importlib.util
from pathlib import Path

import pytest

from robot_pick_place_agent.core.models import ActionStatus, Pose
from robot_pick_place_agent.perception.mock_scene import MockSceneProvider
from robot_pick_place_agent.skills.pick_place import pick_and_place
from robot_pick_place_agent.core.models import TaskIntent
from robot_pick_place_agent.adapters.simulation.piper import PiperMujocoRobot


class FeedbackRobot:
    def __init__(self, feedback, limits=(0.0, 0.07)):
        self.feedback = dict(feedback)
        self.gripper_opening_limits_m = limits
        self.calls = []

    def move_to(self, pose):
        self.calls.append(("move_to", pose))
        return True

    def set_gripper(self, opening_m):
        self.calls.append(("gripper", opening_m))
        return True

    def get_state(self):
        return {**self.feedback, "gripper_opening_limits_m": list(self.gripper_opening_limits_m)}

    def cancel(self):
        self.calls.append(("cancel", None))


def intent_and_scene():
    scene = MockSceneProvider().observe()
    return scene, TaskIntent("把红色方块放进蓝色盒子", "red-block", "blue-box", scene.scene_id)


@pytest.mark.parametrize("feedback", [
    {"placement_verified": False, "placed_object_id": "red-block", "placement_target_id": "blue-box", "placement_error_m": 0.0},
    {"placement_verified": True, "placed_object_id": "red-block", "placement_target_id": "blue-box", "placement_error_m": 0.0, "held_object": "red-block"},
    {"held_object": None},
])
def test_placement_without_verified_target_evidence_is_not_success(feedback):
    scene, intent = intent_and_scene()
    result = pick_and_place(FeedbackRobot(feedback), scene, intent)
    assert result.status is ActionStatus.UNCERTAIN
    assert result.stage == "verify"


def test_placement_requires_identity_and_target_position_and_uses_device_opening():
    scene, intent = intent_and_scene()
    robot = FeedbackRobot({"placement_verified": True, "placed_object_id": "red-block",
                           "placement_target_id": "blue-box", "placement_error_m": 0.001})
    result = pick_and_place(robot, scene, intent)
    assert result.status is ActionStatus.SUCCEEDED
    assert result.evidence["placement_verification"]["verified"] is True
    assert result.evidence["release_opening_m"] == pytest.approx(0.07)
    assert result.evidence["release_opening_source"] == "device_configuration"
    assert robot.calls[-1] == ("gripper", pytest.approx(0.07))


def test_windows_text_newlines_are_canonical_but_binary_is_exact():
    spec = importlib.util.spec_from_file_location("vendor_piper", Path(__file__).parents[1] / "tools/vendor_piper.py")
    vendor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vendor)
    assert vendor.canonical_bytes(b"a\r\nb\rc\n", "upstream/model.xml") == b"a\nb\nc\n"
    assert vendor.canonical_bytes(b"a\r\nb", "meshes/model.STL") == b"a\r\nb"
    assert vendor.verify_assets()


def test_cancel_interrupts_active_joint_trajectory_and_retains_cancelled_state():
    robot = PiperMujocoRobot()
    cancelled = False

    def cancel_once(active_robot):
        nonlocal cancelled
        if not cancelled and active_robot.data.time >= 0.01:
            cancelled = True
            active_robot.cancel()

    robot.on_step = cancel_once
    target = [0.8, 1.2, -1.2, 0.5, 0.7, -0.6]
    assert robot.move_joints(target) is False
    assert cancelled
    assert robot.last_execution["status"] == "cancelled"
    assert robot.data.time < 3.0
    assert max(abs(robot.data.qpos[robot._qadr[:6]] - target)) > 0.1

    # A new command starts a new generation and is allowed to run normally.
    robot.on_step = None
    assert robot.move_joints([0.0, 0.8, -0.7, 0.0, 0.3, 0.0]) is True
