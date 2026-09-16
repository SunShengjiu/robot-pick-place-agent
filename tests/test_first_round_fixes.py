from dataclasses import replace

import pytest

from robot_pick_place_agent.agent.planner import CodeAsPoliciesPlanner
from robot_pick_place_agent.core.models import Pose, SceneObject, SceneSnapshot, ActionStatus, TaskIntent
from robot_pick_place_agent.perception.mock_scene import MockSceneProvider
from robot_pick_place_agent.skills.pick_place import pick_and_place


def scene(objects, targets=None):
    return SceneSnapshot("s", 1.0, tuple(objects), tuple(targets or [SceneObject("blue-box", "盒子", "蓝色", Pose("base", .5, .05, .02))]), "test")


def test_unknown_requested_object_requires_clarification():
    plan = CodeAsPoliciesPlanner().plan("把绿色方块放进蓝色盒子", MockSceneProvider().observe())
    assert plan.clarification
    assert plan.intent is None


def test_fallback_matches_requested_object_after_reordering():
    objects = [
        SceneObject("red-block", "方块", "红色", Pose("base", .3, .05, .03)),
        SceneObject("green-block", "方块", "绿色", Pose("base", .2, .05, .03)),
    ]
    plan = CodeAsPoliciesPlanner().plan("把绿色方块放进蓝色盒子", scene(objects))
    assert plan.intent.source_object_id == "green-block"


def test_ambiguous_and_negative_instructions_do_not_execute():
    objects = [
        SceneObject("red-1", "方块", "红色", Pose("base", .3, .05, .03)),
        SceneObject("red-2", "方块", "红色", Pose("base", .2, .05, .03)),
    ]
    planner = CodeAsPoliciesPlanner()
    assert planner.plan("把红色方块放进蓝色盒子", scene(objects)).clarification
    assert planner.plan("不要抓红色方块", scene(objects)).clarification


@pytest.mark.parametrize("response", [
    {"tool": "pick_and_place", "arguments": {}},
    {"tool": "pick_and_place", "arguments": {"source_object_id": 3, "target_object_id": "blue-box"}},
    {"tool": "pick_and_place", "arguments": {"source_object_id": "", "target_object_id": "blue-box"}},
    {"tool": "not_a_tool", "arguments": {"source_object_id": "red-block", "target_object_id": "blue-box"}},
])
def test_invalid_model_tool_arguments_are_rejected(response):
    planner = CodeAsPoliciesPlanner(lambda _: response)
    with pytest.raises(ValueError):
        planner.plan("把红色方块放进蓝色盒子", MockSceneProvider().observe())


class NoHeldObjectRobot:
    __slots__ = ("pose", "opening")

    def __init__(self):
        self.pose = None
        self.opening = .08

    def move_to(self, pose):
        self.pose = pose
        return True

    def set_gripper(self, opening):
        self.opening = opening
        return False if opening > .05 else True

    def get_state(self):
        return {"pose": self.pose, "gripper_opening_m": self.opening}

    def cancel(self):
        pass


def test_gripper_open_failure_is_not_success_and_backend_needs_no_held_object():
    robot = NoHeldObjectRobot()
    result = pick_and_place(robot, MockSceneProvider().observe(), __import__("robot_pick_place_agent.core.models", fromlist=["TaskIntent"]).TaskIntent("x", "red-block", "blue-box", "mock-scene-1"))
    assert result.status is ActionStatus.FAILED
    assert result.stage == "release"


class UnknownGripperResultRobot(NoHeldObjectRobot):
    def set_gripper(self, opening):
        self.opening = opening
        return None


def test_unconfirmed_gripper_result_is_uncertain():
    robot = UnknownGripperResultRobot()
    intent = __import__("robot_pick_place_agent.core.models", fromlist=["TaskIntent"]).TaskIntent("x", "red-block", "blue-box", "mock-scene-1")
    result = pick_and_place(robot, MockSceneProvider().observe(), intent)
    assert result.status is ActionStatus.UNCERTAIN


class AlternateRobot:
    """A RobotPort-compatible backend deliberately without held_object."""
    def __init__(self):
        self.pose = None
        self.opening = .08

    def move_to(self, pose):
        self.pose = pose
        return True

    def set_gripper(self, opening):
        self.opening = opening
        return True

    def get_state(self):
        return {"pose": self.pose, "gripper_opening_m": self.opening}

    def cancel(self):
        return None


def test_robotport_compatible_backend_can_complete_without_held_object():
    result = pick_and_place(AlternateRobot(), MockSceneProvider().observe(), TaskIntent("x", "red-block", "blue-box", "mock-scene-1"))
    assert result.status is ActionStatus.UNCERTAIN
    assert result.evidence["holding_inferred"] is True


def test_application_does_not_execute_after_invalid_model_response():
    from robot_pick_place_agent.runtime.application import Application
    robot = AlternateRobot()
    planner = CodeAsPoliciesPlanner(lambda _: {"tool": "pick_and_place", "arguments": {"source_object_id": 1, "target_object_id": "blue-box"}})
    result = Application(robot=robot, planner=planner).run("把红色方块放进蓝色盒子")
    assert result.status is ActionStatus.FAILED
    assert result.stage == "plan"
    assert robot.pose is None


def test_cli_run_exit_code_reflects_plan_failure(capsys):
    from robot_pick_place_agent.cli.main import main
    assert main(["run", "把绿色方块放进蓝色盒子"]) == 1
    assert '"status": "failed"' in capsys.readouterr().out
