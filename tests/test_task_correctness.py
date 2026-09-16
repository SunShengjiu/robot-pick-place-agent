"""Regression checks for selection, validation, and truthful task outcomes."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from robot_pick_place_agent.agent.planner import CodeAsPoliciesPlanner, PlanningError
from robot_pick_place_agent.core.models import ActionStatus, Pose, SceneObject, SceneSnapshot, TaskIntent
from robot_pick_place_agent.perception.mock_scene import MockSceneProvider
from robot_pick_place_agent.runtime.application import Application
from robot_pick_place_agent.skills.pick_place import pick_and_place


INSTRUCTION = "把红色方块放进蓝色盒子"


@pytest.mark.parametrize("reverse_objects", [False, True])
@pytest.mark.parametrize("reverse_targets", [False, True])
def test_source_and_target_are_matched_independently(reverse_objects, reverse_targets):
    objects = [SceneObject("red-block", "方块", "红色", Pose("base", .3, 0, .03)),
               SceneObject("blue-block", "方块", "蓝色", Pose("base", .2, 0, .03))]
    targets = [SceneObject("red-box", "盒子", "红色", Pose("base", .5, 0, .03)),
               SceneObject("blue-box", "盒子", "蓝色", Pose("base", .5, .1, .03))]
    scene = SceneSnapshot("s", 1, tuple(objects[:: -1 if reverse_objects else 1]),
                          tuple(targets[:: -1 if reverse_targets else 1]))
    plan = CodeAsPoliciesPlanner().plan(INSTRUCTION, scene)
    assert plan.intent is not None
    assert (plan.intent.source_object_id, plan.intent.target_object_id) == ("red-block", "blue-box")


@pytest.mark.parametrize("instruction", [
    "把红色球放进蓝色盒子", "把红色方块放进绿色盒子",
    "把红色方块放进蓝色杯子", "把紫色方块放进蓝色盒子",
    "不要把红色方块放进蓝色盒子", "别把红色方块放进蓝色盒子",
    "把红色方块放进蓝色盒子，但不要执行", "红色方块和蓝色盒子在哪里？",
    "推动红色方块撞向蓝色盒子", "把红色方块放进蓝色盒子再拿出来",
    "把红色方块放进蓝色盒子吗？", "red block blue box",
    "do not put the red block into the blue box",
])
def test_unsupported_missing_or_negated_request_never_moves(instruction):
    robot = ScriptedRobot()
    result = Application(robot=robot).run(instruction)
    assert result.status is ActionStatus.FAILED
    assert result.stage == "plan"
    assert robot.calls == []


@pytest.mark.parametrize("response", [
    None, 123, [], "null", "[]", "42", '"text"',
    {"tool": [], "arguments": {}}, {"tool": " ", "arguments": {}},
    {"tool": "pick_and_place", "arguments": {"source_object_id": [], "target_object_id": "blue-box"}},
    {"tool": "request_clarification", "arguments": {}},
    {"tool": "request_clarification", "arguments": {"question": None}},
    {"tool": "request_clarification", "arguments": {"question": "  "}},
    {"tool": "request_clarification", "arguments": {"question": "哪一个？", "extra": True}},
])
def test_malformed_model_response_has_a_planning_error_and_no_motion(response):
    planner = CodeAsPoliciesPlanner(lambda _: response)
    with pytest.raises(PlanningError):
        planner.plan(INSTRUCTION, MockSceneProvider().observe())
    robot = ScriptedRobot()
    result = Application(robot=robot, planner=planner).run(INSTRUCTION)
    assert result.status is ActionStatus.FAILED
    assert result.stage == "plan"
    assert robot.calls == []


class ScriptedRobot:
    """Only RobotPort methods and slots: no writable held_object field."""
    __slots__ = ("calls", "failure_at", "outcome")

    def __init__(self, failure_at=None, outcome=False):
        self.calls = []
        self.failure_at = failure_at
        self.outcome = outcome

    def _record(self, command, value):
        self.calls.append((command, value))
        if len(self.calls) - 1 == self.failure_at:
            if isinstance(self.outcome, Exception):
                raise self.outcome
            return self.outcome
        return True

    def move_to(self, pose):
        return self._record("move_to", pose)

    def set_gripper(self, opening_m):
        return self._record("gripper", opening_m)

    def get_state(self):
        return {"device_feedback": "no object sensor"}

    def cancel(self):
        self.calls.append(("cancel", None))


def execute(robot):
    scene = MockSceneProvider().observe()
    return pick_and_place(robot, scene, TaskIntent(INSTRUCTION, "red-block", "blue-box", scene.scene_id))


@pytest.mark.parametrize("index", range(7))
@pytest.mark.parametrize("outcome, expected", [(False, ActionStatus.FAILED), (None, ActionStatus.UNCERTAIN),
                                               (TimeoutError("no acknowledgement"), ActionStatus.UNCERTAIN),
                                               ("success", ActionStatus.UNCERTAIN)])
def test_every_command_is_checked_and_execution_stops(index, outcome, expected):
    robot = ScriptedRobot(index, outcome)
    result = execute(robot)
    assert result.status is expected
    assert len(robot.calls) == index + 1
    assert result.stage == ("pick" if index < 3 else "place" if index < 6 else "release")


def test_completed_commands_do_not_prove_physical_placement():
    robot = ScriptedRobot()
    result = execute(robot)
    assert result.status is ActionStatus.UNCERTAIN
    assert result.stage == "verify"
    assert len(robot.calls) == 7
    assert result.evidence["command_sequence_completed"] is True
    assert result.evidence["physical_success_confirmed"] is False


def test_mock_result_explicitly_describes_only_a_mock_flow():
    from robot_pick_place_agent.adapters.robots.mock import MockRobot
    result = Application(robot=MockRobot()).run(INSTRUCTION)
    assert result.status is ActionStatus.SUCCEEDED
    assert result.evidence["execution_mode"] == "mock"
    assert result.evidence["result_scope"] == "mock_flow"
    assert result.evidence["physical_success_confirmed"] is False
    assert "模拟" in result.message


@pytest.mark.parametrize("command, instruction, code", [
    ("run", INSTRUCTION, 0), ("run", "把绿色方块放进蓝色盒子", 1),
    ("plan", "把绿色方块放进蓝色盒子", 1),
])
def test_module_exit_code_and_json(command, instruction, code):
    root = Path(__file__).resolve().parents[1]
    env = {**os.environ, "PYTHONPATH": str(root / "src")}
    result = subprocess.run([sys.executable, "-m", "robot_pick_place_agent.cli.main", command, instruction],
                            cwd=root, env=env, capture_output=True, text=True)
    assert result.returncode == code, result.stderr
    assert isinstance(json.loads(result.stdout), dict)
    assert result.stderr == ""
