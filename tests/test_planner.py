import pytest

from robot_pick_place_agent.agent.planner import CodeAsPoliciesPlanner, PlanningError, build_prompt
from robot_pick_place_agent.perception.mock_scene import MockSceneProvider


def test_cap_fallback_returns_finite_tool_call():
    scene = MockSceneProvider().observe()
    plan = CodeAsPoliciesPlanner().plan("把红色方块放进蓝色盒子", scene)
    assert plan.tool_calls[0].name == "pick_and_place"
    assert plan.intent.source_object_id == "red-block"
    assert plan.policy_source == "cap_deterministic_fallback"


def test_model_response_is_validated_against_scene():
    scene = MockSceneProvider().observe()
    planner = CodeAsPoliciesPlanner(lambda _: {"tool": "pick_and_place", "arguments": {"source_object_id": "not-seen", "target_object_id": "blue-box"}})
    with pytest.raises(ValueError, match="outside the observed scene"):
        planner.plan("抓取它", scene)


def test_model_response_cannot_execute_code():
    scene = MockSceneProvider().observe()
    planner = CodeAsPoliciesPlanner(lambda _: "```python\nrobot.move_to(...)\n```")
    with pytest.raises(PlanningError):
        planner.plan("随便做", scene)


def test_prompt_contains_scene_and_no_arbitrary_code_contract():
    scene = MockSceneProvider().observe()
    prompt = build_prompt("把红色方块放进蓝色盒子", scene)
    assert "red-block" in prompt and "pick_and_place" in prompt
    assert "never emit code" in prompt
