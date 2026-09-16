"""Code as Policies style planning with a strict structured-tool boundary.

The original CaP idea is used here for decomposition: language selects a
small policy/tool, while deterministic skill code performs the physical work.
No generated source code is evaluated.
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from typing import Any, Callable

from robot_pick_place_agent.core.models import SceneSnapshot, TaskIntent
from .tools import TOOL_DEFINITIONS, ToolCall, validate_tool_call


class PlanningError(ValueError):
    pass


@dataclass(frozen=True)
class TaskPlan:
    instruction: str
    scene_id: str
    tool_calls: tuple[ToolCall, ...]
    intent: TaskIntent | None = None
    clarification: str | None = None
    policy_source: str = "cap_deterministic_fallback"


def build_prompt(instruction: str, scene: SceneSnapshot) -> str:
    objects = [
        {"id": o.object_id, "name": o.name, "color": o.color, "kind": "object"}
        for o in scene.objects
    ] + [
        {"id": o.object_id, "name": o.name, "color": o.color, "kind": "target"}
        for o in scene.targets
    ]
    return (
        "You are a robot task planner using Code as Policies style decomposition. "
        "Choose exactly one finite tool; never emit code or robot commands.\n"
        f"Instruction: {instruction}\nScene: {json.dumps(objects, ensure_ascii=False)}\n"
        f"Tools: {json.dumps(TOOL_DEFINITIONS, ensure_ascii=False)}\n"
        "Return JSON only: {\"tool\": string, \"arguments\": object}."
    )


class CodeAsPoliciesPlanner:
    """Planner with an injectable LLM completion and a safe local fallback.

    `completion` receives the prompt and returns JSON text (or a decoded dict).
    Keeping this dependency injectable lets API, local, and test models share
    the exact same validation and execution boundary.
    """

    def __init__(self, completion: Callable[[str], str | dict[str, Any]] | None = None):
        self.completion = completion

    def plan(self, instruction: str, scene: SceneSnapshot) -> TaskPlan:
        if not isinstance(instruction, str) or not instruction.strip():
            raise PlanningError("instruction must be a non-empty string")
        if self.completion is None:
            call = self._fallback(instruction, scene)
            source = "cap_deterministic_fallback"
        else:
            call = self._parse(self.completion(build_prompt(instruction, scene)))
            source = "cap_model_tool_call"
        try:
            validate_tool_call(call, scene)
        except ValueError as exc:
            raise PlanningError(str(exc)) from exc
        if call.name == "request_clarification":
            return TaskPlan(instruction, scene.scene_id, (call,), clarification=call.arguments["question"], policy_source=source)
        intent = TaskIntent(instruction, call.arguments["source_object_id"], call.arguments["target_object_id"], scene.scene_id)
        return TaskPlan(instruction, scene.scene_id, (call,), intent=intent, policy_source=source)

    @staticmethod
    def _parse(value: str | dict[str, Any]) -> ToolCall:
        if isinstance(value, dict):
            data = value
        elif isinstance(value, str):
            text = re.sub(r"^```(?:json)?\s*|\s*```$", "", value.strip(), flags=re.IGNORECASE)
            try:
                data = json.loads(text)
            except json.JSONDecodeError as exc:
                raise PlanningError("planner response was not valid JSON") from exc
        else:
            raise PlanningError("planner response must be JSON text or an object")
        if not isinstance(data, dict) or set(data) != {"tool", "arguments"} or not isinstance(data.get("tool"), str) or not isinstance(data.get("arguments"), dict):
            raise PlanningError("planner response must contain tool and arguments")
        return ToolCall(data["tool"], data["arguments"])

    @staticmethod
    def _fallback(instruction: str, scene: SceneSnapshot) -> ToolCall:
        text = instruction.strip()
        if re.search(r"不|别|勿|禁止|\b(?:not|never|without|except)\b|n't", text, re.IGNORECASE):
            return ToolCall("request_clarification", {"question": "指令包含否定表达，已拒绝执行；请明确发送要执行的抓放指令。"})
        match = re.fullmatch(r"(?:请\s*)?(?:把|将)\s*(.+?)\s*(?:放进|放入|放到)\s*(.+)", text)
        if match is None:
            match = re.fullmatch(r"(?:please\s+)?(?:put|place|move)\s+(?:the\s+)?(.+?)\s+(?:into|in)\s+(?:the\s+)?(.+)", text, flags=re.IGNORECASE)
        if match is None:
            return ToolCall("request_clarification", {"question": "暂不支持该表达。请使用“把<物体>放进<目标>”的单步指令。"})
        source_matches = _match_objects(match[1], scene.objects)
        target_matches = _match_objects(match[2], scene.targets)
        if len(source_matches) != 1 or len(target_matches) != 1:
            return ToolCall("request_clarification", {"question": "请明确唯一的来源物体和放置目标。"})
        return ToolCall("pick_and_place", {"source_object_id": source_matches[0].object_id, "target_object_id": target_matches[0].object_id})


_COLOR_ALIASES = (("红色", "红", "red"), ("绿色", "绿", "green"), ("蓝色", "蓝", "blue"), ("黄色", "黄", "yellow"), ("紫色", "紫", "purple"))
_NAME_ALIASES = (("方块", "block", "cube"), ("盒子", "盒", "box", "bin"))


def _match_objects(description, items):
    description = re.sub(r"\s+", "", description).casefold()
    matches = []
    for item in items:
        color = next((group for group in _COLOR_ALIASES if item.color.casefold() in group), (item.color.casefold(),))
        name = next((group for group in _NAME_ALIASES if item.name.casefold() in group), (item.name.casefold(),))
        candidates = {item.object_id.casefold(), *color, *name}
        candidates.update(c + n for c in color for n in name)
        if description in {re.sub(r"\s+", "", candidate) for candidate in candidates}:
            matches.append(item)
    return matches
