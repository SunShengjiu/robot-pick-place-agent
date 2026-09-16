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
        if self.completion is None:
            call = self._fallback(instruction, scene)
            source = "cap_deterministic_fallback"
        else:
            call = self._parse(self.completion(build_prompt(instruction, scene)))
            source = "cap_model_tool_call"
        validate_tool_call(call, scene)
        if call.name == "request_clarification":
            return TaskPlan(instruction, scene.scene_id, (call,), clarification=call.arguments["question"], policy_source=source)
        intent = TaskIntent(instruction, call.arguments["source_object_id"], call.arguments["target_object_id"], scene.scene_id)
        return TaskPlan(instruction, scene.scene_id, (call,), intent=intent, policy_source=source)

    @staticmethod
    def _parse(value: str | dict[str, Any]) -> ToolCall:
        if isinstance(value, dict):
            data = value
        else:
            text = re.sub(r"^```(?:json)?\s*|\s*```$", "", value.strip(), flags=re.IGNORECASE)
            try:
                data = json.loads(text)
            except json.JSONDecodeError as exc:
                raise PlanningError("planner response was not valid JSON") from exc
        if "tool" not in data or not isinstance(data.get("arguments"), dict):
            raise PlanningError("planner response must contain tool and arguments")
        return ToolCall(str(data["tool"]), data["arguments"])

    @staticmethod
    def _fallback(instruction: str, scene: SceneSnapshot) -> ToolCall:
        def find(items, words):
            for item in items:
                haystack = f"{item.color}{item.name}{item.object_id}"
                if any(word in haystack or word in instruction for word in words):
                    if any(word in instruction for word in words):
                        return item
            return None

        source = find(scene.objects, ("红", "red", "方块", "block"))
        target = find(scene.targets, ("蓝", "blue", "盒", "box", "target"))
        if source is None or target is None:
            return ToolCall("request_clarification", {"question": "请明确要抓取的物体和放置目标。"})
        return ToolCall("pick_and_place", {"source_object_id": source.object_id, "target_object_id": target.object_id})
