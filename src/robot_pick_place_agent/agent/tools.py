"""Finite tools exposed to the language planner.

The model may select these high-level operations, but it cannot emit Python,
joint commands, or arbitrary simulator calls.
"""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ToolCall:
    name: str
    arguments: dict[str, Any]


TOOL_DEFINITIONS = (
    {
        "name": "pick_and_place",
        "description": "Pick one observed object and place it in one observed target.",
        "parameters": {
            "type": "object",
            "properties": {
                "source_object_id": {"type": "string"},
                "target_object_id": {"type": "string"},
            },
            "required": ["source_object_id", "target_object_id"],
            "additionalProperties": False,
        },
    },
    {
        "name": "request_clarification",
        "description": "Ask the user to disambiguate an object or target.",
        "parameters": {
            "type": "object",
            "properties": {"question": {"type": "string"}},
            "required": ["question"],
            "additionalProperties": False,
        },
    },
)


def validate_tool_call(call: ToolCall, scene) -> None:
    if not isinstance(call, ToolCall) or not isinstance(call.name, str) or not call.name.strip():
        raise ValueError("planner tool name must be a non-empty string")
    if not isinstance(call.arguments, dict):
        raise ValueError("planner arguments must be an object")
    if call.name not in {item["name"] for item in TOOL_DEFINITIONS}:
        raise ValueError(f"unsupported planner tool: {call.name}")
    if call.name == "pick_and_place":
        required = {"source_object_id", "target_object_id"}
        if set(call.arguments) != required:
            raise ValueError("pick_and_place requires exactly source_object_id and target_object_id")
        object_ids = {obj.object_id for obj in scene.objects}
        target_ids = {obj.object_id for obj in scene.targets}
        source = call.arguments.get("source_object_id")
        target = call.arguments.get("target_object_id")
        if not all(isinstance(value, str) and value.strip() for value in (source, target)):
            raise ValueError("pick_and_place IDs must be non-empty strings")
        if source not in object_ids or target not in target_ids:
            raise ValueError("planner selected an object or target outside the observed scene")
        if sum(obj.object_id == source for obj in scene.objects) != 1 or sum(obj.object_id == target for obj in scene.targets) != 1:
            raise ValueError("planner selected ambiguous object IDs")
    elif call.name == "request_clarification":
        if set(call.arguments) != {"question"}:
            raise ValueError("request_clarification requires exactly question")
        question = call.arguments["question"]
        if not isinstance(question, str) or not question.strip():
            raise ValueError("clarification question must be a non-empty string")
