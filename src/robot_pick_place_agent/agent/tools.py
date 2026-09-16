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
    if call.name not in {item["name"] for item in TOOL_DEFINITIONS}:
        raise ValueError(f"unsupported planner tool: {call.name}")
    if call.name == "pick_and_place":
        object_ids = {obj.object_id for obj in scene.objects}
        target_ids = {obj.object_id for obj in scene.targets}
        source = call.arguments.get("source_object_id")
        target = call.arguments.get("target_object_id")
        if source not in object_ids or target not in target_ids:
            raise ValueError("planner selected an object or target outside the observed scene")
