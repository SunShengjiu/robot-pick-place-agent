from dataclasses import dataclass, field
from enum import Enum
from typing import Any


@dataclass(frozen=True)
class Pose:
    frame_id: str
    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


@dataclass(frozen=True)
class SceneObject:
    object_id: str
    name: str
    color: str
    pose: Pose
    radius: float = 0.02


@dataclass(frozen=True)
class SceneSnapshot:
    scene_id: str
    observed_at: float
    objects: tuple[SceneObject, ...]
    targets: tuple[SceneObject, ...] = ()


@dataclass(frozen=True)
class TaskIntent:
    instruction: str
    source_object_id: str
    target_object_id: str
    scene_id: str


class ActionStatus(str, Enum):
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    UNCERTAIN = "uncertain"


@dataclass(frozen=True)
class ActionResult:
    task_id: str
    status: ActionStatus
    stage: str
    message: str = ""
    evidence: dict[str, Any] = field(default_factory=dict)
