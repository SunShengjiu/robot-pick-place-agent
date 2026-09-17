from typing import Protocol
from .models import Pose, SceneSnapshot, ActionResult


class RobotPort(Protocol):
    def move_to(self, pose: Pose) -> bool: ...
    def set_gripper(self, opening_m: float) -> bool: ...
    def get_state(self) -> dict: ...
    def cancel(self) -> None: ...


class JointRobotPort(RobotPort, Protocol):
    """Optional arm commissioning capability; SI radians in declared joint order."""

    def move_joints(self, positions_rad: tuple[float, ...]) -> bool: ...


class SceneProviderPort(Protocol):
    def observe(self) -> SceneSnapshot: ...


class RunRecorderPort(Protocol):
    def record(self, event: str, **data) -> None: ...


class TaskPlannerPort(Protocol):
    def plan(self, instruction: str, scene: SceneSnapshot): ...
