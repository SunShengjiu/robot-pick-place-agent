from robot_pick_place_agent.core.models import Pose


class MockRobot:
    def __init__(self):
        self.pose = Pose("base", 0.0, 0.0, 0.3)
        self.gripper_opening_m = 0.08
        self.held_object = None
        self.history = []

    def move_to(self, pose: Pose) -> bool:
        self.pose = pose
        self.history.append(("move_to", pose))
        return True

    def set_gripper(self, opening_m: float) -> bool:
        self.gripper_opening_m = opening_m
        self.history.append(("gripper", opening_m))
        return True

    def get_state(self) -> dict:
        return {"pose": self.pose, "gripper_opening_m": self.gripper_opening_m, "held_object": self.held_object}

    def cancel(self) -> None:
        self.history.append(("cancel",))
