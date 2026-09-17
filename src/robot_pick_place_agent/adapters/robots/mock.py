from robot_pick_place_agent.core.models import Pose


class MockRobot:
    def __init__(self):
        self.pose = Pose("base", 0.0, 0.0, 0.3)
        self.gripper_opening_m = 0.08
        self.held_object = None
        self.history = []
        self.gripper_opening_limits_m = (0.0, 0.08)

    def move_to(self, pose: Pose) -> bool:
        self.pose = pose
        self.history.append(("move_to", pose))
        return True

    def set_gripper(self, opening_m: float) -> bool:
        self.gripper_opening_m = opening_m
        self.history.append(("gripper", opening_m))
        return True

    def get_state(self) -> dict:
        return {"pose": self.pose, "gripper_opening_m": self.gripper_opening_m,
                "gripper_opening_limits_m": list(self.gripper_opening_limits_m),
                "held_object": self.held_object}

    def verify_placement(self, source_object_id, target_object_id, target_pose, tolerance_m):
        """Explicit simulated evidence; this is not physical success evidence."""
        return {"placement_verified": True, "placed_object_id": source_object_id,
                "placement_target_id": target_object_id, "placement_position": target_pose,
                "placement_error_m": 0.0, "held_object": None,
                "verification_source": "mock_simulated_feedback"}

    def cancel(self) -> None:
        self.history.append(("cancel",))
