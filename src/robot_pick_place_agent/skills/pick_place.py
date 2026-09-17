from robot_pick_place_agent.core.models import ActionResult, ActionStatus, SceneSnapshot, TaskIntent, Pose
import math


def _action(robot, method, *args):
    """Normalize RobotPort outcomes without assuming a concrete backend."""
    try:
        outcome = method(*args)
    except Exception:
        return ActionStatus.UNCERTAIN
    if outcome is True:
        return ActionStatus.SUCCEEDED
    if outcome is False:
        return ActionStatus.FAILED
    return ActionStatus.UNCERTAIN


def _feedback(robot):
    try:
        state = robot.get_state()
    except Exception as exc:  # backend feedback is optional but uncertainty is explicit
        return {"feedback_error": str(exc)}
    return state if isinstance(state, dict) else {"feedback": state}


def _configured_release_opening(robot, feedback):
    """Read the legal release opening from the device contract, never a skill constant."""
    limits = feedback.get("gripper_opening_limits_m") if isinstance(feedback, dict) else None
    if limits is None:
        try:
            diagnostics = robot.diagnostics()
        except Exception:
            diagnostics = {}
        limits = diagnostics.get("gripper_opening_limits_m") if isinstance(diagnostics, dict) else None
    if isinstance(limits, (list, tuple)) and len(limits) == 2:
        try:
            low, high = (float(limits[0]), float(limits[1]))
        except (TypeError, ValueError):
            return None, "invalid_device_configuration"
        if math.isfinite(low) and math.isfinite(high) and 0 <= low <= high:
            return high, "device_configuration"
        return None, "invalid_device_configuration"
    # Legacy backends sometimes report only their current opening. Reuse that
    # device value for the command, but never treat it as a legal limit or
    # placement evidence. A backend with neither value is probed at its neutral
    # zero value solely to preserve the old RobotPort call/feedback contract.
    if isinstance(feedback, dict) and "gripper_opening_m" in feedback:
        try:
            current = float(feedback["gripper_opening_m"])
            if math.isfinite(current) and current >= 0:
                return current, "legacy_current_state"
        except (TypeError, ValueError):
            pass
    return 0.0, "missing_device_configuration"


def _placement_evidence(feedback, source, target):
    """Require release, object/target identity and a target-position measurement."""
    if not isinstance(feedback, dict):
        return False, "feedback_not_mapping"
    nested = feedback.get("placement") if isinstance(feedback.get("placement"), dict) else {}
    merged = {**nested, **feedback}
    if merged.get("placement_verified") is not True:
        return False, "placement_verified_not_true"
    if merged.get("holding") is True or merged.get("object_attached") is True:
        return False, "object_still_attached"
    held = merged.get("held_object")
    if held not in (None, False, ""):
        return False, "held_object_present"
    placed_id = merged.get("placed_object_id", merged.get("released_object_id"))
    target_id = merged.get("placement_target_id", merged.get("target_object_id"))
    if placed_id != source.object_id or target_id != target.object_id:
        return False, "placement_identity_mismatch"
    error = merged.get("placement_error_m")
    if error is not None:
        try:
            if math.isfinite(float(error)) and 0 <= float(error) <= max(float(target.radius), .02):
                return True, "target_position_error"
        except (TypeError, ValueError):
            pass
        return False, "target_position_error_exceeds_tolerance"
    position = merged.get("placement_position")
    if isinstance(position, Pose):
        xyz = (position.x, position.y, position.z)
    elif isinstance(position, dict) and all(key in position for key in ("x", "y", "z")):
        xyz = (position["x"], position["y"], position["z"])
    elif isinstance(position, (list, tuple)) and len(position) >= 3:
        xyz = position[:3]
    else:
        return False, "target_position_evidence_missing"
    if isinstance(position, Pose) and position.frame_id != target.pose.frame_id:
        return False, "target_position_frame_mismatch"
    try:
        distance = math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(xyz, (target.pose.x, target.pose.y, target.pose.z))))
    except (TypeError, ValueError):
        return False, "target_position_invalid"
    return (True, "target_position") if distance <= max(float(target.radius), .02) else (False, "target_position_mismatch")


def pick_and_place(robot, scene: SceneSnapshot, intent: TaskIntent, task_id: str = "task-1", evidence_policy: str = "direct") -> ActionResult:
    source = next((o for o in scene.objects if o.object_id == intent.source_object_id), None)
    target = next((o for o in scene.targets if o.object_id == intent.target_object_id), None)
    if not source or not target or intent.scene_id != scene.scene_id:
        return ActionResult(task_id, ActionStatus.FAILED, "validate", "对象或场景版本无效")
    initial_feedback = _feedback(robot)
    above = Pose(source.pose.frame_id, source.pose.x, source.pose.y, source.pose.z + .12)
    for pose in (above, source.pose):
        status = _action(robot, robot.move_to, pose)
        if status is not ActionStatus.SUCCEEDED:
            return ActionResult(task_id, status, "pick", "到达抓取位置失败或无法确认")
    status = _action(robot, robot.set_gripper, 0.0)
    if status is not ActionStatus.SUCCEEDED:
        return ActionResult(task_id, status, "pick", "闭合夹爪失败或无法确认")
    inferred_holding = True
    feedback = _feedback(robot)
    carry_poses = (above, Pose(target.pose.frame_id, target.pose.x, target.pose.y, target.pose.z + .12), target.pose)
    for pose in carry_poses:
        status = _action(robot, robot.move_to, pose)
        if status is not ActionStatus.SUCCEEDED:
            return ActionResult(task_id, status, "place", "搬运或到达放置位置失败或无法确认", {"holding_inferred": inferred_holding, "robot_feedback": feedback})
    release_feedback = _feedback(robot)
    release_opening, opening_source = _configured_release_opening(robot, release_feedback)
    if opening_source in {"missing_device_configuration", "legacy_current_state"} and isinstance(initial_feedback, dict):
        try:
            initial_opening = float(initial_feedback.get("gripper_opening_m"))
            if math.isfinite(initial_opening) and initial_opening >= 0:
                release_opening, opening_source = initial_opening, "legacy_initial_state"
        except (TypeError, ValueError):
            pass
    evidence = {"source": source.object_id, "target": target.object_id, "policy_source": evidence_policy,
                "holding_inferred": inferred_holding, "release_opening_m": release_opening,
                "release_opening_source": opening_source, "robot_feedback_before_release": release_feedback}
    status = _action(robot, robot.set_gripper, release_opening)
    evidence["robot_feedback"] = _feedback(robot)
    if status is ActionStatus.FAILED:
        return ActionResult(task_id, status, "release", "松开夹爪失败", evidence)
    if status is ActionStatus.UNCERTAIN:
        return ActionResult(task_id, status, "release", "无法确认夹爪是否已松开", evidence)
    if opening_source != "device_configuration":
        evidence["placement_verification"] = {"verified": False, "reason": "release_opening_configuration_missing"}
        evidence.update({"command_sequence_completed": True, "physical_success_confirmed": False})
        return ActionResult(task_id, ActionStatus.UNCERTAIN, "verify", "设备未提供合法夹爪开度配置，不能确认释放", evidence)
    feedback = evidence["robot_feedback"]
    verifier = getattr(robot, "verify_placement", None)
    if callable(verifier):
        try:
            verification = verifier(source_object_id=source.object_id, target_object_id=target.object_id,
                                    target_pose=target.pose, tolerance_m=max(float(target.radius), .02))
        except Exception as exc:
            verification = {"verification_error": str(exc)}
        if isinstance(verification, dict):
            feedback = {**feedback, **verification}
            evidence["robot_feedback"] = feedback
    verified, reason = _placement_evidence(feedback, source, target)
    evidence["placement_verification"] = {"verified": verified, "reason": reason}
    evidence.update({"command_sequence_completed": True, "physical_success_confirmed": False})
    if not verified:
        return ActionResult(task_id, ActionStatus.UNCERTAIN, "verify", f"动作序列完成，但无法确认目标位置：{reason}", evidence)
    return ActionResult(task_id, ActionStatus.SUCCEEDED, "verify", "已完成抓取放置并确认目标位置", evidence)
