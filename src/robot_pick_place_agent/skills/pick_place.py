from robot_pick_place_agent.core.models import ActionResult, ActionStatus, SceneSnapshot, TaskIntent, Pose


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


def pick_and_place(robot, scene: SceneSnapshot, intent: TaskIntent, task_id: str = "task-1", evidence_policy: str = "direct") -> ActionResult:
    source = next((o for o in scene.objects if o.object_id == intent.source_object_id), None)
    target = next((o for o in scene.targets if o.object_id == intent.target_object_id), None)
    if not source or not target or intent.scene_id != scene.scene_id:
        return ActionResult(task_id, ActionStatus.FAILED, "validate", "对象或场景版本无效")
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
    status = _action(robot, robot.set_gripper, 0.08)
    evidence = {"source": source.object_id, "target": target.object_id, "policy_source": evidence_policy, "holding_inferred": inferred_holding, "robot_feedback": _feedback(robot)}
    if status is ActionStatus.FAILED:
        return ActionResult(task_id, status, "release", "松开夹爪失败", evidence)
    if status is ActionStatus.UNCERTAIN:
        return ActionResult(task_id, status, "release", "无法确认夹爪是否已松开", evidence)
    feedback = evidence["robot_feedback"]
    evidence.update({"command_sequence_completed": True, "physical_success_confirmed": False})
    if not isinstance(feedback, dict) or not any(key in feedback for key in ("held_object", "holding", "object_attached", "placement_verified")):
        return ActionResult(task_id, ActionStatus.UNCERTAIN, "verify", "动作序列完成，但设备反馈无法确认实际抓放结果", evidence)
    return ActionResult(task_id, ActionStatus.SUCCEEDED, "verify", "已完成抓取放置", evidence)
