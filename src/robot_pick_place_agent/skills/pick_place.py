from robot_pick_place_agent.core.models import ActionResult, ActionStatus, SceneSnapshot, TaskIntent, Pose


def pick_and_place(robot, scene: SceneSnapshot, intent: TaskIntent, task_id: str = "task-1") -> ActionResult:
    source = next((o for o in scene.objects if o.object_id == intent.source_object_id), None)
    target = next((o for o in scene.targets if o.object_id == intent.target_object_id), None)
    if not source or not target or intent.scene_id != scene.scene_id:
        return ActionResult(task_id, ActionStatus.FAILED, "validate", "对象或场景版本无效")
    above = Pose(source.pose.frame_id, source.pose.x, source.pose.y, source.pose.z + .12)
    if not robot.move_to(above) or not robot.move_to(source.pose) or not robot.set_gripper(0.0):
        return ActionResult(task_id, ActionStatus.FAILED, "pick", "抓取动作失败")
    robot.held_object = source.object_id
    if not robot.move_to(above) or not robot.move_to(Pose(target.pose.frame_id, target.pose.x, target.pose.y, target.pose.z + .12)) or not robot.move_to(target.pose):
        return ActionResult(task_id, ActionStatus.FAILED, "place", "放置动作失败")
    robot.set_gripper(0.08)
    robot.held_object = None
    return ActionResult(task_id, ActionStatus.SUCCEEDED, "verify", "已完成抓取放置", {"source": source.object_id, "target": target.object_id})
