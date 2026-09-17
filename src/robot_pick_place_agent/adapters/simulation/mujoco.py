"""Small, headless MuJoCo pick-and-place fixture.

The fixture deliberately uses the same RobotPort methods as a real adapter. A
free cube, a table, a bin and two sliding fingers are simulated with gravity,
contacts and friction; no state is teleported during the task.
"""
from __future__ import annotations

from robot_pick_place_agent.core.models import Pose


MODEL_XML = r"""
<mujoco model="pick_place">
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>
  <compiler angle="radian" autolimits="true"/>
  <default><geom contype="1" conaffinity="1" condim="3" margin="0" gap="0" solref="0.002 1" solimp="0.95 0.99 0.001" friction="5.0 0.5 0.1"/></default>
  <worldbody>
    <geom name="table" type="plane" pos="0 0 0" size="0.7 0.55 0.01" rgba="0.35 0.25 0.15 1"/>
    <camera name="overhead" pos="0 0 2.00" mode="targetbody" target="cube" fovy="1.57"/>
    <body name="cube" pos="0.30 0.05 0.07">
      <freejoint/><geom name="cube_geom" type="box" size="0.035 0.035 0.035" mass="0.08" rgba="0.9 0.1 0.1 1"/>
    </body>
    <body name="bin" pos="0.50 0.05 0.045">
      <geom type="box" pos="0 0 0" size="0.10 0.10 0.015" rgba="0.1 0.2 0.8 1"/>
      <geom type="box" pos="-.09 0 .07" size=".01 .10 .07"/>
      <geom type="box" pos=".09 0 .07" size=".01 .10 .07"/>
      <geom type="box" pos="0 -.09 .07" size=".08 .01 .07"/>
      <geom type="box" pos="0 .09 .07" size=".08 .01 .07"/>
    </body>
    <body name="gripper" mocap="true" pos="0 0 0.25">
      <geom name="palm" type="box" pos="0 0 0" size=".06 .07 .02" mass="0.2" contype="0" conaffinity="0"/>
    </body>
    <body name="left_finger" mocap="true" pos="0 .10 .10">
      <geom name="left_finger_geom" type="box" size=".025 .012 .05" mass="0.03"/>
    </body>
    <body name="right_finger" mocap="true" pos="0 0 .10">
      <geom name="right_finger_geom" type="box" size=".025 .012 .05" mass="0.03"/>
    </body>
  </worldbody>
</mujoco>
"""


class MujocoRobot:
    """RobotPort implementation backed by a kinematic MuJoCo gripper."""

    def __init__(self, xml: str = MODEL_XML, render: bool = False, xml_path: str | None = None, cube_start: tuple[float, float] = (0.30, 0.05)):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError("MuJoCo 未安装，请运行: pip install -e '.[simulation]'") from exc
        self.mujoco = mujoco
        self.model = (mujoco.MjModel.from_xml_path(xml_path) if xml_path else mujoco.MjModel.from_xml_string(xml))
        self.data = mujoco.MjData(self.model)
        self.render = render
        self._mocap = self.model.body("gripper").mocapid[0]
        self._left_mocap = self.model.body("left_finger").mocapid[0]
        self._right_mocap = self.model.body("right_finger").mocapid[0]
        self.gripper_center = [0.0, 0.0, 0.25]
        self.gripper_opening_m = 0.10
        self.max_speed_mps = 0.35
        self.max_opening_speed_mps = 0.20
        self._set_cube_start(cube_start)
        mujoco.mj_forward(self.model, self.data)

    def _step(self, seconds: float = 0.25):
        for _ in range(max(1, int(seconds / self.model.opt.timestep))):
            self.mujoco.mj_step(self.model, self.data)

    def _set_cube_start(self, start: tuple[float, float]):
        """Set the free body's initial state before the first simulation step."""
        if len(start) != 2 or not all(-0.5 < value < 0.8 for value in start):
            raise ValueError("cube_start must be a valid tabletop (x, y) pair")
        cube_jid = self.model.body("cube").jntadr[0]
        qpos = self.model.jnt_qposadr[cube_jid]
        self.data.qpos[qpos:qpos + 3] = [start[0], start[1], .07]
        self.data.qpos[qpos + 3:qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[:] = 0.0

    def _set_finger_positions(self):
        half = 0.047 + self.gripper_opening_m / 2.0
        cx, cy, cz = self.gripper_center
        self.data.mocap_pos[self._mocap] = [cx, cy, cz]
        self.data.mocap_pos[self._left_mocap] = [cx, cy + half, cz - .07]
        self.data.mocap_pos[self._right_mocap] = [cx, cy - half, cz - .07]

    def move_to(self, pose: Pose) -> bool:
        if pose.frame_id != "base":
            return False
        target = [pose.x, pose.y, pose.z]
        current = list(self.gripper_center)
        distance = sum((target[i] - current[i]) ** 2 for i in range(3)) ** 0.5
        duration = max(self.model.opt.timestep, distance / self.max_speed_mps)
        steps = max(1, int(duration / self.model.opt.timestep))
        for step in range(1, steps + 1):
            ratio = step / steps
            self.gripper_center = [current[i] + (target[i] - current[i]) * ratio for i in range(3)]
            self._set_finger_positions()
            self.mujoco.mj_step(self.model, self.data)
        return True

    def set_gripper(self, opening_m: float) -> bool:
        target = max(0.0, min(0.10, float(opening_m)))
        duration = abs(target - self.gripper_opening_m) / self.max_opening_speed_mps
        steps = max(1, int(duration / self.model.opt.timestep))
        start = self.gripper_opening_m
        for step in range(1, steps + 1):
            self.gripper_opening_m = start + (target - start) * step / steps
            self._set_finger_positions()
            self.mujoco.mj_step(self.model, self.data)
        return True

    def grasp(self) -> bool:
        """Return true only when both fingers physically contact the cube."""
        self._step(0.1)
        cube_geom = self.model.geom("cube_geom").id
        left_geom = self.model.geom("left_finger_geom").id
        right_geom = self.model.geom("right_finger_geom").id
        pairs = {frozenset((int(self.data.contact[i].geom1), int(self.data.contact[i].geom2))) for i in range(self.data.ncon)}
        return frozenset((cube_geom, left_geom)) in pairs and frozenset((cube_geom, right_geom)) in pairs

    def release(self) -> bool:
        return self.set_gripper(0.10)

    def diagnostics(self) -> dict:
        joints = []
        return {"joints": joints, "gripper_axis": (0.0, 1.0, 0.0), "gripper_opening_limits_m": [0.0, 0.10], "motion_limit_mps": self.max_speed_mps}

    def render_camera(self, width: int = 320, height: int = 240):
        """Render RGB pixels only; this does not expose simulator object poses."""
        renderer = self.mujoco.Renderer(self.model, height=height, width=width)
        renderer.update_scene(self.data, camera="overhead")
        image = renderer.render()
        renderer.close()
        return image

    def get_state(self) -> dict:
        cube = self.data.body("cube").xpos.copy()
        return {"cube_position": tuple(float(v) for v in cube), "time": float(self.data.time)}

    def cancel(self) -> None:
        self.set_gripper(0.08)

    def verify_in_bin(self) -> bool:
        x, y, z = self.data.body("cube").xpos
        return bool(0.42 < x < 0.58 and -0.03 < y < 0.13 and 0.03 < z < 0.16)


class MujocoCameraObserver:
    """Camera-only observer using rendered RGB and fixed tabletop calibration."""

    def __init__(self, robot: MujocoRobot):
        self.robot = robot

    def observe(self):
        import time
        import numpy as np
        from robot_pick_place_agent.core.models import SceneObject, SceneSnapshot

        image = self.robot.render_camera()
        red = (image[:, :, 0] > 60) & (image[:, :, 0] > image[:, :, 1] * 1.3) & (image[:, :, 0] > image[:, :, 2] * 1.3)
        blue = (image[:, :, 2] > 25) & (image[:, :, 2] >= image[:, :, 0] * 0.5)

        def centroid(mask):
            ys, xs = np.where(mask)
            return (float(xs.mean()), float(ys.mean()), int(len(xs))) if len(xs) else None

        red_c, blue_c = centroid(red), centroid(blue)
        if not red_c or not blue_c:
            raise RuntimeError("仿真相机未检测到红色方块或蓝色盒子")

        def image_to_table(c):
            # Calibration for the fixed orthographic overhead camera.
            u, v, _ = c
            return Pose("base", 0.70 * u / image.shape[1] - 0.10, 0.70 * (1.0 - v / image.shape[0]) - 0.35, 0.035)

        return SceneSnapshot(
            scene_id="mujoco-camera-scene",
            observed_at=time.time(),
            objects=(SceneObject("red-block", "方块", "红色", image_to_table(red_c)),),
            targets=(SceneObject("blue-box", "盒子", "蓝色", image_to_table(blue_c), .08),),
            observation_source="mujoco_camera_rgb",
        )


class MujocoStateObserver:
    """State observer: explicitly reads MuJoCo body poses for controller tuning."""

    def __init__(self, robot: MujocoRobot):
        self.robot = robot

    def observe(self):
        import time
        from robot_pick_place_agent.core.models import SceneObject, SceneSnapshot
        cube = self.robot.data.body("cube").xpos
        target = self.robot.data.body("bin").xpos
        return SceneSnapshot(
            scene_id="mujoco-state-scene", observed_at=time.time(),
            objects=(SceneObject("red-block", "方块", "红色", Pose("base", *[float(v) for v in cube])),),
            targets=(SceneObject("blue-box", "盒子", "蓝色", Pose("base", *[float(v) for v in target]), .08),),
            observation_source="mujoco_sim_state",
        )


def run_physics_pick_place(start: tuple[float, float] = (0.30, 0.05), observation_mode: str = "state") -> dict:
    """Execute the fixed red-cube to blue-bin scenario and return evidence."""
    if observation_mode not in {"state", "camera"}:
        raise ValueError("observation_mode 必须是 state 或 camera")
    robot = MujocoRobot()
    # The free cube starts on the table; only the mocap gripper is commanded.
    robot.data.qpos[0:3] = [start[0], start[1], .07]
    robot.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    robot.data.qvel[:] = 0.0
    robot.mujoco.mj_forward(robot.model, robot.data)
    observer = MujocoStateObserver(robot) if observation_mode == "state" else MujocoCameraObserver(robot)
    observed_scene = observer.observe()
    source_pose = next(o.pose for o in observed_scene.objects if o.object_id == "red-block")
    target_pose = next(o.pose for o in observed_scene.targets if o.object_id == "blue-box")
    robot.move_to(Pose("base", source_pose.x, source_pose.y, .16))
    robot.set_gripper(.10)
    robot.move_to(Pose("base", source_pose.x, source_pose.y, .13))
    robot.set_gripper(.0)
    grasped = robot.grasp()
    if grasped:
        robot.held_object = "red-block"
    if observation_mode == "state":
        before_lift = observer.observe().objects[0].pose.z
    robot.move_to(Pose("base", source_pose.x, source_pose.y, .24))
    lifted = None
    if observation_mode == "state":
        lifted = observer.observe().objects[0].pose.z > before_lift + .03
    robot.move_to(Pose("base", target_pose.x, target_pose.y, .20))
    robot.move_to(Pose("base", target_pose.x, target_pose.y, .15))
    robot.release()
    robot.held_object = None
    robot._step(0.5)
    state = robot.get_state() if observation_mode == "state" else {"time": float(robot.data.time)}
    in_bin = robot.verify_in_bin() if observation_mode == "state" else None
    state.update({"observation_mode": observation_mode, "observation_source": observed_scene.observation_source, "grasp_contact": bool(grasped), "lifted": lifted, "in_bin": in_bin, "success": bool(grasped and lifted and in_bin) if observation_mode == "state" else False, "diagnostics": robot.diagnostics()})
    return state
