"""Native MuJoCo PiPER joint backend. Cartesian motion is an M2 capability."""
from __future__ import annotations

import math
import threading

import mujoco
import numpy as np

from robot_pick_place_agent.core.models import Pose
from .piper_model import FORCE_LIMITS, HOME, JOINT_NAMES, KD, KP, build_model_xml


class PiperMujocoRobot:
    arm_joint_names = JOINT_NAMES[:6]
    max_joint_speed_radps = 0.5
    max_joint_acceleration_radps2 = 1.0
    max_opening_speed_mps = 0.04
    gripper_opening_limits_m = (0.0, 0.07)

    def __init__(self):
        self.xml = build_model_xml()
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self._qadr = np.array([self.model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._dadr = np.array([self.model.joint(n).dofadr[0] for n in JOINT_NAMES])
        self._aids = np.array([self.model.actuator(n + "_motor").id for n in JOINT_NAMES])
        self.limits = np.array([self.model.joint(n).range for n in JOINT_NAMES])
        # Initial state only. All subsequent motion is integrated actuator motion.
        self.data.qpos[self._qadr] = [*HOME, .035, -.035]
        self.target = self.data.qpos[self._qadr].copy()
        self.target_velocity = np.zeros(8)
        self.on_step = None
        self.phase = "initial"
        self.last_execution = {"status": "idle"}
        self._cancel_requested = threading.Event()
        mujoco.mj_forward(self.model, self.data)

    def _step(self):
        q = self.data.qpos[self._qadr]
        velocity = self.data.qvel[self._dadr]
        # Bias compensation is explicit simulation feedback, never a real motor spec.
        effort = np.array(KP) * (self.target - q) + np.array(KD) * (self.target_velocity - velocity)
        effort += self.data.qfrc_bias[self._dadr]
        self.data.ctrl[self._aids] = np.clip(effort, -np.array(FORCE_LIMITS), FORCE_LIMITS)
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        if self.on_step:
            self.on_step(self)

    def hold(self, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("Hold duration must be finite and nonnegative")
        self.target_velocity[:] = 0
        for _ in range(math.ceil(seconds / self.model.opt.timestep)):
            if self._cancel_requested.is_set():
                self._finish_cancelled()
                return False
            self._step()
            if self._cancel_requested.is_set():
                self._finish_cancelled()
                return False
        return True

    def _finish_cancelled(self):
        """Freeze the trajectory at the integrated state and retain cancellation evidence."""
        self.target = self.data.qpos[self._qadr].copy()
        self.target_velocity[:] = 0
        self.last_execution = {"status": "cancelled", "reason": "motion_cancelled",
                               "time_s": float(self.data.time),
                               "joint_positions_rad": self.data.qpos[self._qadr[:6]].tolist()}

    def _execute(self, target, minimum_duration=0.8):
        self._cancel_requested.clear()
        start = self.data.qpos[self._qadr].copy()
        delta = target - start
        duration = max(minimum_duration, 1.875 * np.max(np.abs(delta[:6])) / self.max_joint_speed_radps,
                       math.sqrt(5.774 * np.max(np.abs(delta[:6])) / self.max_joint_acceleration_radps2),
                       1.875 * abs(delta[6] - delta[7]) / self.max_opening_speed_mps)
        steps = math.ceil(duration / self.model.opt.timestep)
        duration = steps * self.model.opt.timestep
        peak_penetration = 0.0
        for step in range(1, steps + 1):
            if self._cancel_requested.is_set():
                self._finish_cancelled()
                return False
            t = step / steps
            self.target = start + delta * (10 * t**3 - 15 * t**4 + 6 * t**5)
            self.target_velocity = delta * (30 * t**2 - 60 * t**3 + 30 * t**4) / duration
            self._step()
            if self._cancel_requested.is_set():
                self._finish_cancelled()
                return False
            peak_penetration = max(peak_penetration, max((-c.dist for c in self.data.contact), default=0))
            if not np.isfinite(self.data.qpos).all() or peak_penetration > .002:
                self.cancel()
                self.last_execution = {"status": "failed", "reason": "collision_or_nonfinite_state", "peak_penetration_m": float(peak_penetration)}
                return False
        self.target = target.copy()
        if not self.hold(.35):
            return False
        peak_penetration = max(peak_penetration, max((-c.dist for c in self.data.contact), default=0))
        error = np.abs(self.data.qpos[self._qadr] - target)
        reached = bool(np.max(error[:6]) < .01 and np.max(error[6:]) < .0005
                       and np.max(np.abs(self.data.qvel[self._dadr])) < .01 and peak_penetration <= .002
                       and all(w.number == 0 for w in self.data.warning))
        self.last_execution = {"status": "succeeded" if reached else "failed", "reason": "target_reached" if reached else "tracking_error",
                               "arm_error_rad": float(np.max(error[:6])), "finger_error_m": float(np.max(error[6:])),
                               "duration_s": duration + .35, "peak_penetration_m": float(peak_penetration)}
        return reached

    def move_joints(self, positions_rad) -> bool:
        try:
            positions = np.asarray(positions_rad, dtype=float)
        except (ValueError, TypeError):
            positions = np.array([])
        if positions.shape != (6,) or not np.isfinite(positions).all():
            self.last_execution = {"status": "rejected", "reason": "expected_six_finite_radians"}
            return False
        if np.any(positions < self.limits[:6, 0]) or np.any(positions > self.limits[:6, 1]):
            self.last_execution = {"status": "rejected", "reason": "joint_limit"}
            return False
        target = self.target.copy()
        target[:6] = positions
        return self._execute(target)

    def set_gripper(self, opening_m: float) -> bool:
        low, high = self.gripper_opening_limits_m
        if not isinstance(opening_m, (int, float)) or not math.isfinite(opening_m) or not low <= opening_m <= high:
            self.last_execution = {"status": "rejected", "reason": f"opening_m_must_be_in_{low}_to_{high}"}
            return False
        target = self.target.copy()
        target[6:] = [opening_m / 2, -opening_m / 2]
        return self._execute(target)

    def move_to(self, pose: Pose) -> bool:
        self.last_execution = {"status": "rejected", "reason": "cartesian_ik_not_implemented_m1"}
        return False

    def cancel(self) -> None:
        """Request cancellation; the active loop freezes at its next step boundary."""
        self._cancel_requested.set()
        self.last_execution = {"status": "cancelled", "reason": "motion_cancel_requested",
                               "time_s": float(self.data.time)}

    def get_state(self) -> dict:
        quaternion = np.zeros(4)
        mujoco.mju_mat2Quat(quaternion, self.data.site("tcp").xmat)
        xyz = self.data.site("tcp").xpos
        return {"observation_source": "mujoco_sim_state", "execution_mode": "piper_mujoco",
                "time": float(self.data.time), "joint_names": list(self.arm_joint_names),
                "joint_positions_rad": self.data.qpos[self._qadr[:6]].tolist(),
                "gripper_opening_m": float(self.data.qpos[self._qadr[6]] - self.data.qpos[self._qadr[7]]),
                "gripper_opening_limits_m": list(self.gripper_opening_limits_m),
                "pose": Pose("base", *map(float, xyz), *map(float, quaternion[1:]), float(quaternion[0])),
                "last_execution": dict(self.last_execution),
                "capabilities": {"joint_motion": True, "gripper": True, "cartesian_motion": False, "physical_pick_place": False}}
