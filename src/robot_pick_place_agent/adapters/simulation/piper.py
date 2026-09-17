"""Native MuJoCo PiPER joint backend. Cartesian motion is an M2 capability."""
from __future__ import annotations

import math
import threading

import mujoco
import numpy as np

from robot_pick_place_agent.core.models import Pose
from .piper_model import FORCE_LIMITS, HOME, JOINT_NAMES, KD, KP, TCP_OFFSET, build_model_xml


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

    @staticmethod
    def _pose_rotation(pose: Pose):
        quaternion = np.asarray([pose.qw, pose.qx, pose.qy, pose.qz], dtype=float)
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm < 1e-12:
            return None
        quaternion /= norm
        matrix = np.zeros(9)
        mujoco.mju_quat2Mat(matrix, quaternion)
        return matrix.reshape(3, 3)

    @staticmethod
    def _rotation_error(current, target):
        # Geometric orientation residual used by the damped least-squares IK.
        return 0.5 * sum((np.cross(current[:, i], target[:, i]) for i in range(3)), np.zeros(3))

    def forward_pose(self, positions_rad) -> Pose | None:
        """Return the actual TCP pose for six joint positions without executing them."""
        try:
            positions = np.asarray(positions_rad, dtype=float)
        except (TypeError, ValueError):
            return None
        if positions.shape != (6,) or not np.isfinite(positions).all():
            return None
        if np.any(positions < self.limits[:6, 0]) or np.any(positions > self.limits[:6, 1]):
            return None
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        scratch.qpos[self._qadr[:6]] = positions
        mujoco.mj_forward(self.model, scratch)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, scratch.site("tcp").xmat)
        xyz = scratch.site("tcp").xpos
        return Pose("base", *map(float, xyz), float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0]))

    def solve_ik(self, pose: Pose, seed=None, position_tolerance_m=2e-4,
                 orientation_tolerance_rad=2e-3, max_iterations=300):
        """Solve TCP position and orientation with bounded damped least squares."""
        if pose.frame_id != "base" or not all(math.isfinite(float(v)) for v in (pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw)):
            return None
        target_rotation = self._pose_rotation(pose)
        if target_rotation is None:
            return None
        if seed is None:
            q = self.data.qpos[self._qadr[:6]].copy()
        else:
            try:
                q = np.asarray(seed, dtype=float).copy()
            except (TypeError, ValueError):
                return None
            if q.shape != (6,) or not np.isfinite(q).all():
                return None
        q = np.clip(q, self.limits[:6, 0], self.limits[:6, 1])
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        site_id = self.model.site("tcp").id
        damping = 1e-4
        best = None
        for iteration in range(max_iterations):
            scratch.qpos[self._qadr[:6]] = q
            mujoco.mj_forward(self.model, scratch)
            position_error = np.asarray([pose.x, pose.y, pose.z]) - scratch.site("tcp").xpos
            current_rotation = scratch.site("tcp").xmat.reshape(3, 3)
            orientation_error = self._rotation_error(current_rotation, target_rotation)
            residual = np.concatenate((position_error, orientation_error))
            score = float(np.linalg.norm(position_error) + np.linalg.norm(orientation_error))
            if best is None or score < best[0]:
                best = (score, q.copy(), float(np.linalg.norm(position_error)), float(np.linalg.norm(orientation_error)), iteration + 1)
            if np.linalg.norm(position_error) <= position_tolerance_m and np.linalg.norm(orientation_error) <= orientation_tolerance_rad:
                return {"positions_rad": q.copy(), "iterations": iteration + 1,
                        "position_error_m": float(np.linalg.norm(position_error)),
                        "orientation_error_rad": float(np.linalg.norm(orientation_error))}
            mujoco.mj_jacSite(self.model, scratch, jacp, jacr, site_id)
            jacobian = np.vstack((jacp[:, self._dadr[:6]], jacr[:, self._dadr[:6]]))
            normal = jacobian.T @ jacobian + damping * np.eye(6)
            try:
                step = np.linalg.solve(normal, jacobian.T @ residual)
            except np.linalg.LinAlgError:
                return None
            step = np.clip(step, -0.08, 0.08)
            q = np.clip(q + step, self.limits[:6, 0], self.limits[:6, 1])
        if best and best[2] <= position_tolerance_m and best[3] <= orientation_tolerance_rad:
            return {"positions_rad": best[1], "iterations": best[4],
                    "position_error_m": best[2], "orientation_error_rad": best[3]}
        return None

    def _collision_report(self, q_arm, opening_m=None):
        q_arm = np.asarray(q_arm, dtype=float)
        scratch = mujoco.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        scratch.qpos[self._qadr[:6]] = q_arm
        if opening_m is not None:
            scratch.qpos[self._qadr[6:]] = [opening_m / 2, -opening_m / 2]
        mujoco.mj_forward(self.model, scratch)
        contacts = []
        for contact in scratch.contact:
            names = {self.model.geom(contact.geom1).name, self.model.geom(contact.geom2).name}
            # The two official finger meshes have a sub-micron CAD overlap at
            # the mechanical closed stop. It is intentional end-stop contact,
            # not a swept-path collision.
            if opening_m is not None and opening_m <= self.gripper_opening_limits_m[0] + 1e-7 and names == {"link7_collision", "link8_collision"}:
                continue
            if contact.dist < -1e-7:
                contacts.append({"geom1": self.model.geom(contact.geom1).name,
                                 "geom2": self.model.geom(contact.geom2).name,
                                 "penetration_m": float(-contact.dist)})
        return {"valid": not contacts, "contacts": contacts,
                "max_penetration_m": max((item["penetration_m"] for item in contacts), default=0.0)}

    def check_joint_trajectory(self, positions_rad, opening_m=None, samples=121):
        """Check a continuous quintic joint path before any actuator command."""
        try:
            target_arm = np.asarray(positions_rad, dtype=float)
        except (TypeError, ValueError):
            return {"valid": False, "reason": "expected_six_finite_radians"}
        if target_arm.shape != (6,) or not np.isfinite(target_arm).all():
            return {"valid": False, "reason": "expected_six_finite_radians"}
        if np.any(target_arm < self.limits[:6, 0]) or np.any(target_arm > self.limits[:6, 1]):
            return {"valid": False, "reason": "joint_limit"}
        if opening_m is not None and (not math.isfinite(float(opening_m)) or not self.gripper_opening_limits_m[0] <= opening_m <= self.gripper_opening_limits_m[1]):
            return {"valid": False, "reason": "gripper_limit"}
        start = self.data.qpos[self._qadr[:6]].copy()
        delta = target_arm - start
        duration = max(0.8, 1.875 * np.max(np.abs(delta)) / self.max_joint_speed_radps,
                       math.sqrt(5.774 * np.max(np.abs(delta)) / self.max_joint_acceleration_radps2))
        reports = []
        for t in np.linspace(0, 1, max(2, int(samples))):
            ratio = 10 * t**3 - 15 * t**4 + 6 * t**5
            opening = None if opening_m is None else float(self.data.qpos[self._qadr[6]] - self.data.qpos[self._qadr[7]] + (opening_m - (self.data.qpos[self._qadr[6]] - self.data.qpos[self._qadr[7]])) * ratio)
            report = self._collision_report(start + delta * ratio, opening)
            reports.append({"t": float(t), **report})
            if not report["valid"]:
                return {"valid": False, "reason": "collision", "duration_s": duration,
                        "samples_checked": len(reports), "first_collision": reports[-1], "reports": reports}
        return {"valid": True, "reason": "clear", "duration_s": duration,
                "samples_checked": len(reports), "max_penetration_m": max((r["max_penetration_m"] for r in reports), default=0.0),
                "reports": reports}

    def _execute(self, target, minimum_duration=0.8):
        self._cancel_requested.clear()
        start = self.data.qpos[self._qadr].copy()
        delta = target - start
        collision = self.check_joint_trajectory(target[:6],
                                                target[6] - target[7],
                                                samples=min(121, max(2, math.ceil(np.max(np.abs(delta[:6])) * 80))))
        if not collision["valid"]:
            self.last_execution = {"status": "failed", "reason": "collision_precheck", "collision": collision}
            return False
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
        if pose.frame_id != "base":
            self.last_execution = {"status": "rejected", "reason": "pose_frame_must_be_base"}
            return False
        solution = self.solve_ik(pose)
        if solution is None:
            self.last_execution = {"status": "failed", "reason": "ik_no_solution"}
            return False
        target = self.target.copy()
        target[:6] = solution["positions_rad"]
        reached = self._execute(target)
        self.last_execution.update({"ik_iterations": solution["iterations"],
                                    "ik_position_error_m": solution["position_error_m"],
                                    "ik_orientation_error_rad": solution["orientation_error_rad"],
                                    "target_pose": {"frame_id": pose.frame_id, "x": pose.x, "y": pose.y, "z": pose.z,
                                                    "qx": pose.qx, "qy": pose.qy, "qz": pose.qz, "qw": pose.qw}})
        return reached

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
                "tcp_offset_m": list(TCP_OFFSET),
                "capabilities": {"joint_motion": True, "gripper": True, "cartesian_motion": True, "physical_pick_place": False}}
