"""M1 commissioning and evidence recording for the full articulated PiPER."""
from __future__ import annotations

import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess

import mujoco
import numpy as np

from .piper import PiperMujocoRobot
from .piper_model import ASSETS, FORCE_LIMITS, HOME, JOINT_NAMES, KD, KP, TCP_OFFSET


def record_sample(robot):
    m, d = robot.model, robot.data
    row = {"time_s": float(d.time), "phase": robot.phase}
    for name in JOINT_NAMES:
        joint = m.joint(name)
        index = JOINT_NAMES.index(name)
        row[f"{name}_position"] = float(d.qpos[joint.qposadr[0]])
        row[f"{name}_velocity"] = float(d.qvel[joint.dofadr[0]])
        row[f"{name}_target"] = float(robot.target[index])
        row[f"{name}_effort"] = float(d.actuator(name + "_motor").force[0])
    row["opening_m"] = row["joint7_position"] - row["joint8_position"]
    row.update({f"tcp_{axis}_m": float(v) for axis, v in zip("xyz", d.site("tcp").xpos)})
    row["contact_count"] = d.ncon
    row["max_penetration_m"] = float(max([0, *[-c.dist for c in d.contact]]))
    force = np.zeros(6)
    table_force = 0.0
    for i, contact in enumerate(d.contact):
        if m.geom("table").id in (contact.geom1, contact.geom2):
            mujoco.mj_contactForce(m, d, i, force)
            table_force += float(force[0])
    row["table_contact_force_N"] = table_force
    row["base_displacement_m"] = float(np.linalg.norm(d.body("base_link").xpos))
    return row


class EvidenceRecorder:
    def __init__(self, robot, output, video):
        self.output = output
        self.rows = []
        self.frame_count = 0
        self.renderer = None
        self.encoder = None
        self.font = None
        self.large_font = None
        self.snapshots = set()
        if video:
            from PIL import ImageFont
            if shutil.which("ffmpeg") is None:
                raise RuntimeError("ffmpeg is required for M1 video recording")
            self.renderer = mujoco.Renderer(robot.model, height=960, width=1280)
            try:
                self.font = ImageFont.truetype("DejaVuSans.ttf", 21)
                self.large_font = ImageFont.truetype("DejaVuSans.ttf", 28)
            except OSError:
                self.font = self.large_font = ImageFont.load_default()
            self.encoder = subprocess.Popen([
                "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                "-s", "1280x960", "-r", "25", "-i", "-", "-an", "-c:v", "libx264",
                "-crf", "19", "-pix_fmt", "yuv420p", str(output / "piper_m1_continuous.mp4")], stdin=subprocess.PIPE)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [.12, 0, .25]
        self.camera.distance = 1.25
        self.camera.azimuth = 135
        self.camera.elevation = -22
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0
        self.options.sitegroup[2] = 0

    def __call__(self, robot):
        row = record_sample(robot)
        self.rows.append(row)
        if self.renderer and len(self.rows) % 40 == 1:
            from PIL import Image, ImageDraw
            self.renderer.update_scene(robot.data, self.camera, self.options)
            frame = Image.fromarray(self.renderer.render())
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 1280, 106), fill=(26, 29, 31))
            draw.text((24, 10), "PiPER | M1 joint commissioning | MuJoCo state", fill="white", font=self.large_font)
            draw.text((24, 53), f"t = {row['time_s']:6.2f} s    {robot.phase}    opening = {row['opening_m'] * 1000:5.1f} mm", fill=(190, 220, 230), font=self.font)
            q = "  ".join(f"J{i}: {row[f'joint{i}_position']:+.3f}" for i in range(1, 7))
            draw.rectangle((0, 902, 1280, 960), fill=(26, 29, 31))
            draw.text((24, 918), q + " rad", fill="white", font=self.font)
            self.encoder.stdin.write(np.asarray(frame).tobytes())
            self.frame_count += 1
            if robot.phase in ("home_hold", "specified_pose_hold", "gripper_closed_hold", "gripper_open_hold") and robot.phase not in self.snapshots:
                frame.save(self.output / f"{robot.phase}.png")
                self.snapshots.add(robot.phase)

    def close(self):
        if self.renderer:
            self.renderer.close()
        if self.encoder:
            self.encoder.stdin.close()
            if self.encoder.wait() != 0:
                raise RuntimeError("ffmpeg failed to encode PiPER video")
        if self.rows:
            with (self.output / "joint_trace.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=self.rows[0].keys())
                writer.writeheader()
                writer.writerows(self.rows)


def run_m1(output: Path, video=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    robot = PiperMujocoRobot()
    recorder = EvidenceRecorder(robot, output, video)
    robot.on_step = recorder
    motions = []

    def motion(name, fn, target):
        robot.phase = name
        begin = float(robot.data.time)
        passed = fn(target)
        motions.append({"phase": name, "start_s": begin, "end_s": float(robot.data.time),
                        "command": list(target) if isinstance(target, (list, tuple)) else target,
                        "passed": passed, "execution": dict(robot.last_execution)})
        if not passed:
            raise RuntimeError(f"M1 motion failed: {motions[-1]}")

    error = None
    try:
        robot.phase = "home_hold"
        robot.hold(.8)
        for i in range(6):
            target = list(HOME)
            target[i] += .25
            motion(f"joint{i + 1}_positive", robot.move_joints, target)
            robot.phase = f"joint{i + 1}_hold"
            robot.hold(.3)
            motion(f"joint{i + 1}_return", robot.move_joints, HOME)
        motion("specified_pose", robot.move_joints, (.35, 1.05, -1.05, .25, .5, -.35))
        robot.phase = "specified_pose_hold"
        robot.hold(.8)
        motion("return_home", robot.move_joints, HOME)
        motion("gripper_close", robot.set_gripper, 0.0)
        robot.phase = "gripper_closed_hold"
        robot.hold(.6)
        motion("gripper_open", robot.set_gripper, .07)
        robot.phase = "gripper_open_hold"
        robot.hold(.6)
        motion("gripper_half", robot.set_gripper, .035)
        motion("gripper_reopen", robot.set_gripper, .07)
        robot.phase = "final_hold"
        robot.hold(.5)
    except Exception as exc:
        error = str(exc)
    finally:
        recorder.close()
    rows = recorder.rows
    rejected = []
    for label, command in (("joint_outside_limit", [-3, *HOME[1:]]), ("joint_nan", [float("nan"), *HOME[1:]])):
        before = float(robot.data.time)
        accepted = robot.move_joints(command)
        rejected.append({"case": label, "rejected_without_motion": not accepted and robot.data.time == before, "result": dict(robot.last_execution)})
    before = float(robot.data.time)
    accepted = robot.set_gripper(.08)
    rejected.append({"case": "unsupported_80mm_opening", "rejected_without_motion": not accepted and robot.data.time == before, "result": dict(robot.last_execution)})
    checks = {
        "all_18_commands_completed": len(motions) == 18 and all(m["passed"] for m in motions),
        "six_joints_individually_moved": all(any(row["phase"] == f"joint{i + 1}_hold" and abs(row[f"joint{i + 1}_position"] - HOME[i]) > .24 for row in rows) for i in range(6)),
        "base_fixed": all(r["base_displacement_m"] < 1e-12 for r in rows),
        "joint_limits_respected": all(robot.limits[i, 0] - 1e-5 <= r[f"{name}_position"] <= robot.limits[i, 1] + 1e-5 for r in rows for i, name in enumerate(JOINT_NAMES)),
        "no_table_contact": all(r["table_contact_force_N"] < .01 for r in rows),
        "no_penetration_above_1mm": all(r["max_penetration_m"] < .001 for r in rows),
        "invalid_commands_rejected": all(r["rejected_without_motion"] for r in rejected),
        "no_mocap_freejoint_or_weld": robot.model.nmocap == 0 and robot.model.neq == 0 and robot.model.nq == 8,
        "no_solver_warnings": all(w.number == 0 for w in robot.data.warning),
    }
    source = json.loads((ASSETS / "source.json").read_text())
    result = {"milestone": "M1", "passed": error is None and all(checks.values()), "error": error,
              "scope": "full_piper_joint_commissioning_only", "observation_source": "mujoco_sim_state",
              "physical_pick_place_verified": False, "cap_api_used": False, "checks": checks,
              "duration_s": float(robot.data.time), "video_frames": recorder.frame_count,
              "source_commit": source["commit"], "mujoco_version": mujoco.__version__, "python_version": platform.python_version(),
              "max_penetration_m": max((r["max_penetration_m"] for r in rows), default=0),
              "motions": motions, "rejections": rejected,
              "final_pose": asdict(robot.get_state()["pose"])}
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "adapted_model.xml").write_text(robot.xml)
    audit = {"source": source, "adapted_xml_sha256": hashlib.sha256(robot.xml.encode()).hexdigest(),
             "base_frame": "base = official base_link = world; meters, radians, Pose quaternion xyzw",
             "tcp": {"parent": "link6", "position_m": TCP_OFFSET, "quaternion_xyzw": [0, 0, 0, 1], "calibrated": False},
             "gripper_mapping": "joint7 = opening_m / 2; joint8 = -opening_m / 2; nominal 0..0.07 m",
             "total_urdf_mass_kg": float(robot.model.body_mass.sum()),
             "joints": [{"name": n, "qpos_index": int(robot.model.joint(n).qposadr[0]),
                         "dof_index": int(robot.model.joint(n).dofadr[0]),
                         "axis_local": robot.model.joint(n).axis.tolist(), "range": robot.limits[i].tolist(),
                         "actuator": n + "_motor", "sim_kp": KP[i], "sim_kd": KD[i], "sim_effort_limit": FORCE_LIMITS[i]}
                        for i, n in enumerate(JOINT_NAMES)],
             "body_inertias": [{"name": robot.model.body(i).name, "mass_kg": float(robot.model.body_mass[i]),
                                 "principal_inertia_kgm2": robot.model.body_inertia[i].tolist()}
                                for i in range(1, robot.model.nbody)],
             "collision": "official STL convex hull per link; visual mesh separate; base_link/link1 bearing pair excluded; MuJoCo default parent filtering",
             "hardware_parameters_verified": False}
    (output / "model_audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    return result
