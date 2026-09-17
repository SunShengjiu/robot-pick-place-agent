"""M2a commissioning evidence: TCP pose IK, trajectory and collision checks."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import shutil
import subprocess

import mujoco
import numpy as np

from robot_pick_place_agent.core.models import Pose
from .piper import PiperMujocoRobot
from .piper_model import HOME


def _pose_error(actual: Pose, target: Pose):
    position = float(np.linalg.norm(np.asarray([actual.x - target.x, actual.y - target.y, actual.z - target.z])))
    actual_rotation = PiperMujocoRobot._pose_rotation(actual)
    target_rotation = PiperMujocoRobot._pose_rotation(target)
    orientation = float(np.linalg.norm(PiperMujocoRobot._rotation_error(actual_rotation, target_rotation)))
    return position, orientation


class M2aRecorder:
    def __init__(self, robot, output: Path, video=True):
        self.output = output
        self.rows = []
        self.frames = 0
        self.renderer = None
        self.encoder = None
        if video:
            from PIL import Image, ImageDraw, ImageFont
            if shutil.which("ffmpeg") is None:
                raise RuntimeError("ffmpeg is required for M2a video recording")
            self._Image = Image
            self._ImageDraw = ImageDraw
            try:
                self._font = ImageFont.truetype("DejaVuSans.ttf", 21)
                self._large_font = ImageFont.truetype("DejaVuSans.ttf", 28)
            except OSError:
                self._font = self._large_font = ImageFont.load_default()
            self.renderer = mujoco.Renderer(robot.model, height=960, width=1280)
            self.encoder = subprocess.Popen([
                "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                "-s", "1280x960", "-r", "25", "-i", "-", "-an", "-c:v", "libx264", "-crf", "19",
                "-pix_fmt", "yuv420p", str(output / "piper_m2a_continuous.mp4")], stdin=subprocess.PIPE)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [.12, 0, .25]
        self.camera.distance = 1.25
        self.camera.azimuth = 135
        self.camera.elevation = -22
        self.options = mujoco.MjvOption()
        self.options.geomgroup[3] = 0
        self.options.sitegroup[2] = 0

    def __call__(self, robot):
        data = robot.data
        row = {"time_s": float(data.time), "phase": robot.phase}
        for i, name in enumerate(robot.arm_joint_names, 1):
            row[f"joint{i}_position_rad"] = float(data.qpos[robot._qadr[i - 1]])
            row[f"joint{i}_velocity_radps"] = float(data.qvel[robot._dadr[i - 1]])
            row[f"joint{i}_target_rad"] = float(robot.target[i - 1])
        row.update({f"tcp_{axis}_m": float(value) for axis, value in zip("xyz", data.site("tcp").xpos)})
        row["contact_count"] = data.ncon
        row["max_penetration_m"] = float(max([0, *[-contact.dist for contact in data.contact]]))
        self.rows.append(row)
        if self.renderer and len(self.rows) % 40 == 1:
            self.renderer.update_scene(data, self.camera, self.options)
            frame = self._Image.fromarray(self.renderer.render())
            draw = self._ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 1280, 105), fill=(26, 29, 31))
            draw.text((24, 10), "PiPER | M2a TCP IK and trajectory | MuJoCo state", fill="white", font=self._large_font)
            draw.text((24, 53), f"t = {row['time_s']:6.2f} s    {robot.phase}", fill=(190, 220, 230), font=self._font)
            q = "  ".join(f"J{i}: {row[f'joint{i}_position_rad']:+.3f}" for i in range(1, 7))
            draw.rectangle((0, 902, 1280, 960), fill=(26, 29, 31))
            draw.text((24, 918), q + " rad", fill="white", font=self._font)
            self.encoder.stdin.write(np.asarray(frame).tobytes())
            self.frames += 1

    def close(self):
        if self.renderer:
            self.renderer.close()
        if self.encoder:
            self.encoder.stdin.close()
            if self.encoder.wait() != 0:
                raise RuntimeError("ffmpeg failed to encode M2a video")
        if self.rows:
            with (self.output / "tcp_joint_trace.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=self.rows[0].keys())
                writer.writeheader()
                writer.writerows(self.rows)


def run_m2a(output: Path, video=True):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    robot = PiperMujocoRobot()
    recorder = M2aRecorder(robot, output, video)
    robot.on_step = recorder
    # These are generated from known joint poses, then consumed only as TCP
    # targets. They exercise the inverse kinematics path rather than teleporting qpos.
    target_joints = [
        np.asarray([.15, .9, -.8, .1, .25, .1]),
        np.asarray([-.2, 1.0, -.9, -.1, .4, -.15]),
        np.asarray([.1, .75, -.6, .2, .2, .2]),
    ]
    targets = [robot.forward_pose(q) for q in target_joints]
    target_records = []
    error = None
    try:
        robot.phase = "home_hold"
        robot.hold(.4)
        for index, target in enumerate(targets, 1):
            planned = robot.solve_ik(target, seed=robot.data.qpos[robot._qadr[:6]])
            collision = robot.check_joint_trajectory(planned["positions_rad"] if planned else [0] * 6,
                                                      samples=121) if planned else {"valid": False, "reason": "ik_no_solution"}
            robot.phase = f"pose{index}_move"
            passed = robot.move_to(target)
            actual = robot.get_state()["pose"]
            position_error, orientation_error = _pose_error(actual, target)
            target_records.append({"index": index, "target_pose": {"frame_id": target.frame_id, "x": target.x, "y": target.y, "z": target.z,
                                                                     "qx": target.qx, "qy": target.qy, "qz": target.qz, "qw": target.qw},
                                   "ik_solution": planned and {"positions_rad": planned["positions_rad"].tolist(), "iterations": planned["iterations"],
                                                                "position_error_m": planned["position_error_m"], "orientation_error_rad": planned["orientation_error_rad"]},
                                   "collision_precheck": {"valid": collision["valid"], "reason": collision["reason"],
                                                           "samples_checked": collision.get("samples_checked", 0),
                                                           "max_penetration_m": collision.get("max_penetration_m", collision.get("first_collision", {}).get("max_penetration_m", 0.0))},
                                   "executed": passed, "actual_tcp_pose": {"frame_id": actual.frame_id, "x": actual.x, "y": actual.y, "z": actual.z,
                                                                              "qx": actual.qx, "qy": actual.qy, "qz": actual.qz, "qw": actual.qw},
                                   "position_error_m": position_error, "orientation_error_rad": orientation_error,
                                   "execution": dict(robot.last_execution)})
            if not passed:
                raise RuntimeError(f"M2a target {index} failed: {target_records[-1]}")
            robot.phase = f"pose{index}_hold"
            if not robot.hold(.4):
                raise RuntimeError(f"M2a target {index} hold cancelled")
    except Exception as exc:
        error = str(exc)
    finally:
        recorder.close()
    invalid = robot.check_joint_trajectory([.45, 3.1, -1.9, 0.0, .3, -.75], samples=61)
    unreachable = robot.move_to(Pose("base", 2.0, 2.0, 2.0))
    rows = recorder.rows
    max_step = max((abs(float(rows[i][f"joint{j}_position_rad"]) - float(rows[i - 1][f"joint{j}_position_rad"]))
                    for i in range(1, len(rows)) for j in range(1, 7)), default=0.0)
    result = {"milestone": "M2a", "passed": error is None and len(target_records) == 3 and all(record["executed"] for record in target_records),
              "error": error, "scope": "tcp_ik_trajectory_and_collision_checks_only",
              "observation_source": "mujoco_sim_state", "physical_pick_place_verified": False, "cap_api_used": False,
              "tcp_definition": {"parent": "link6", "position_m": [0.0, 0.0, .125], "orientation": "link6 frame", "calibrated": False},
              "targets": target_records, "collision_rejection": {"valid": invalid["valid"], "reason": invalid["reason"],
                                                                       "samples_checked": invalid.get("samples_checked", 0),
                                                                       "first_collision": invalid.get("first_collision")},
              "unreachable_rejection": {"accepted": unreachable, "execution": dict(robot.last_execution)},
              "trajectory": {"samples": len(rows), "video_frames": recorder.frames, "max_joint_step_rad": max_step,
                              "max_penetration_m": max((float(row["max_penetration_m"]) for row in rows), default=0.0),
                              "max_contact_count": max((int(row["contact_count"]) for row in rows), default=0)},
              "mujoco_version": mujoco.__version__}
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "adapted_model.xml").write_text(robot.xml)
    return result


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/piper_m2a"))
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    result = run_m2a(args.output, video=not args.no_video)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
