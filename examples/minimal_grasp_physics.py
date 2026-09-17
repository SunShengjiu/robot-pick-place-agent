"""Standalone dynamic parallel-jaw grasp acceptance experiment.

Run with MUJOCO_GL=egl python3 examples/minimal_grasp_physics.py.
Only the control target is kinematic. No equality references the cube.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np


XML = """
<mujoco model="minimal_dynamic_grasp">
  <compiler autolimits="true"/>
  <option timestep="0.0005" integrator="implicitfast" gravity="0 0 -9.81" noslip_iterations="10"/>
  <visual><global offwidth="960" offheight="720"/></visual>
  <default>
    <geom condim="3" friction="0.6 0.005 0.0001"
          solref="0.002 1" solimp="0.95 0.99 0.001"/>
    <joint damping="2" armature="0.002"/>
  </default>
  <worldbody>
    <light pos="0 -0.3 1" dir="0 0 -1"/>
    <geom name="table" type="plane" size="0.5 0.4 0.01" rgba="0.65 0.68 0.7 1"/>
    <body name="cube" pos="0 0 0.03">
      <freejoint/>
      <geom name="cube_geom" type="box" size="0.03 0.03 0.03" mass="0.08" rgba="0.85 0.15 0.12 1"/>
    </body>
    <body name="target" mocap="true" pos="0 0 0.10"/>
    <body name="gripper" pos="0 0 0.10">
      <freejoint/>
      <geom name="palm" type="box" size="0.045 0.08 0.012" mass="0.25" rgba="0.25 0.3 0.34 1"/>
      <body name="left_finger" pos="0 0.055 -0.07">
        <joint name="left_slide" type="slide" axis="0 -1 0" range="0 0.025"/>
        <geom name="left_pad" type="box" size="0.024 0.01 0.022" mass="0.04" friction="0.8 0.005 0.0001" rgba="0.15 0.5 0.7 1"/>
      </body>
      <body name="right_finger" pos="0 -0.055 -0.07">
        <joint name="right_slide" type="slide" axis="0 1 0" range="0 0.025"/>
        <geom name="right_pad" type="box" size="0.024 0.01 0.022" mass="0.04" friction="0.8 0.005 0.0001" rgba="0.15 0.5 0.7 1"/>
      </body>
    </body>
  </worldbody>
  <equality><weld body1="target" body2="gripper" solref="0.005 1"/></equality>
  <actuator>
    <position name="left_motor" joint="left_slide" kp="800" kv="8" ctrlrange="0 0.025" forcerange="-5 5"/>
    <position name="right_motor" joint="right_slide" kp="800" kv="8" ctrlrange="0 0.025" forcerange="-5 5"/>
  </actuator>
</mujoco>
"""

# Smooth starts/stops make acceleration, rather than target jumps, drive motion.
PHASES = [
    ("settle", 0.6, (0, 0, .10), 0),
    ("close", 1.0, (0, 0, .10), .018),
    ("grip_hold", .5, (0, 0, .10), .018),
    ("lift", 1.5, (0, 0, .18), .018),
    ("lift_hold", .6, (0, 0, .18), .018),
    ("translate", 1.8, (.12, 0, .18), .018),
    ("transport_hold", .6, (.12, 0, .18), .018),
    ("open", .8, (.12, 0, .18), 0),
    ("fall_and_settle", 1.0, (.12, 0, .18), 0),
]


def sample(model, data, phase):
    cube = data.body("cube")
    base = data.body("gripper")
    relative = base.xmat.reshape(3, 3).T @ (cube.xpos - base.xpos)
    row = {"time_s": float(data.time), "phase": phase}
    for prefix, xyz in (("cube", cube.xpos), ("gripper", base.xpos), ("relative", relative)):
        row.update({f"{prefix}_{axis}_m": float(v) for axis, v in zip("xyz", xyz)})
    cube_dof = model.jnt_dofadr[model.body("cube").jntadr[0]]
    row["cube_vz_mps"] = float(data.qvel[cube_dof + 2])
    row["cube_speed_mps"] = float(np.linalg.norm(data.qvel[cube_dof:cube_dof + 3]))
    row["cube_angular_speed_radps"] = float(np.linalg.norm(data.qvel[cube_dof + 3:cube_dof + 6]))
    row["penetration_m"] = 0.0
    row["table_normal_N"] = 0.0
    row["palm_normal_N"] = 0.0
    cube_id = model.geom("cube_geom").id
    cube_bottom = cube.xpos[2] - np.abs(cube.xmat.reshape(3, 3)[2]) @ model.geom_size[cube_id]
    for side in ("left", "right"):
        row[f"{side}_normal_N"] = 0.0
        row[f"{side}_vertical_N"] = 0.0
        row[f"{side}_side_normal_min"] = 1.0
        pad = data.geom(f"{side}_pad")
        pad_bottom = pad.xpos[2] - np.abs(pad.xmat.reshape(3, 3)[2]) @ model.geom_size[pad.id]
        row[f"{side}_tip_above_cube_bottom_m"] = float(pad_bottom - cube_bottom)
        row[f"{side}_normal_vertical_N"] = 0.0
        row[f"{side}_q_m"] = float(data.joint(f"{side}_slide").qpos[0])
        row[f"{side}_actuator_N"] = float(data.actuator(f"{side}_motor").force[0])
    for index, contact in enumerate(data.contact):
        if cube_id not in (contact.geom1, contact.geom2):
            continue
        row["penetration_m"] = max(row["penetration_m"], -float(contact.dist))
        other = contact.geom2 if contact.geom1 == cube_id else contact.geom1
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, wrench)
        if other == model.geom("table").id:
            row["table_normal_N"] += float(wrench[0])
        if other == model.geom("palm").id:
            row["palm_normal_N"] += float(wrench[0])
        for side in ("left", "right"):
            if other != model.geom(f"{side}_pad").id or wrench[0] <= .01:
                continue
            frame = contact.frame.reshape(3, 3)
            force = frame.T @ wrench[:3] * (1 if contact.geom2 == cube_id else -1)
            pad = data.geom(f"{side}_pad")
            normal_local = pad.xmat.reshape(3, 3).T @ frame[0]
            row[f"{side}_normal_N"] += float(wrench[0])
            row[f"{side}_vertical_N"] += float(force[2])
            row[f"{side}_side_normal_min"] = min(row[f"{side}_side_normal_min"], abs(float(normal_local[1])))
            row[f"{side}_normal_vertical_N"] += abs(float(frame[0, 2] * wrench[0]))
    for side in ("left", "right"):
        row[f"{side}_effective"] = int(row[f"{side}_normal_N"] > .05 and row[f"{side}_side_normal_min"] > .95)
    return row


def evaluate(rows):
    def phase(name):
        return [r for r in rows if r["phase"] == name]

    initial = phase("settle")[-200:]
    z0 = float(np.mean([r["cube_z_m"] for r in initial]))
    airborne = [r for r in rows if r["phase"] in ("lift_hold", "translate", "transport_hold")]
    final = phase("fall_and_settle")[-300:]
    relative = np.array([[r[f"relative_{axis}_m"] for axis in "xyz"] for r in airborne])
    released = [r for r in rows if r["phase"] in ("open", "fall_and_settle") and not r["left_effective"] and not r["right_effective"] and r["table_normal_N"] < .01]
    freefall = [r for r in released if r["cube_vz_mps"] < -.05]
    accelerations = [(b["cube_vz_mps"] - a["cube_vz_mps"]) / (b["time_s"] - a["time_s"]) for a, b in zip(freefall, freefall[1:]) if b["time_s"] - a["time_s"] < .0011]
    timestep = rows[1]["time_s"] - rows[0]["time_s"]
    holds = {name: phase(name)[-1]["time_s"] - phase(name)[0]["time_s"] + timestep
             for name in ("lift_hold", "transport_hold")}
    checks = {
        "holds_at_least_half_second": all(duration >= .5 for duration in holds.values()),
        "initially_stationary": max(r["cube_speed_mps"] for r in initial) < .001,
        "no_closure_launch": max(r["cube_z_m"] for r in phase("close") + phase("grip_hold")) - z0 < .002,
        "sustained_lift_at_least_5cm": min(r["cube_z_m"] for r in airborne) - z0 >= .05,
        "bilateral_effective_contact_through_transport": all(r["left_effective"] and r["right_effective"] for r in airborne),
        "horizontal_transport_at_least_10cm": phase("transport_hold")[-1]["cube_x_m"] - initial[-1]["cube_x_m"] >= .10,
        "relative_position_drift_below_2mm": float(np.max(np.linalg.norm(relative - relative[0], axis=1))) < .002,
        "penetration_below_1mm": max(r["penetration_m"] for r in rows) < .001,
        "side_contact_without_tip_support": all(r[f"{s}_side_normal_min"] > .95 and r[f"{s}_tip_above_cube_bottom_m"] > .005 and r[f"{s}_normal_vertical_N"] < .01 for r in airborne for s in ("left", "right")),
        "no_table_or_palm_support_in_air": all(r["table_normal_N"] < .01 and r["palm_normal_N"] < .01 for r in airborne),
        "released_freefall": len(freefall) >= 50 and bool(accelerations) and abs(float(np.median(accelerations)) + 9.81) < .1,
        "stable_after_fall": all(abs(r["cube_z_m"] - z0) < .001 and r["cube_speed_mps"] < .001 and r["cube_angular_speed_radps"] < .01 and not r["left_effective"] and not r["right_effective"] for r in final),
    }
    return {"passed": all(checks.values()), "checks": checks,
            "minimum_sustained_lift_m": float(min(r["cube_z_m"] for r in airborne) - z0),
            "horizontal_transport_m": float(phase("transport_hold")[-1]["cube_x_m"] - initial[-1]["cube_x_m"]),
            "max_relative_drift_m": float(np.max(np.linalg.norm(relative - relative[0], axis=1))),
            "max_penetration_m": max(r["penetration_m"] for r in rows),
            "freefall_acceleration_mps2": float(np.median(accelerations)) if accelerations else None,
            "hold_durations_s": holds,
            "minimum_tip_above_cube_bottom_m": min(r[f"{s}_tip_above_cube_bottom_m"] for r in airborne for s in ("left", "right")),
            "minimum_normal_force_N": {s: min(r[f"{s}_normal_N"] for r in airborne) for s in ("left", "right")}}


def run(output: Path, video=True, xml=XML):
    output.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=720, width=960) if video else None
    encoder = None
    if video:
        encoder = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "960x720", "-r", "50", "-i", "-", "-an", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", str(output / "continuous.mp4")], stdin=subprocess.PIPE)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.05, 0, .075]
    camera.distance = .55
    camera.azimuth = 135
    camera.elevation = -22
    rows = []
    step = 0
    try:
        for name, duration, target, opening in PHASES:
            start = data.mocap_pos[0].copy()
            control = data.ctrl.copy()
            count = round(duration / model.opt.timestep)
            for i in range(1, count + 1):
                t = i / count
                ratio = t * t * t * (10 + t * (-15 + 6 * t))
                data.mocap_pos[0] = start + (np.array(target) - start) * ratio
                data.ctrl[:] = control + (opening - control) * ratio
                mujoco.mj_step(model, data)
                mujoco.mj_forward(model, data)
                rows.append(sample(model, data, name))
                if renderer and step % round(1 / (50 * model.opt.timestep)) == 0:
                    renderer.update_scene(data, camera)
                    encoder.stdin.write(renderer.render().tobytes())
                step += 1
    finally:
        if renderer:
            renderer.close()
        if encoder:
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("Video encoding failed")
    with (output / "trace.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    result = evaluate(rows)
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "model.xml").write_text(xml)
    print(json.dumps(result, indent=2))
    return result


def plot_trace(output: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with (output / "trace.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    def values(key):
        return np.array([float(row[key]) for row in rows])

    time = values("time_s")
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True, layout="constrained")
    axes[0].plot(time, values("cube_z_m") * 100, label="Cube center")
    axes[0].plot(time, (values("gripper_z_m") - .07) * 100, "--", label="Finger center")
    axes[0].set_ylabel("Height (cm)")
    for side in ("left", "right"):
        axes[1].plot(time, values(f"{side}_normal_N"), label=side, linestyle="-" if side == "left" else "--")
        axes[2].plot(time, values(f"{side}_effective"), label=side, linestyle="-" if side == "left" else "--")
    axes[1].set_ylabel("Normal force (N)")
    axes[2].set_ylabel("Effective contact")
    for axis in "xyz":
        axes[3].plot(time, values(f"relative_{axis}_m") * 1000, label=axis)
    axes[3].set_ylabel("Cube in base frame (mm)")
    axes[4].plot(time, values("cube_x_m") * 100, label="Cube x (cm)")
    axes[4].plot(time, values("penetration_m") * 1000, label="Max penetration (mm)")
    axes[4].set_xlabel("Simulation time (s)")
    elapsed = 0
    for name, duration, _, _ in PHASES:
        for ax in axes:
            ax.axvline(elapsed, color="gray", linewidth=.6, alpha=.4)
        axes[0].text(elapsed + duration / 2, 1.02, name.replace("_", "\n"),
                     transform=axes[0].get_xaxis_transform(), ha="center", va="bottom", fontsize=8)
        elapsed += duration
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend(loc="upper left", ncol=3)
    fig.savefig(output / "trace.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/minimal_grasp"))
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--plot", action="store_true", help="Generate trace.png (requires matplotlib)")
    args = parser.parse_args()
    result = run(args.output, not args.no_video)
    if args.plot:
        plot_trace(args.output)
    raise SystemExit(0 if result["passed"] else 1)
