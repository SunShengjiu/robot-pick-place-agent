import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

from robot_pick_place_agent.adapters.simulation.piper import PiperMujocoRobot
from robot_pick_place_agent.adapters.simulation.piper_m2a import run_m2a
from robot_pick_place_agent.adapters.simulation.piper_model import HOME
from robot_pick_place_agent.core.models import Pose


def test_m2a_three_tcp_targets_and_evidence(tmp_path):
    result = run_m2a(tmp_path, video=False)
    assert result["passed"], result
    assert len(result["targets"]) == 3
    for target in result["targets"]:
        assert target["executed"]
        assert target["collision_precheck"]["valid"]
        assert target["position_error_m"] < 2e-4
        assert target["orientation_error_rad"] < 2e-3
    assert result["trajectory"]["samples"] > 5000
    assert result["trajectory"]["max_joint_step_rad"] < .002
    assert result["trajectory"]["max_penetration_m"] == 0
    assert result["collision_rejection"]["valid"] is False
    assert result["collision_rejection"]["reason"] == "collision"
    assert result["unreachable_rejection"]["accepted"] is False
    assert not result["physical_pick_place_verified"]
    assert not result["cap_api_used"]


def test_ik_and_collision_precheck_do_not_modify_live_state():
    robot = PiperMujocoRobot()
    before_qpos = robot.data.qpos.copy()
    before_time = robot.data.time
    target = robot.forward_pose([.15, .9, -.8, .1, .25, .1])
    solution = robot.solve_ik(target, seed=HOME)
    assert solution is not None
    assert robot.check_joint_trajectory(solution["positions_rad"])["valid"]
    np.testing.assert_array_equal(robot.data.qpos, before_qpos)
    assert robot.data.time == before_time


def test_move_to_requires_base_pose_and_rejects_unreachable_target():
    robot = PiperMujocoRobot()
    assert robot.move_to(Pose("camera", .2, .1, .3)) is False
    assert robot.last_execution["reason"] == "pose_frame_must_be_base"
    assert robot.move_to(Pose("base", 2.0, 2.0, 2.0)) is False
    assert robot.last_execution["reason"] == "ik_no_solution"
