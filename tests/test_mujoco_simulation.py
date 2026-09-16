import pytest


mujoco = pytest.importorskip("mujoco")


def test_physics_pick_place():
    from robot_pick_place_agent.adapters.simulation.mujoco import run_physics_pick_place
    result = run_physics_pick_place()
    assert result["grasp_contact"], result
    assert not result["success"]


def test_physics_pick_place_from_second_start():
    from robot_pick_place_agent.adapters.simulation.mujoco import run_physics_pick_place
    result = run_physics_pick_place((0.25, -0.10))
    assert result["grasp_contact"], result
    assert not result["success"]


def test_camera_observation_is_explicit_and_does_not_report_state_pose():
    from robot_pick_place_agent.adapters.simulation.mujoco import run_physics_pick_place
    result = run_physics_pick_place(observation_mode="camera")
    assert result["observation_source"] == "mujoco_camera_rgb"
    assert result["observation_mode"] == "camera"
    assert "cube_position" not in result
    assert result["lifted"] is None
