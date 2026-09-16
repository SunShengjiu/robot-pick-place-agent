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
