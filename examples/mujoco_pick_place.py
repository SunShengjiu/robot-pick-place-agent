import json
from robot_pick_place_agent.adapters.simulation.mujoco import run_physics_pick_place


if __name__ == "__main__":
    print(json.dumps(run_physics_pick_place(), indent=2))
