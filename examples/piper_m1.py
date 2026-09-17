"""Full PiPER M1 demonstration. Run with MUJOCO_GL=egl and PYTHONPATH=src."""
import sys

from robot_pick_place_agent.cli.main import main

if __name__ == "__main__":
    raise SystemExit(main(["piper-m1", *sys.argv[1:]]))
