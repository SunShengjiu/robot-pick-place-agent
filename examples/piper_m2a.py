"""Full PiPER M2a TCP IK demonstration. Run with MUJOCO_GL=egl."""
import sys

from robot_pick_place_agent.adapters.simulation.piper_m2a import run_m2a

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="artifacts/piper_m2a")
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    result = run_m2a(args.output, video=not args.no_video)
    print(result)
    raise SystemExit(0 if result["passed"] else 1)
