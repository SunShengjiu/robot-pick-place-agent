import argparse
import json
from dataclasses import asdict
from robot_pick_place_agent.core.models import ActionStatus
from robot_pick_place_agent.runtime.application import Application


def main(argv=None):
    parser = argparse.ArgumentParser(prog="robot-agent")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("observe")
    plan = sub.add_parser("plan"); plan.add_argument("instruction")
    run = sub.add_parser("run"); run.add_argument("instruction")
    sim = sub.add_parser("simulate"); sim.add_argument("instruction", nargs="?", default="把红色方块放进蓝色盒子"); sim.add_argument("--xml", help="可选的 PiPER/夹爪 MJCF 文件"); sim.add_argument("--observation", choices=("state", "camera"), default="state", help="观测来源")
    args = parser.parse_args(argv)
    app = Application()
    if args.command == "observe":
        print(json.dumps(asdict(app.observe()), ensure_ascii=False, default=str, indent=2))
    elif args.command == "plan":
        try:
            plan_result = app.plan(args.instruction)
            print(json.dumps(asdict(plan_result), ensure_ascii=False, default=str, indent=2))
            return 0 if plan_result.clarification is None else 1
        except Exception as exc:
            print(json.dumps({"status": "failed", "stage": "plan", "message": str(exc)}, ensure_ascii=False, indent=2))
            return 1
    elif args.command == "run":
        result = app.run(args.instruction)
        print(json.dumps(asdict(result), ensure_ascii=False, default=str, indent=2))
        return {ActionStatus.SUCCEEDED: 0, ActionStatus.FAILED: 1, ActionStatus.UNCERTAIN: 2}[result.status]
    else:
        try:
            result = app.simulate(args.instruction, xml_path=args.xml, observation_mode=args.observation)
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0 if result.get("success") else 1
        except RuntimeError as exc:
            parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
