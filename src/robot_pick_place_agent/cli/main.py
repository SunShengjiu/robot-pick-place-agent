import argparse
import json
from dataclasses import asdict
from robot_pick_place_agent.runtime.application import Application


def main(argv=None):
    parser = argparse.ArgumentParser(prog="robot-agent")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("observe")
    run = sub.add_parser("run"); run.add_argument("instruction")
    sim = sub.add_parser("simulate"); sim.add_argument("instruction", nargs="?", default="把红色方块放进蓝色盒子"); sim.add_argument("--xml", help="可选的 PiPER/夹爪 MJCF 文件"); sim.add_argument("--observation", choices=("state", "camera"), default="state", help="观测来源")
    args = parser.parse_args(argv)
    app = Application()
    if args.command == "observe":
        print(json.dumps(asdict(app.observe()), ensure_ascii=False, default=str, indent=2))
    elif args.command == "run":
        print(json.dumps(asdict(app.run(args.instruction)), ensure_ascii=False, default=str, indent=2))
    else:
        try:
            print(json.dumps(app.simulate(args.instruction, xml_path=args.xml, observation_mode=args.observation), ensure_ascii=False, indent=2))
        except RuntimeError as exc:
            parser.error(str(exc))
    return 0


if __name__ == "__main__":
    main()
