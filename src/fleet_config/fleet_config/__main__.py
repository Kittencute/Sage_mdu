from __future__ import annotations

import argparse
import sys
from pathlib import Path

from fleet_config.errors import RenderError
from fleet_config.model import parse_instance
from fleet_unit.errors import UnitError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="fleet_config")
    commands = parser.add_subparsers(dest="command", required=True)
    render = commands.add_parser("render", help="render a target into a directory")
    render.add_argument("--target", required=True)
    render.add_argument("--instance", default="1")
    render.add_argument("--out", required=True, type=Path)
    commands.add_parser("list", help="print the known targets")
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "list":
            from fleet_config.targets import TARGETS

            for name in TARGETS:
                print(name)
            return 0
        from fleet_config.render import render_target

        render_target(arguments.target, parse_instance(arguments.instance), arguments.out)
        return 0
    except (RenderError, UnitError) as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
