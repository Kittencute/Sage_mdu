from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

from fleet_unit import commands
from fleet_unit.errors import UnitError
from fleet_unit.manifest import load_manifest
from fleet_unit.supervisor import UnitSupervisor

LOG_FORMAT = "%(asctime)s %(levelname)s %(name)s %(message)s"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="fleet_unit", description="Run one SAGE robot unit from its manifest"
    )
    parser.add_argument(
        "command",
        choices=("run",),
        help="start the unit router and every manifest process, then supervise them",
    )
    parser.add_argument("--unit-dir", required=True, type=Path, help="unit render directory")
    arguments = parser.parse_args(argv)
    logging.basicConfig(stream=sys.stderr, level=logging.INFO, format=LOG_FORMAT)
    log = logging.getLogger("fleet_unit")
    try:
        manifest = load_manifest(arguments.unit_dir)
        os.environ.update(commands.ros_environment(manifest))
        log.info("[%s] starting unit from %s", manifest.unit, manifest.unit_dir)
        return UnitSupervisor(manifest).run()
    except UnitError as exc:
        log.error("%s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
