from __future__ import annotations

from dataclasses import replace

from fleet_config.classes.aeroscout import AEROSCOUT
from fleet_config.classes.terrascout import TERRASCOUT
from fleet_config.model import (
    Datum,
    Features,
    PhysicalTarget,
    RobotGroup,
    SimTarget,
    SourceFile,
    StationTarget,
    Target,
    World,
)

FOREST = World(name="forest", template=SourceFile("fleet_simulation", "worlds/forest.sdf.in"))

SIM = SimTarget(
    world=FOREST,
    datum=Datum(lat=47.397742, lon=8.545594, elevation=488.0),
    seed=1,
    gpu=True,
    groups={
        "scouts": RobotGroup(TERRASCOUT, count=1, spawn_poses=((0.0, 0.0, 3.0, 0.0, 0.0, 0.0),)),
        #"aeros": RobotGroup(AEROSCOUT, count=1, spawn_poses=((5.0, 2.0, 0.0, 0.0, 0.0, 0.0),)),
    },
)

TARGETS: dict[str, Target] = {
    "terrascout1": PhysicalTarget(robot_class=TERRASCOUT, id="terrascout1"),
    "sim": SIM,
    "sim-madum": replace(SIM, features=Features(recording_enabled=True, madum_enabled=True)),
    "station": StationTarget(robots=(("terrascout1", TERRASCOUT),)),
}
