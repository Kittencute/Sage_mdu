from __future__ import annotations

from fleet_config.common import (
    GZ_PARTITION_VARIABLE,
    ROUTER,
    WORLD_READY,
    exec_process,
    world_ready,
)
from fleet_config.errors import RenderError
from fleet_config.paths import CONTAINER_RENDER_DIR, CONTAINER_WORKSPACE_DIR
from fleet_config.unit import UnitContext
from fleet_unit.manifest import Process

GZ_EXECUTABLE = "gz"
GZ_SERVER = "gz_server"
GZ_CLIENT = "gz_client"
GZ_VERBOSITY = "1"
SOFTWARE_RENDERING_VARIABLE = "LIBGL_ALWAYS_SOFTWARE"
GZ_RESOURCE_PATH_VARIABLE = "GZ_SIM_RESOURCE_PATH"
GZ_SYSTEM_PLUGIN_PATH_VARIABLE = "GZ_SIM_SYSTEM_PLUGIN_PATH"
SIMULATION_SYSTEMS_PACKAGE = "fleet_simulation"
SIMULATION_SYSTEMS_DIR = f"{CONTAINER_WORKSPACE_DIR}/install/{SIMULATION_SYSTEMS_PACKAGE}/lib"
WORLD_MODELS_DIR = f"{CONTAINER_WORKSPACE_DIR}/install/{SIMULATION_SYSTEMS_PACKAGE}/share/{SIMULATION_SYSTEMS_PACKAGE}/models"


def simulator_environment(unit: UnitContext) -> dict[str, str]:
    return {
        GZ_PARTITION_VARIABLE: unit.instance.gz_partition,
        GZ_SYSTEM_PLUGIN_PATH_VARIABLE: SIMULATION_SYSTEMS_DIR,
        GZ_RESOURCE_PATH_VARIABLE: ":".join((WORLD_MODELS_DIR, *unit.gz_resource_paths)),
    }


def processes(unit: UnitContext) -> list[Process]:
    simulation = unit.simulation
    if simulation is None:
        raise RenderError(
            "build world manifest", "the unit carries no simulation configuration", unit=unit.id
        )
    if simulation.seed < 1:
        raise RenderError(
            "build world manifest",
            f"expected a positive seed (gz sim leaves 0 unseeded), got {simulation.seed}",
            unit=unit.id,
        )
    server_cmd = [GZ_EXECUTABLE, "sim", "-r", "-s", "--seed", str(simulation.seed)]
    server_env = simulator_environment(unit)
    if not simulation.gpu:
        server_cmd.append("--headless-rendering")
        server_env[SOFTWARE_RENDERING_VARIABLE] = "1"
    server_cmd += ["-v", GZ_VERBOSITY, f"{CONTAINER_RENDER_DIR}/{simulation.world.sdf_file}"]
    return [
        exec_process(GZ_SERVER, cmd=server_cmd, env=server_env, after=(ROUTER,)),
        world_ready(unit, simulator_environment(unit), after=(GZ_SERVER,)),
        exec_process(
            GZ_CLIENT,
            cmd=[GZ_EXECUTABLE, "sim", "-g", "-v", GZ_VERBOSITY],
            env=simulator_environment(unit),
            after=(WORLD_READY,),
        ),
    ]
