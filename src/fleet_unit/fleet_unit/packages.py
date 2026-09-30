from __future__ import annotations

from pathlib import Path

from fleet_unit.errors import UnitError


def package_share(package: str) -> Path:
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )

    operation = f"resolve share directory of package {package}"
    try:
        return Path(get_package_share_directory(package))
    except PackageNotFoundError as exc:
        raise UnitError(operation, exc) from exc


def package_executable(package: str, executable: str) -> Path:
    from ros2pkg.api import PackageNotFound
    from ros2run.api import MultipleExecutables, get_executable_path

    operation = f"resolve executable {executable} of package {package}"
    try:
        resolved = get_executable_path(package_name=package, executable_name=executable)
    except PackageNotFound as exc:
        raise UnitError(operation, exc) from exc
    except MultipleExecutables as exc:
        raise UnitError(
            operation,
            f"several executables match: {', '.join(sorted(exc.paths))}",
        ) from exc
    if resolved is None:
        raise UnitError(operation, "the package installs no executable of that name")
    return Path(resolved)


def package_file(package: str | None, path: str) -> Path:
    if package is None:
        return Path(path)
    return package_share(package) / path
