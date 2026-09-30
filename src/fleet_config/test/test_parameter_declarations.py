import ast
import re
from pathlib import Path

import pytest
import yaml

from fleet_config import model
from fleet_config.model import Instance
from fleet_config.render import render_target
from fleet_config.targets import TARGETS
from fleet_unit.manifest import load_manifest

SOURCE_ROOT = model.SOURCE_ROOT
NUMERIC_TYPES = {"DOUBLE", "INTEGER", "double", "int"}
CPP_DECLARATION = re.compile(
    r'declare_parameter<(\w+)>\(\s*"(\w+)",\s*describe\("([^"]+)"(?:,\s*([-\d.e+]+),\s*([-\d.e+]+))?\)\)'
)
ROS_PARAMETERS = {"use_sim_time"}


def first_party_packages():
    return {
        path.parent.name: path.parent
        for path in SOURCE_ROOT.glob("*/package.xml")
        if path.parent.name != "third_party"
    }


def literal(node):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return -literal(node.operand)
    if isinstance(node, ast.Attribute):
        return node.attr
    raise ValueError(f"not a literal: {ast.unparse(node)}")


def conditional_calls(tree):
    return {
        id(call)
        for branch in ast.walk(tree)
        if isinstance(branch, ast.If)
        for statement in branch.body + branch.orelse
        for call in ast.walk(statement)
        if isinstance(call, ast.Call)
    }


def python_declarations(path):
    declarations = {}
    tree = ast.parse(path.read_text())
    conditional = conditional_calls(tree)
    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        name = (
            call.func.attr if isinstance(call.func, ast.Attribute) else getattr(call.func, "id", "")
        )
        if name == "declare_parameter":
            raise AssertionError(f"{path}: declare_parameter outside fleet_common.params.declare")
        if name != "declare":
            continue
        arguments = [literal(argument) for argument in call.args[1:]]
        arguments += [None] * (6 - len(arguments))
        keywords = {keyword.arg: literal(keyword.value) for keyword in call.keywords}
        parameter, kind, description, low, high, constraints = arguments
        declarations[parameter] = {
            "type": kind,
            "description": description,
            "low": keywords.get("low", low),
            "high": keywords.get("high", high),
            "constraints": keywords.get("constraints", constraints) or "",
            "optional": id(call) in conditional,
        }
    return declarations


def cpp_declarations(package_dir):
    declarations = {}
    for path in [*package_dir.rglob("*.hpp"), *package_dir.rglob("*.cpp")]:
        text = path.read_text()
        found = CPP_DECLARATION.findall(text)
        if len(found) != text.count("declare_parameter<"):
            raise AssertionError(f"{path}: a declare_parameter call is not in the describe form")
        for kind, parameter, description, low, high in found:
            declarations[parameter] = {
                "type": kind,
                "description": description,
                "low": float(low) if low else None,
                "high": float(high) if high else None,
                "constraints": "",
                "optional": False,
            }
    return declarations


def declarations_for(package, executable):
    package_dir = first_party_packages()[package]
    sources = [
        path
        for path in package_dir.rglob(f"{Path(executable).stem}.py")
        if "test" not in path.parts
    ]
    if sources:
        return python_declarations(sources[0])
    return cpp_declarations(package_dir)


def all_first_party_declarations():
    declarations = []
    for package_dir in first_party_packages().values():
        for path in package_dir.rglob("*.py"):
            if "test" not in path.parts and path.name != "params.py":
                declarations += [
                    (path, name, spec) for name, spec in python_declarations(path).items()
                ]
        declarations += [
            (package_dir, name, spec) for name, spec in cpp_declarations(package_dir).items()
        ]
    return declarations


def test_every_declaration_is_described_and_bounded():
    declarations = all_first_party_declarations()
    assert declarations
    for source, name, spec in declarations:
        assert spec["description"], f"{source}: parameter {name} has no description"
        if spec["type"] in NUMERIC_TYPES:
            assert spec["low"] is not None or spec["constraints"], (
                f"{source}: numeric parameter {name} has neither a range nor constraints"
            )


def rendered_values(unit_dir, process):
    values = {}
    for params_file in process.params:
        document = yaml.safe_load((unit_dir / params_file).read_text()) or {}
        for key, section in document.items():
            if key == "/**" or key.split("/")[-1] == process.node_name:
                values.update(section.get("ros__parameters", {}))
    values.update(process.param_values)
    return {name: value for name, value in values.items() if name not in ROS_PARAMETERS}


@pytest.mark.parametrize("target", sorted(TARGETS))
def test_rendered_parameters_match_their_declarations(target, tmp_path, monkeypatch):
    monkeypatch.setattr(model, "SITE_FILE", SOURCE_ROOT / "fleet_config" / "site.example.yaml")
    render_target(target, Instance(1), tmp_path)
    packages = first_party_packages()
    for unit_dir in sorted((tmp_path / "units").iterdir()):
        manifest = load_manifest(unit_dir)
        for process in manifest.processes:
            package = getattr(process, "package", None)
            if package not in packages or process.kind != "node":
                continue
            declared = declarations_for(package, process.executable)
            values = rendered_values(unit_dir, process)
            where = f"{target}/{unit_dir.name}/{process.name}"
            for name in values.keys() - declared.keys():
                raise AssertionError(
                    f"{where}: rendered parameter {name} is not declared by the node"
                )
            for name, spec in declared.items():
                if name not in values:
                    assert spec["optional"], (
                        f"{where}: declared parameter {name} has no rendered value"
                    )
                    continue
                if spec["low"] is not None:
                    assert spec["low"] <= values[name] <= spec["high"], (
                        f"{where}: {name}={values[name]} is outside [{spec['low']}, {spec['high']}]"
                    )
