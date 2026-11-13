"""Assemble a standalone source project from the maintained algorithm sources.

Generated copies live only in sim/output. No renderer or IR-SIM dependency is
included. The small private geometry kernel is required for planning validity.
"""

import argparse
from pathlib import Path
import shutil
import tomllib

import yaml


def build(output: Path):
    root = Path(__file__).resolve().parents[2]
    template = root / "planner" / "sdk"
    metadata = tomllib.loads((template / "pyproject.toml").read_text())
    version = metadata["project"]["version"]
    destination = output.resolve() / f"route-plan-sdk-{version}"
    if destination.exists():
        raise FileExistsError(f"SDK output already exists: {destination}; choose a new output directory")
    package = destination / "src" / "route_plan_sdk"
    package.mkdir(parents=True)
    for name in ("api.py", "core.py", "length.py", "stops.py", "motion.py", "ompl_motion.py", "records.py",
                 "escape.py", "adjustment.py"):
        source = (root / "planner" / "src" / "route_plan_planner" / name).read_text()
        # Move only imports of the maintained geometry kernel into a private
        # namespace. All algorithm imports remain relative and unchanged.
        source = source.replace("from route_plan_sim", "from route_plan_sdk._geometry")
        (package / name).write_text(source)
    (package / "__init__.py").write_text(
        '"""Pure algorithm SDK: static scene request -> verified map pose plan."""\n'
        'from .api import Target, PlanningRequest, PlanningResponse, PlanStep, PickupStop, plan\n'
        'from .escape import EscapeResponse, plan_escape\n'
        'from .adjustment import plan_pickup_adjustment\n'
        f'__version__ = "{version}"\n'
        '__all__ = ["Target", "PlanningRequest", "PlanningResponse", "PlanStep", "PickupStop", "plan",\n'
        '           "EscapeResponse", "plan_escape", "plan_pickup_adjustment", "__version__"]\n')
    geometry = package / "_geometry"
    geometry.mkdir()
    for name in ("__init__.py", "scene.py", "boundary.py", "config.py", "control.py"):
        shutil.copy2(root / "sim" / "src" / "route_plan_sim" / name, geometry / name)
    original_world = yaml.safe_load((root / "sim" / "src" / "route_plan_sim" / "world.yaml").read_text())
    minimal_world = {"world": {key: original_world["world"][key] for key in ("width", "height", "step_time")},
                     "robot": {"state": original_world["robot"]["state"]},
                     "custom": {"random_count": 0, "rule_version": original_world["custom"]["rule_version"],
                                "seed": original_world["custom"]["seed"],
                                "target_diameter": original_world["custom"]["target_diameter"]}}
    (geometry / "world.yaml").write_text(yaml.safe_dump(minimal_world), encoding="utf-8")
    (geometry / "config").mkdir()
    shutil.copy2(root / "sim" / "config" / "robot.yaml", geometry / "config" / "robot.yaml")
    shutil.copy2(template / "pyproject.toml", destination / "pyproject.toml")
    shutil.copy2(template / "README.md", destination / "README.md")
    shutil.copytree(template / "examples", destination / "examples", ignore=shutil.ignore_patterns("__pycache__"))
    print(destination)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[2] / "sim" / "output" / "sdk")
    build(parser.parse_args().output)
