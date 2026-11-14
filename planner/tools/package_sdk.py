"""One-command SDK packaging. Full regressions are intentionally not run here."""

import argparse
from datetime import datetime, timezone
from email import message_from_bytes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import tomllib
import zipfile

from build_sdk import build

ROOT = Path(__file__).resolve().parents[2]


def run(command, **kwargs):
    subprocess.run([str(value) for value in command], check=True, **kwargs)


def check_wheel(path, project):
    """Fast checks only: no install, solver invocation, or GUI imports."""
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        required = {"route_plan_sdk/__init__.py", "route_plan_sdk/api.py", "route_plan_sdk/observation.py",
                    "route_plan_sdk/direction_policy.py",
                    "route_plan_sdk/_geometry/boundary.py",
                    "route_plan_sdk/_geometry/config/robot.yaml", "route_plan_sdk/_geometry/world.yaml"}
        if not required.issubset(names):
            raise ValueError(f"Missing SDK files: {required - set(names)}")
        for name in names:
            if name.endswith(".py"):
                source = archive.read(name)
                compile(source, name, "exec")
                if any(f"/{module}.py" in name for module in ("app", "drawing", "visualize", "cli")):
                    raise ValueError(f"Display/CLI module must not be packaged: {name}")
                if b"from route_plan_sim" in source or b"from route_plan_planner" in source:
                    raise ValueError(f"Unresolved repository import: {name}")
        metadata_name = next(name for name in names if name.endswith(".dist-info/METADATA"))
        metadata = message_from_bytes(archive.read(metadata_name))
        normalize = lambda value: value.lower().replace("_", "-")
        if normalize(metadata["Name"]) != normalize(project["name"]) or metadata["Version"] != project["version"]:
            raise ValueError("Wheel identity does not match SDK project")
        actual = {normalize(value) for value in metadata.get_all("Requires-Dist", [])}
        expected = {normalize(value) for value in project["dependencies"]}
        if actual != expected:
            raise ValueError(f"Unexpected runtime dependencies: {actual}")


def verify_install(uv, wheel, source, workspace):
    """Optional installation/example check, independent of the repository env."""
    environment = workspace / "verify-venv"
    run([uv, "venv", "--python", sys.executable, environment])
    python = environment / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    run([uv, "pip", "install", "--python", python, wheel])
    consumer = workspace / "consumer-project"
    consumer.mkdir()
    shutil.copy2(source / "examples/quick_start.py", consumer / "app.py")
    with (workspace / "example.log").open("w", encoding="utf-8") as output:
        run([python, "-I", consumer / "app.py"], cwd=consumer, stdout=output)
    run([python, "-I", source / "examples/from_json.py", source / "examples/request.json",
         "--output", workspace / "response.json"], cwd=consumer)
    run([python, "-I", source / "examples/multi_pickup.py"], cwd=consumer)
    run([python, "-I", source / "examples/fence.py"], cwd=consumer)
    run([python, "-I", source / "examples/auto_fence.py"], cwd=consumer)
    run([python, "-I", source / "examples/observation.py"], cwd=consumer)
    run([python, "-I", source / "examples/gateway_coverage.py"], cwd=consumer)
    result = json.loads((workspace / "response.json").read_text())
    if not result["success"] or set(result["expected_collected_ids"]) != {"object-101", "object-205"}:
        raise ValueError("Installed example did not collect both expected targets")
    run([python, "-I", "-c", "import importlib.metadata as m; "
         "names={d.metadata['Name'].lower().replace('_','-') for d in m.distributions()}; "
         "assert names == {'route-plan-sdk','ompl','pyyaml'}, names"])


def package(output, verify=False):
    started = time.monotonic()
    uv = shutil.which("uv")
    if uv is None:
        raise ValueError("Packaging requires uv on PATH")
    project = tomllib.loads((ROOT / "planner/sdk/pyproject.toml").read_text())["project"]
    version = project["version"]
    output = Path(output).absolute()
    output.parent.mkdir(parents=True, exist_ok=True)
    # All work happens in staging. Existing artifacts are replaced only after
    # builds and requested checks pass; user-edited source directories are not
    # touched. Re-running the command needs no cleanup or new directory name.
    with TemporaryDirectory(prefix="sdk-package-", dir=output.parent) as directory:
        staging = Path(directory)
        source = build(staging / "source")
        dist = staging / "dist"
        run([uv, "build", "--sdist", "--wheel", "--out-dir", dist, source])
        wheel = dist / f"route_plan_sdk-{version}-py3-none-any.whl"
        archive = dist / f"route_plan_sdk-{version}.tar.gz"
        check_wheel(wheel, project)
        source_zip = dist / f"route_plan_sdk-{version}-source.zip"
        with zipfile.ZipFile(source_zip, "w", zipfile.ZIP_DEFLATED) as bundle:
            for path in sorted(source.rglob("*")):
                if path.is_file() and "__pycache__" not in path.parts:
                    bundle.write(path, arcname=path.relative_to(source.parent))
        if verify:
            verify_install(uv, wheel, source, staging)
        git = shutil.which("git")
        revision = subprocess.run([git, "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True) if git else None
        dirty = subprocess.run([git, "status", "--porcelain"], cwd=ROOT, capture_output=True, text=True) if git else None
        info = {"sdk": project["name"], "version": version,
                "built_at_utc": datetime.now(timezone.utc).isoformat(),
                "source_commit": revision.stdout.strip() if revision and revision.returncode == 0 else None,
                "source_dirty": bool(dirty.stdout.strip()) if dirty and dirty.returncode == 0 else None,
                "install_verified": verify, "runtime_dependencies": project["dependencies"],
                "full_regression_run": False}
        build_info = dist / "BUILD_INFO.json"
        build_info.write_text(json.dumps(info, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        artifacts = [wheel, archive, source_zip, build_info]
        sums = dist / "SHA256SUMS"
        sums.write_text("".join(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n"
                                for path in artifacts), encoding="utf-8")
        output.mkdir(parents=True, exist_ok=True)
        for path in artifacts + [sums]:
            os.replace(path, output / path.name)
    print(f"\nSDK {version} packaged in {time.monotonic() - started:.2f}s: {output}")
    print("Checks: wheel structure, Python syntax, resources, dependencies" +
          (", isolated install and examples" if verify else " (no full regression)"))
    return output


def main():
    parser = argparse.ArgumentParser(description="固定纯算法 SDK 打包入口，默认不跑完整回归")
    parser.add_argument("--output", type=Path, default=ROOT / "sim/output/sdk/dist")
    parser.add_argument("--verify", action="store_true", help="额外创建独立环境，验证安装和调用示例")
    args = parser.parse_args()
    try:
        package(args.output, args.verify)
    except (OSError, ValueError, SyntaxError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"Packaging failed: {error}\n")


if __name__ == "__main__":
    main()
