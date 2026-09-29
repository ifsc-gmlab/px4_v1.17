#!/usr/bin/env python3
"""Prepare a Gazebo Classic source tree with the flying-car model and plugins."""

from pathlib import Path
import shutil
import sys


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: prepare.py CLASSIC_SUBMODULE BUILD_SOURCE_DIR", file=sys.stderr)
        return 2

    base = Path(sys.argv[1]).resolve()
    stage = Path(sys.argv[2]).resolve()
    assets = Path(__file__).resolve().parent

    if not (base / "CMakeLists.txt").is_file():
        print(f"Gazebo Classic submodule is not initialized: {base}", file=sys.stderr)
        return 1

    shutil.copytree(base, stage, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns(".git", "build", "__pycache__"))

    for root in ("overlay", "models", "worlds"):
        source = assets / root
        if root == "overlay":
            for file in source.rglob("*"):
                if file.is_file():
                    target = stage / file.relative_to(source)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(file, target)
        else:
            shutil.copytree(source, stage / root, dirs_exist_ok=True)

    model_bindings = assets / "MODEL_BINDINGS.md"
    if model_bindings.is_file():
        shutil.copy2(model_bindings, stage / model_bindings.name)

    print(f"Prepared flying-car Gazebo Classic source: {stage}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
