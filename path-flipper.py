#!/usr/bin/env python

import fnmatch
import json
import tomllib
from pathlib import Path


GEN_SUFFIX = " (G)"
FLIPIGNORE = ".flipignore"
MAX_X = 16.54
MAX_Y = 8.07

ROOT_PATH = Path(__file__).parent / "src" / "main" / "deploy" / "pathplanner"
AUTOS_DIR = ROOT_PATH / "autos"
PATHS_DIR = ROOT_PATH / "paths"

REPLACEMENTS = {
    # "Left": "Right",
    # "left": "right",
}


def get_replaced_name(name: str):
    for k, v in REPLACEMENTS.items():
        name = name.replace(k, v)
    return name


def get_gen_path(fn: Path):

    if fn.stem.endswith(GEN_SUFFIX):
        return

    name = get_replaced_name(fn.stem)
    return fn.with_stem(name + GEN_SUFFIX)


def is_ignored(fn: Path, ignores: list[str]):
    return fn.stem.endswith(GEN_SUFFIX) or any(
        fnmatch.fnmatch(fn, pat) for pat in ignores
    )


def read_ignore_file(fn: Path):
    ignore_file = fn / FLIPIGNORE
    if ignore_file.is_file():
        with ignore_file.open() as data:
            return [
                line.rstrip("\n")
                for line in data.readlines()
                if not line.startswith("#")
            ]
    return []


def process_auto_command(command, path_ignores: list[str]):
    """
    Recursively process commands in groups.

    :param command: The command to drill down into.
    """
    if command["type"] in ("sequential", "parallel"):
        command["data"]["commands"] = [
            process_auto_command(c, path_ignores) for c in command["data"]["commands"]
        ]
    elif command["type"] == "path":
        path_name = command["data"]["pathName"]
        if not is_ignored((PATHS_DIR / path_name).with_suffix(".path"), path_ignores):
            command["data"]["pathName"] = get_replaced_name(path_name) + GEN_SUFFIX
    return command


def process_auto(auto_fn: Path, path_ignores: list[str]):

    gen_auto_fn = get_gen_path(auto_fn)
    if not gen_auto_fn:
        return

    # Read
    with auto_fn.open() as auto_file:
        json_data = json.load(auto_file)

    # Process
    process_auto_command(json_data["command"], path_ignores)

    # Write
    with gen_auto_fn.open("w") as gen_file:
        json.dump(json_data, gen_file, indent=2)


def process_path(path_fn: Path):

    gen_path_fn = get_gen_path(path_fn)
    if not gen_path_fn:
        return

    # Read
    with path_fn.open() as auto_file:
        json_data = json.load(auto_file)

    # Process
    ## Waypoints
    for wp in json_data["waypoints"]:
        for pose in ["anchor", "prevControl", "nextControl"]:
            if wp[pose] is not None:
                wp[pose]["y"] = MAX_Y - wp[pose]["y"]
    ## Rotation Targets
    for rot in json_data["rotationTargets"]:
        rot["rotationDegrees"] = -rot["rotationDegrees"]
    json_data["goalEndState"]["rotation"] = -json_data["goalEndState"]["rotation"]

    # Write
    with gen_path_fn.open("w") as gen_file:
        json.dump(json_data, gen_file, indent=2)


def process_all_paths():
    ignores = read_ignore_file(PATHS_DIR)
    for p in PATHS_DIR.rglob("*.path"):
        if p.is_file() and not is_ignored(p.relative_to(PATHS_DIR), ignores):
            print("Processing", p)
            process_path(p)


def clean_all_paths():
    for p in PATHS_DIR.rglob("*.path"):
        if p.is_file() and p.stem.endswith(GEN_SUFFIX):
            print("Deleting", p)
            p.unlink(True)


def process_all_autos():
    ignores = read_ignore_file(AUTOS_DIR)
    path_ignores = read_ignore_file(PATHS_DIR)
    for p in AUTOS_DIR.rglob("*.auto"):
        if p.is_file() and not is_ignored(p.relative_to(AUTOS_DIR), ignores):
            print("Processing", p)
            process_auto(p, path_ignores)


def clean_all_autos():
    for p in AUTOS_DIR.rglob("*.auto"):
        if p.is_file() and p.stem.endswith(GEN_SUFFIX):
            print("Deleting", p)
            p.unlink(True)


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("generate", "clean"))
    args = parser.parse_args()

    if args.action == "clean":
        clean_all_paths()
        clean_all_autos()
    elif args.action == "generate":
        process_all_paths()
        process_all_autos()


if __name__ == "__main__":
    main()
