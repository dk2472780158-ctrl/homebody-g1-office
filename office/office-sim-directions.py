"""Additional navigation integration checks in the existing offline MuJoCo proxy.

Uses the old simulation-only start; never loads the real localization candidate.
"""
import hb_config
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from homebody.backends.mujoco import MujocoBackend
from homebody.session.settings import Settings
from homebody.skills.contract import Context, Runner
from homebody.skills.navigate_real import task as navigate_real
from homebody.skills.registry import task_registry


parser = argparse.ArgumentParser()
parser.add_argument("case", choices=["north", "west", "occupied"])
args = parser.parse_args()
root = Path(hb_config.expand('${HB_REPO_ROOT}'))
scene = Path(hb_config.expand('${HB_OFFICE_ROOT}/office-map-proxy-scene'))
config = json.loads((scene / "task_scene.json").read_text())
start_xy = np.array(config["robot_start"][:2])
with np.load(scene / "map.npz") as artifact:
    occupancy = artifact["occupancy"]
    transform = artifact["T_map_px"]
if args.case == "occupied":
    rows, cols = np.nonzero(occupancy)
    points = (transform @ np.array([cols, rows, np.ones(len(cols))]))[:2].T
    goal = points[np.argmin(np.linalg.norm(points - start_xy, axis=1))].tolist()
else:
    offset = np.array([0., .8]) if args.case == "north" else np.array([-.8, 0.])
    goal = (start_xy + offset).tolist()
base = Settings()
settings = replace(base, camera=replace(base.camera, width=160, height=120))
robot = MujocoBackend(root, settings, scene=scene)
try:
    start = robot.snapshot()
    context = Context(robot, robot, {}, settings=settings, navigation=navigate_real,
                      navigation_evidence=robot.navigation_evidence)
    result = Runner(context, task_registry()).run("navigate", {
        "goal_xy_m": goal, "snap": False,
    }, start)
    end = robot.measure()
    error = float(np.linalg.norm(end.base_pose[:2] - goal))
    motion = float(np.linalg.norm(end.base_pose[:2] - start.base_pose[:2]))
    valid = bool(robot.evaluation_state()["physics_valid"])
    passed = (not result.ok and motion < .1 and valid) if args.case == "occupied" else (
        result.ok and error <= settings.motion.arrival_distance and valid)
    report = {
        "scope": "offline MuJoCo map proxy; no real start or room semantics",
        "case": args.case, "code": result.code, "ok": result.ok,
        "start_xy": start.base_pose[:2].tolist(), "goal_xy": goal,
        "end_xy": end.base_pose[:2].tolist(), "error_m": error,
        "motion_from_start_m": motion, "physics_valid": valid,
        "sim_time_s": robot.time, "passed": bool(passed),
    }
    print(json.dumps(report), flush=True)
    raise SystemExit(0 if passed else 1)
finally:
    robot.close()
