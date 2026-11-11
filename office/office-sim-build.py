"""Build a map-derived MuJoCo proxy for offline G1 navigation checks only.

The supplied occupancy grid has an open outer boundary and no verified room
labels. This scene is deliberately separate from any real-robot map or config.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt


def read_yaml(path: Path) -> dict:
    settings = {}
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        key, value = line.split(":", 1)
        try:
            settings[key.strip()] = ast.literal_eval(value.strip())
        except (ValueError, SyntaxError):
            settings[key.strip()] = value.strip()
    return settings


def rectangles(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
    """Merge equal horizontal occupied runs on consecutive image rows."""
    result = []
    active = {}
    for row_index, row in enumerate(mask):
        edges = np.diff(np.r_[False, row, False].astype(np.int8))
        runs = list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))
        next_active = {}
        for left, right in runs:
            key = (int(left), int(right))
            start = active.pop(key, row_index)
            next_active[key] = start
        result.extend((left, right, start, row_index)
                      for (left, right), start in active.items())
        active = next_active
    result.extend((left, right, start, len(mask))
                  for (left, right), start in active.items())
    return result


def main(map_dir: Path, output: Path) -> None:
    config = read_yaml(map_dir / "map.yaml")
    image_path = map_dir / config["image"]
    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.uint8)
    height, width = pixels.shape
    resolution = float(config["resolution"])
    origin_x, origin_y, yaw = map(float, config["origin"])
    if yaw != 0 or int(config["negate"]) != 0:
        raise ValueError("This proxy builder requires zero map yaw and negate=0")
    probability = (255 - pixels.astype(float)) / 255
    occupied = probability >= float(config["occupied_thresh"])
    free = probability <= float(config["free_thresh"])
    unknown = ~(occupied | free)
    if np.any(unknown):
        occupied |= unknown  # fail closed in the offline proxy
    original_occupied = int(occupied.sum())
    # The provided grid labels almost its entire outer edge free. Add a virtual
    # perimeter to the simulation only so paths cannot leave the raster extent.
    occupied[[0, -1], :] = True
    occupied[:, [0, -1]] = True

    step_cells = round(0.8 / resolution)
    clearance = distance_transform_edt(~occupied) * resolution
    # Pick a high-clearance, short eastbound route in the visible upper hall.
    # This is a smoke-test route, not an asserted robot start or meeting room.
    candidates = []
    for row in range(70, min(125, height - 1)):
        for col in range(120, min(220, width - step_cells - 1)):
            minimum = float(clearance[row, col:col + step_cells + 1].min())
            if minimum >= 0.65:
                candidates.append((minimum, row, col))
    if not candidates:
        raise ValueError("No 0.8 m proxy route with at least 0.65 m clearance")
    minimum, start_row, start_col = max(candidates)
    start_x = origin_x + (start_col + 0.5) * resolution
    start_y = origin_y + (height - start_row - 0.5) * resolution
    goal_x = start_x + step_cells * resolution

    scene = ET.Element("mujoco", {"model": "office_map_proxy"})
    ET.SubElement(scene, "asset")
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "geom", {
        "name": "floor__office_proxy", "type": "plane", "pos": "0 0 0",
        "size": "0 0 0.05", "contype": "2", "conaffinity": "3",
        "group": "2", "friction": "1.2 0.005 0.0001",
        "rgba": "0.7 0.75 0.75 1", "mass": "0",
    })
    boxes = rectangles(occupied)
    for index, (left, right, top, bottom) in enumerate(boxes):
        x0, x1 = origin_x + left * resolution, origin_x + right * resolution
        y0 = origin_y + (height - bottom) * resolution
        y1 = origin_y + (height - top) * resolution
        ET.SubElement(world, "geom", {
            "name": f"structure__map_{index:04d}", "type": "box",
            "pos": f"{(x0 + x1) / 2:.5f} {(y0 + y1) / 2:.5f} 1.1",
            "size": f"{(x1 - x0) / 2:.5f} {(y1 - y0) / 2:.5f} 1.1",
            "contype": "2", "conaffinity": "3", "group": "3",
            "rgba": "0.28 0.32 0.36 1", "mass": "0",
        })
    output.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(scene).write(output / "scene.xml", encoding="unicode")
    (output / "task_scene.json").write_text(json.dumps({
        "name": "Office occupancy proxy (not surveyed for robot safety)",
        "frame": "map", "robot_start": [start_x, start_y, 0.0],
        "smoke_goal_xy": [goal_x, start_y],
        "movable_entities": [], "collision_boxes": [],
        "default_task": "Metric offline navigation test only; room labels unknown.",
    }, indent=2))
    (output / "physics.json").write_text('{"objects": {}}\n')
    transform = np.array([
        [resolution, 0, origin_x + resolution / 2],
        [0, -resolution, origin_y + (height - 0.5) * resolution],
        [0, 0, 1],
    ])
    np.savez_compressed(output / "map.npz", occupancy=occupied, T_map_px=transform)
    report = {
        "source_map_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        "map_shape": [height, width], "resolution_m": resolution,
        "original_occupied_cells": original_occupied,
        "unknown_cells": int(unknown.sum()),
        "simulation_occupied_cells": int(occupied.sum()),
        "collision_rectangles": len(boxes),
        "test_start_xy_m": [start_x, start_y],
        "test_goal_xy_m": [goal_x, start_y],
        "minimum_route_clearance_m": minimum,
        "virtual_perimeter": True,
        "robot_start_and_room_labels_surveyed": False,
    }
    (output / "build-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
