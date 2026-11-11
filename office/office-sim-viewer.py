"""Interactive desktop viewer for the isolated office navigation simulation."""
import hb_config
from dataclasses import replace
import json
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from homebody.backends.mujoco import MujocoBackend
from homebody.session.settings import Settings
from homebody.skills.contract import Context, Runner
from homebody.skills.navigate_real import task as navigate_real
from homebody.skills.registry import task_registry

root = Path(hb_config.expand('${HB_REPO_ROOT}'))
scene = Path(hb_config.expand('${HB_OFFICE_ROOT}/office-map-proxy-scene'))
base = Settings()
settings = replace(base, camera=replace(base.camera, width=160, height=120))
robot = MujocoBackend(root, settings, scene=scene)
robot.model.vis.map.zfar = 100.0 / robot.model.stat.extent
robot.model.vis.map.fogstart = 80.0 / robot.model.stat.extent
robot.model.vis.map.fogend = 100.0 / robot.model.stat.extent
pending = []
def key(code):
    if code in (ord('N'), ord('W'), ord('H')):
        pending.append(chr(code))

try:
    with mujoco.viewer.launch_passive(robot.model, robot.data, key_callback=key,
                                     show_left_ui=False, show_right_ui=False) as viewer:
        # MuJoCo hides group 3 by default; the map-derived walls use this group.
        viewer.opt.geomgroup[2] = 1
        viewer.opt.geomgroup[3] = 1
        robot.model.vis.map.zfar = 100.0 / robot.model.stat.extent
        robot.model.vis.map.fogstart = 80.0 / robot.model.stat.extent
        robot.model.vis.map.fogend = 100.0 / robot.model.stat.extent
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.fixedcamid = -1
        viewer.cam.trackbodyid = -1
        viewer.cam.lookat[:] = [.725, -1.75, .5]
        viewer.cam.distance = 19
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -85
        viewer.sync()
        print('VIEW', viewer.cam.type, viewer.cam.lookat.tolist(), viewer.cam.distance,
              viewer.cam.azimuth, viewer.cam.elevation, viewer.opt.geomgroup.tolist(),
              robot.model.vis.map.zfar, robot.model.stat.extent, flush=True)
        print('OFFICE_GEOMETRY_VISIBLE group=3 camera=office-overview', flush=True)
        original_tick = robot._tick
        last_wall = [time.monotonic()]
        last_sync = [robot.time]
        def tick():
            if not viewer.is_running():
                raise KeyboardInterrupt
            before = robot.time
            original_tick()
            delay = robot.time - before - (time.monotonic()-last_wall[0])
            if delay > 0:
                time.sleep(delay)
            last_wall[0] = time.monotonic()
            if robot.time-last_sync[0] >= 1/30:
                viewer.sync()
                last_sync[0] = robot.time
        robot._tick = tick
        home = np.array(json.loads((scene/'task_scene.json').read_text())['robot_start'][:2])
        print('SIMULATION ONLY. N: north goal; W: west goal; H: original simulation start. No robot connection.', flush=True)
        print('VIEWER_READY', flush=True)
        while viewer.is_running():
            if pending:
                command = pending.pop(0)
                goal = home + ({'N':np.array([0.,.8]), 'W':np.array([-.8,0.]), 'H':np.zeros(2)}[command])
                print('SIM_GOAL '+json.dumps(goal.tolist()), flush=True)
                start = robot.snapshot()
                context = Context(robot, robot, {}, settings=settings, navigation=navigate_real,
                                  navigation_evidence=robot.navigation_evidence)
                result = Runner(context, task_registry()).run('navigate', {
                    'goal_xy_m':goal.tolist(), 'snap':False}, start)
                print(json.dumps({'code':result.code, 'ok':result.ok, 'end_xy':robot.measure().base_pose[:2].tolist()}), flush=True)
            else:
                robot.stop(robot.epoch)
                robot.advance(robot.epoch, .02)
except KeyboardInterrupt:
    pass
finally:
    robot.close()
