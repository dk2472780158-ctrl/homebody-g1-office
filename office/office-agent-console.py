"""Office-only HomeBody console with nine fixed places and two simulated greetings."""
from __future__ import annotations
import hb_config

import argparse
import os
import json
import math
import re
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from homebody.backends.mujoco import MujocoBackend
from homebody.helpers.navigation import body_frame, cell_free, checked_segment, free_space, pixel
from homebody.helpers.kinematics import Arm
from homebody.session.recorder import Recorder
from homebody.session.session import Session
from homebody.session.settings import Settings
from homebody.skills import navigate_sim
from homebody.skills.contract import Result, SkillFailure, declared
from homebody.ui.console import Console, Controls
from homebody.ui.scene import SceneView
from homebody.ui.theme import note, section
from homebody.vlm.catalog import make, model_label
from homebody.vlm.observations import semantic_prior
from homebody.vlm.provider import Decision
from office_real_workflow import RealWorkflow,open_office_server
from office_landmarks import LANDMARK_PLACES,visible_marks,approach_points

ROOT = Path(hb_config.expand('${HB_REPO_ROOT}'))
ARTIFACTS = Path(hb_config.expand('${HB_OFFICE_ROOT}'))
SCENE = ARTIFACTS / "office-map-proxy-scene"
WAYPOINT_FILE = ARTIFACTS / "office-waypoints.json"
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
PLACES = (
    ("go_north", "北"), ("go_northeast", "东北"), ("go_east", "东"),
    ("go_southeast", "东南"), ("go_south", "南"), ("go_southwest", "西南"),
    ("go_west", "西"), ("go_northwest", "西北"), ("go_door", "门口"),
    *LANDMARK_PLACES.items(),
)
MAP_TASK_PREFIX = "__office_map_click__:"

class OfficeControls(Controls):
    real_workflow=None
    def submit(self,text):
        if self.real_workflow is not None and self.real_workflow.real_mode:
            return self.real_workflow.submit(text)
        return super().submit(text)
    def stop(self):
        if self.real_workflow is not None and self.real_workflow.real_mode:
            return self.real_workflow.request_stop()
        return super().stop()


def single_timed_move(task):
    """Only a plain one-direction, one-duration operator request gets an automatic done."""
    if any(word in task for word in ("然后", "接着", "随后", "先", "再", "并且", "之后", "，", ",")):
        return False
    directions = re.findall(r"(?:向|往|朝)?(东北|东南|西北|西南|[左右前后东南西北])"
                            r"(?:方)?(?:走|移动|横移|退|步)", task)
    durations = re.findall(r"(?:\d+(?:\.\d+)?|[一二三四五六七八九十两]+)\s*秒", task)
    return len(directions) == len(durations) == 1


def waypoints(navigation):
    saved = json.loads(WAYPOINT_FILE.read_text())
    if saved.get("map_shape") != list(navigation.occupied.shape) or not np.allclose(
            saved.get("T_map_px"), navigation.T_map_px, atol=1e-9):
        raise ValueError("Saved office points do not match the active navigation map")
    saved["points"]=approach_points(ARTIFACTS,saved["points"])
    points = {}
    for _, label in PLACES:
        value = np.asarray(saved["points"][label], dtype=float)
        if value.shape != (2,) or not np.isfinite(value).all():
            raise ValueError(f"Invalid office point: {label}")
        points[label] = value
    return points


class PlaceSkill:
    NEEDS = navigate_sim.NEEDS
    FAILURES = navigate_sim.FAILURES

    def __init__(self, name, label):
        self.name, self.label = name, label
        self.PROMPT = (f"{name}: Go to the saved {label} point in the upper rectangular office. "
                       f"Use for Chinese requests such as 去{label}、到{label}边. "
                       "No arguments. The fixed saved point is used; never supply another coordinate.")

    def prepare(self, arguments, selected, current, planning):
        if arguments:
            raise SkillFailure("INVALID_ARGUMENT", "This named place takes no arguments")
        goal = waypoints(current.navigation)[self.label]
        return navigate_sim.prepare({"goal_xy_m": goal.tolist(), "snap": False},
                                    selected, current, planning)

    def execute(self, plan, context):
        result = navigate_sim.execute(plan, context)
        return Result(self.name, result.code, result.message,
                      {**result.details, "place": self.label})


@dataclass(frozen=True)
class MovePlan:
    epoch: int
    direction: str
    duration_s: float
    speed_m_s: float
    unit_xy: np.ndarray
    start_xy: np.ndarray
    target_xy: np.ndarray


class MoveForSkill:
    NEEDS = ("base_velocity", "advance", "stop")
    FAILURES = ("NO_ROUTE", "PATH_BLOCKED")
    PROMPT = ("move_for: Walk at the current configured base speed for a specified time. "
              "Arguments: direction is forward, backward, left, right (relative to the robot), "
              "or north, northeast, east, southeast, south, southwest, west, northwest "
              "(fixed office-map compass: north +y, east +x); duration_s is 0.1 to 5.0 seconds. "
              "Use for 按照现在的速度往左走两秒, 往北走两秒, 向东南走两秒, 后退一秒. "
              "Left/right are sideways steps, not turns. Each single timed request is one move. "
              "Refuse a blocked straight path; never substitute a named destination.")

    def prepare(self, arguments, selected, current, planning):
        if set(arguments) != {"direction", "duration_s"}:
            raise SkillFailure("INVALID_ARGUMENT", "Supply direction and duration_s only")
        direction, duration = arguments["direction"], arguments["duration_s"]
        if direction not in ("forward", "backward", "left", "right", "north", "northeast",
                             "east", "southeast", "south", "southwest", "west", "northwest"):
            raise SkillFailure("INVALID_ARGUMENT", "Unknown movement direction")
        if (type(duration) not in (int, float) or not math.isfinite(duration)
                or not 0.1 <= duration <= 5.0):
            raise SkillFailure("INVALID_ARGUMENT", "duration_s must be between 0.1 and 5.0")
        motion = planning.settings.motion
        speed = min(motion.drive_speed, motion.loaded_drive_speed) if planning.carried else motion.drive_speed
        yaw = float(current.base_pose[2])
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        left = np.array([-math.sin(yaw), math.cos(yaw)])
        diagonal = 1 / math.sqrt(2)
        unit = {"forward": forward, "backward": -forward, "left": left, "right": -left,
                "north": np.array([0., 1.]), "northeast": np.array([diagonal, diagonal]),
                "east": np.array([1., 0.]), "southeast": np.array([diagonal, -diagonal]),
                "south": np.array([0., -1.]), "southwest": np.array([-diagonal, -diagonal]),
                "west": np.array([-1., 0.]), "northwest": np.array([-diagonal, diagonal])}[direction]
        start = current.base_pose[:2].copy()
        target = start + unit * speed * duration
        free = free_space(current.navigation, motion.robot_radius)
        if not checked_segment(current.navigation, free, start, target):
            raise SkillFailure("NO_ROUTE", "Straight timed movement would cross occupied or unsafe map cells")
        return MovePlan(current.epoch, direction, float(duration), speed, unit, start, target)

    def execute(self, plan, context):
        frame = context.observe(plan.epoch)
        start_time = frame.time
        motion = context.settings.motion
        navigation = frame.navigation
        free = free_space(navigation, motion.robot_radius)
        context.stage(f"Moving {plan.direction} for {plan.duration_s:g} s")
        try:
            while frame.time - start_time + 1e-9 < plan.duration_s:
                if not checked_segment(navigation, free, frame.base_pose[:2], plan.target_xy):
                    raise SkillFailure("PATH_BLOCKED", "The remaining straight path is no longer clear")
                command = body_frame(frame.base_pose, plan.unit_xy * plan.speed_m_s)
                context.actions.base_velocity(plan.epoch, float(command[0]), float(command[1]), 0.)
                context.advance(plan.epoch, motion.tick)
                frame = context.measure(plan.epoch)
        finally:
            context.actions.stop(plan.epoch)
        return Result("move_for", "OK", "Timed movement completed and base stopped",
                      {"direction": plan.direction, "requested_duration_s": plan.duration_s,
                       "elapsed_sim_s": frame.time - start_time, "speed_m_s": plan.speed_m_s,
                       "actual_distance_m": float(np.linalg.norm(frame.base_pose[:2] - plan.start_xy)),
                       "start_xy": plan.start_xy.tolist(), "end_xy": frame.base_pose[:2].tolist()})


@dataclass(frozen=True)
class GesturePlan:
    epoch: int
    side: str
    start: np.ndarray
    poses: tuple[tuple[str, np.ndarray, float | None], ...]


class GestureSkill:
    FAILURES = ("HAND_OCCUPIED", "ARM_NOT_READY", "ARM_NOT_REACHED")
    TOLERANCE = 0.08
    STAGE_TIMEOUT = 4.0

    def __init__(self, name):
        self.name = name
        self.NEEDS = ("arm_target", "advance") if name == "wave" else ("arm_target", "grip", "advance")
        self.PROMPT = ("wave: Simulated right-hand wave while standing still; no arguments. "
                       "Use for 挥手 or 挥挥手. "
                       "Raises and swings the empty arm, then returns it to rest."
                       if name == "wave" else
                       "handshake: Simulated right-hand handshake gesture while standing still; no person contact. "
                       "Use for 握手 or 握个手. Extends the empty hand, makes two small shakes, "
                       "then returns it to rest. No arguments.")

    def prepare(self, arguments, selected, current, planning):
        if arguments:
            raise SkillFailure("INVALID_ARGUMENT", "This greeting takes no arguments")
        side = "right"
        if planning.carried.get(side) or current.hands[side].closure > planning.settings.placement.open_closure_limit:
            raise SkillFailure("HAND_OCCUPIED", "Right hand must be empty and open")
        start = current.arm_joints(side).copy()
        rest = planning.settings.placement.rest_arm(side)
        if np.max(np.abs(start - rest)) > 0.5:
            raise SkillFailure("ARM_NOT_READY", "Right arm must be near its rest pose")
        if self.name == "wave":
            raised = np.array([-1.05, -0.55, 0.0, 1.2, 0.0, 0.0, 0.0])
            outward, inward = raised.copy(), raised.copy()
            outward[2], outward[4] = 0.25, 0.45
            inward[2], inward[4] = -0.25, -0.45
            poses = (("Raise right hand", raised, None),
                     ("Wave outward", outward, None), ("Wave inward", inward, None),
                     ("Wave outward", outward, None), ("Wave inward", inward, None),
                     ("Lower right hand", start, None))
        else:
            extended = np.array([-0.75, -0.24, 0.0, 0.42, 0.0, 0.0, 0.0])
            up, down = extended.copy(), extended.copy()
            up[0], down[0] = -0.89, -0.61
            poses = (("Extend open right hand", extended, 0.0),
                     ("Greet with right hand", up, 0.25),
                     ("Shake downward", down, None), ("Shake upward", up, None),
                     ("Shake downward", down, None),
                     ("Open and lower right hand", start, 0.0))
        arm = planning.arms[side]
        for _, joints, _ in poses:
            if np.any(joints < arm.lower) or np.any(joints > arm.upper):
                raise SkillFailure("ARM_NOT_READY", "Greeting pose exceeds right-arm limits")
        return GesturePlan(current.epoch, side, start, poses)

    def execute(self, plan, context):
        for stage, joints, closure in plan.poses:
            context.stage(stage)
            if closure is not None:
                context.actions.grip(plan.epoch, plan.side, closure)
            self._reach(plan, context, joints)
        if self.name == "handshake":
            context.actions.grip(plan.epoch, plan.side, 0.0)
            context.advance(plan.epoch, 0.2)
        return Result(self.name, "OK", f"Simulated {self.name} finished; right arm returned to rest",
                      {"simulation_only": True, "side": plan.side})

    def abort(self, plan, context):
        if self.name == "handshake":
            context.actions.grip(plan.epoch, plan.side, 0.0)
        context.stage("Returning right arm to rest")
        self._reach(plan, context, plan.start)

    def _reach(self, plan, context, target):
        frame = context.measure(plan.epoch)
        deadline = frame.time + self.STAGE_TIMEOUT
        while np.max(np.abs(frame.arm_joints(plan.side) - target)) > self.TOLERANCE:
            if frame.time >= deadline:
                raise SkillFailure("ARM_NOT_REACHED", "Right arm did not reach the greeting pose")
            context.actions.arm_target(plan.epoch, plan.side, target)
            context.advance(plan.epoch)
            frame = context.measure(plan.epoch)


def registry():
    skills = {name: declared(name, PlaceSkill(name, label)) for name, label in PLACES}
    skills["move_for"] = declared("move_for", MoveForSkill())
    skills["navigate"] = declared("navigate", navigate_sim)
    for name in ("wave", "handshake"):
        skills[name] = declared(name, GestureSkill(name))
    return skills


class OfficeSession(Session):
    """Route map clicks through the normal session skill runner without an LLM round trip."""

    _click_goal = None
    _single_move = False

    def run_task(self, task):
        if not task.startswith(MAP_TASK_PREFIX):
            self._single_move = single_timed_move(task)
            try:
                return super().run_task(task)
            finally:
                self._single_move = False
        goal = json.loads(task[len(MAP_TASK_PREFIX):])
        if (not isinstance(goal, list) or len(goal) != 2 or
                any(type(value) not in (int, float) or not math.isfinite(value) for value in goal)):
            raise ValueError("Invalid map click coordinates")
        self._click_goal = goal
        try:
            return super().run_task(f"地图点击导航到 ({goal[0]:.2f}, {goal[1]:.2f}) m")
        finally:
            self._click_goal = None

    def _decision_context(self, task, names, attempt):
        if (self._single_move and attempt.name != "step-000" and self.history
                and self.history[-1].get("result", {}).get("skill") == "move_for"):
            frame = self.observed.snapshot()
            result = self.history[-1]["result"]
            summary = ("已按指令完成一次定时移动。" if result["code"] == "OK"
                       else f"定时移动未完成：{result['code']}")
            return frame, Decision(summary, "done", {"summary": summary})
        if self._click_goal is None:
            return super()._decision_context(task, names, attempt)
        frame = self.observed.snapshot()
        if attempt.name == "step-000":
            return frame, Decision("正在规划地图点击目标", "navigate",
                                   {"goal_xy_m": self._click_goal, "snap": False})
        code = self.history[-1]["result"]["code"]
        summary = "地图点击导航完成" if code == "OK" else f"地图点击目标未到达：{code}"
        return frame, Decision(summary, "done", {"summary": summary})


def clicked_floor(ray_origin, ray_direction, navigation, free):
    """Map a 3D scene click onto the z=0 office floor; reject walls and off-map rays."""
    origin, direction = np.asarray(ray_origin, float), np.asarray(ray_direction, float)
    if (origin.shape != (3,) or direction.shape != (3,) or
            not np.isfinite(origin).all() or not np.isfinite(direction).all() or
            abs(direction[2]) < 1e-6):
        return None
    distance = -origin[2] / direction[2]
    if distance <= 0:
        return None
    goal = origin[:2] + distance * direction[:2]
    return goal if cell_free(free, pixel(navigation, goal)) else None


def map_image(navigation):
    pixels = np.where(navigation.occupied[..., None], [47, 62, 77], [230, 236, 241]).astype("uint8")
    inverse = np.linalg.inv(navigation.T_map_px)
    marks = []
    for label, xy in visible_marks(ARTIFACTS,waypoints(navigation)).items():
        point = inverse @ np.r_[xy, 1.0]
        col, row = point[:2] / point[2]
        marks.append((label, col, row))
    left = max(0, int(min(col for _, col, _ in marks) - 30))
    right = min(pixels.shape[1], int(max(col for _, col, _ in marks) + 31))
    top = max(0, int(min(row for _, _, row in marks) - 30))
    bottom = min(pixels.shape[0], int(max(row for _, _, row in marks) + 31))
    scale = 4
    image = Image.fromarray(pixels).crop((left, top, right, bottom)).resize(
        ((right - left) * scale, (bottom - top) * scale), Image.Resampling.NEAREST)
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(FONT, 23)
    for label, col, row in marks:
        x, y = int((col - left + 0.5) * scale), int((row - top + 0.5) * scale)
        color = "#e69a2e" if label == "门口" else "#338e6a"
        draw.ellipse((x - 8, y - 8, x + 8, y + 8), fill=color, outline="#172b30", width=2)
        draw.text((x + 10, y - 23), label, font=font, fill="#142633", stroke_width=2,
                  stroke_fill="#eef3f6")
    return np.asarray(image)


def quick_actions(ui):
    ui.server.gui.add_html(section("办公室技能 · 跟随运行模式"))
    navigation = ui.robot.navigation
    safe = free_space(navigation, ui.robot.settings.motion.robot_radius)
    ui.server.gui.add_image(map_image(navigation), label="办公室地点 · 桌子 / 架子 / 窗口", format="jpeg")
    ui.server.gui.add_html(note("点击下方“选择目标”，再点右侧场景地面一次即可直接规划并前往；"
                                "自然语言也支持“往北走两秒”“向东南走两秒”等方位定时移动。"
                                "真机模式生成任务预览；仿真模式执行 MuJoCo。"))
    click_status = ui.server.gui.add_html(note("地图点击导航已就绪；先按“选择目标”启用一次选点。"))
    topdown = ui.server.gui.add_button("选择目标 · 俯视地图")
    perspective = ui.server.gui.add_button("返回 3D 视角")
    click_mode = {"armed_at": None}

    def show_topdown(_event):
        if ui.real_workflow.real_mode:
            ui.real_workflow.top_view();ui.real_workflow.click_at=time.monotonic()
            click_status.content=note('点击现场地图空地，预览真机路线。')
            return
        for client in ui.server.get_clients().values():
            with client.atomic():
                client.camera.up_direction = (0., 1., 0.)
                client.camera.position = (-0.4, -0.8, 8.0)
                client.camera.look_at = (-0.4, -0.8, 0.)
        click_mode["armed_at"] = time.monotonic()
        click_status.content = note("已进入一次性选点模式；点击右侧场景中的空地前往（30 秒内）。")

    topdown.on_click(show_topdown)

    def show_perspective(_event):
        click_mode["armed_at"] = None
        ui.scene_view.reset_camera()
        click_status.content = note("已退出选点模式。")

    perspective.on_click(show_perspective)

    @ui.server.scene.on_click()
    def navigate_click(event):
        if ui.real_workflow.real_mode:return
        armed_at = click_mode["armed_at"]
        if armed_at is None:
            return
        elapsed = time.monotonic() - armed_at
        if elapsed < 0.75:  # The GUI button click can propagate to the scene canvas.
            return
        if elapsed > 30:
            click_mode["armed_at"] = None
            click_status.content = note("选点已超时；请重新按“选择目标”。")
            return
        if ui.submit_button.disabled:
            click_status.content = note("任务正在执行；请等待完成或先按停止。")
            return
        goal = clicked_floor(event.ray_origin, event.ray_direction, navigation, safe)
        if goal is None:
            click_status.content = note("此处是墙体、地图外或没有安全净空；请点击长方形房间内的空地。")
            return
        click_mode["armed_at"] = None
        ui.submit_button.disabled = True
        click_status.content = note(f"正在前往点击位置：x={goal[0]:.2f}, y={goal[1]:.2f} m")
        ui.server.scene.add_icosphere("/office-click-goal", radius=0.12,
                                      color=(235, 151, 24), position=(float(goal[0]), float(goal[1]), 0.1))
        ui.controls.submit(MAP_TASK_PREFIX + json.dumps(goal.tolist()))

    with ui.server.gui.add_folder("地点导航 + 招手 / 握手", expand_by_default=True):
        for name, label in (*((n,l) for n,l in PLACES if l in ('门口',*LANDMARK_PLACES.values())), ("wave", "招手"), ("handshake", "握手")):
            button = ui.server.gui.add_button(label)
            def clicked(_event, skill=name):
                if ui._task_active or ui.submit_button.disabled:
                    return
                ui.task.value = f"请调用 {skill} 一次，完成后调用 done。不要调用其他技能。"
                ui._start(None)
            button.on_click(clicked)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--task", default="")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    if args.headless and not args.task.strip():
        parser.error("--headless needs --task")
    settings = Settings.load(ROOT / "configs/simulation.toml", scene=SCENE)
    settings = replace(settings, agent=replace(settings.agent, model=os.environ.get("HB_MODEL","astra-api"), max_steps=12,
                                                timeout=90.0))
    recorder = Recorder(ROOT, settings, ARTIFACTS)
    backend = ui = workflow = None
    close_live_panel = None
    try:
        backend = MujocoBackend(ROOT, settings, scene=SCENE)
        arms = {side: Arm(ROOT / "assets/robot/g1_29dof_with_hand.urdf", side)
                for side in ("left", "right")}
        calibration = json.loads((ROOT / "assets/robot/calibration.json").read_text())
        controls = OfficeControls()
        scene_update = None
        if not args.headless:
            import homebody.ui.console as console_module
            console_module.open_server=open_office_server
            ui = Console(controls, host="127.0.0.1", port=args.port, initial_task="",
                         scene_name="办公室九点导航", model_name=model_label(settings.agent),
                         topdown_settings=settings.topdown, render_in_background=True)
            ui.task.label = "自然语言指令（例：先去西南，挥挥手，再到门口）"
            ui.submit_button.label = "执行任务"
            ui.stop_button.label = "停止任务"
            ui.robot = backend
            ui.scene_view = SceneView(ui.server, backend, include_group3=True)
            ui.scene_view.home = ((3.8, -7.0, 6.0), (-0.4, -0.8, 0.5))
            ui.scene_view.reset_camera()
            workflow=RealWorkflow(ui,ARTIFACTS,make(settings.agent),settings.motion.drive_speed)
            ui.real_workflow=controls.real_workflow=workflow
            scene_update=lambda:ui.scene_view.update() if not workflow.real_mode else None
            quick_actions(ui)
        session = OfficeSession(backend, make(settings.agent), settings, recorder, controls,
                          arms=arms, jaw_samples=calibration["jaw_samples"], ui=ui,
                          observer_frame=backend.observer_frame if args.headless else None,
                          shoulder_frame=backend.shoulder_frame if args.headless else None,
                          scene_update=scene_update, real_time=ui is not None,
                          semantic_context=semantic_prior(SCENE), registry=registry())
        print(f"Evidence: {recorder.path}", flush=True)
        if args.headless:
            outcome = session.run_task(args.task)
            print(json.dumps(outcome, ensure_ascii=False), flush=True)
            return 0 if outcome["status"] == "model_done" else 1
        ui.update("Ready")
        print(f"OFFICE_AGENT_READY: http://127.0.0.1:{args.port}", flush=True)
        session.run()
        return 0
    finally:
        if workflow is not None:workflow.close()
        if close_live_panel is not None:
            close_live_panel()
        if backend is not None:
            backend.close()
        if ui is not None:
            ui.close()
        recorder.close()


if __name__ == "__main__":
    raise SystemExit(main())
