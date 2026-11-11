"""Display file-based real-robot observations; contains no control callbacks."""
import hb_config
import html
import json
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def display_state(state, now, expected_session=None):
    if expected_session is not None and state.get('session_id') != expected_session:
        return False
    if state.get('status') != 'live' or not state.get('readonly'):
        return False
    limits = {'updated_unix_s': 3, 'cloud_received_unix_s': 3,
              'odometry_received_unix_s': 3, 'geometry_checked_unix_s': 6}
    for key, limit in limits.items():
        age = now - float(state.get(key) or 0)
        if not np.isfinite(age) or age < -1 or age > limit:
            return False
    pose = state.get('pose') or {}
    try:
        values = np.asarray([*pose['xyz_m'], pose['yaw_degrees']], float)
        return values.shape == (4,) and np.isfinite(values).all()
    except (KeyError, TypeError, ValueError):
        return False


def attach(ui, artifacts):
    live = artifacts / 'sensors/runtime/live-readonly/live-state.json'
    points = json.loads((artifacts / 'office-waypoints.json').read_text())
    inverse = np.linalg.inv(np.asarray(points['T_map_px'], float))
    pgm = Path(hb_config.expand('${HB_OFFICE_ROOT}/office-cloud-slice/map.pgm'))
    base = Image.open(pgm).convert('RGB')
    marks = [(label, (inverse @ np.r_[xy, 1])[:2]) for label, xy in points['points'].items()]
    coords = np.asarray([xy for _, xy in marks])
    left, top = np.maximum(np.floor(coords.min(axis=0) - 34), 0).astype(int)
    right, bottom = np.minimum(np.ceil(coords.max(axis=0) + 35), base.size).astype(int)
    scale = 4
    base = base.crop((left, top, right, bottom)).resize(
        ((right-left)*scale, (bottom-top)*scale), Image.Resampling.NEAREST)
    font = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 20)

    def pixel(xy):
        return (np.asarray(xy) - [left, top]) * scale

    draw = ImageDraw.Draw(base)
    for label, xy in marks:
        x, y = pixel(xy)
        draw.ellipse((x-6, y-6, x+6, y+6), fill='#d38a28' if label == '门口' else '#368866')
        draw.text((x+9, y-23), label, font=font, fill='#243647', stroke_width=2, stroke_fill='white')
    draw.text((12, 10), '上北 · 下南 · 左西 · 右东', font=font, fill='#243647',
              stroke_width=2, stroke_fill='white')
    with ui.server.gui.add_folder('G1 实时位置 · 只读', expand_by_default=True):
        status = ui.server.gui.add_html('<p>等待实时定位数据…</p>')
        image = ui.server.gui.add_image(np.asarray(base), label='历史办公室地图 · 定位候选', format='jpeg')
        ui.server.gui.add_html('<p style="font-size:12px">箭头表示 FAST-LIO body 的位置与朝向候选。'
                              '本面板仅显示位置；下方技能和右侧 3D 场景仍操作仿真。</p>')
    stop = threading.Event()

    def update():
        while not stop.is_set():
            try:
                state = json.loads(live.read_text())
                now = time.time()
                session = (live.parent / 'session-id').read_text().strip()
                fresh = display_state(state, now, session)
                message = str(state.get('message', '等待定位'))
                frame = base.copy()
                color = '#238767' if fresh else '#a16c24'
                detail = ''
                if fresh:
                    pose = state['pose']
                    x, y = pixel((inverse @ np.r_[pose['xyz_m'][:2], 1])[:2])
                    yaw = np.deg2rad(pose['yaw_degrees'])
                    # Map y increases upward, image rows increase downward.
                    forward = np.array([np.cos(yaw), -np.sin(yaw)])
                    side = np.array([-forward[1], forward[0]])
                    center = np.array([x, y])
                    triangle = [tuple(center + forward*22), tuple(center - forward*14 + side*12),
                                tuple(center - forward*14 - side*12)]
                    draw = ImageDraw.Draw(frame)
                    draw.ellipse((x-25, y-25, x+25, y+25), outline='#2476d1', width=3)
                    draw.polygon(triangle, fill='#2476d1', outline='white')
                    age = now - state['odometry_received_unix_s']
                    detail = (f'<br>x={pose["xyz_m"][0]:.2f} m，y={pose["xyz_m"][1]:.2f} m；'
                              f'朝向 {pose["yaw_degrees"]:.1f}°（0°为东）'
                              f'<br>数据接收距今 {age:.1f} 秒；'
                              f'点云 {state["cloud_messages"]} 帧 / 里程计 {state["odometry_messages"]} 条')
                elif state.get('status') == 'live':
                    message = '实时数据已过期，等待点云、里程计和地图校验恢复'
                status.content = f'<p style="color:{color}">{html.escape(message)}{detail}</p>'
                image.image = np.asarray(frame)
            except (OSError, ValueError, KeyError, TypeError):
                status.content = '<p>等待只读定位服务的数据…</p>'
                image.image = np.asarray(base)
            stop.wait(1)

    thread = threading.Thread(target=update, name='office-readonly-location-ui', daemon=True)
    thread.start()

    def close():
        stop.set()
        thread.join(timeout=2)
    return close
