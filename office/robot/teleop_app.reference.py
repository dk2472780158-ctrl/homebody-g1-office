"""G1 teleop-proxy：键鼠粗粒度遥控 → 腿速（单 vel owner）。

TELEOP_BACKEND=ros2（默认）：rclpy `/cmd_vel` → g1-cmd-vel-bridge → sport 7105。
TELEOP_BACKEND=ros1（回滚）：rospy /cmd_vel → abotclaw_vel → LocoClient.Move。

使能：acquire vel_owner=teleop →（ros1 时停 vel）→ Agent env.stand() → armed
急停：零速 → Agent env.estop_legs()（Squat FSM2 → 确认 → Damp；禁止 Sit）→ release owner
"""

from __future__ import annotations

import http.client
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from typing import Any, Optional

from fastapi import FastAPI, Header, HTTPException
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

TOKEN = os.environ["TELEOP_TOKEN"]
MAX_VX = float(os.environ.get("TELEOP_MAX_VX", "0.35"))
MAX_VY = float(os.environ.get("TELEOP_MAX_VY", "0.25"))
MAX_WZ = float(os.environ.get("TELEOP_MAX_WZ", "0.45"))
DEADMAN_S = float(os.environ.get("TELEOP_DEADMAN_S", "1.0"))
REQUIRE_ARM = os.environ.get("TELEOP_REQUIRE_ARM", "1") == "1"
AUTO_STAND = os.environ.get("TELEOP_AUTO_STAND", "1") == "1"
AUTO_DAMP = os.environ.get("TELEOP_AUTO_DAMP", "1") == "1"
ARM_DEADMAN_S = float(os.environ.get("TELEOP_ARM_DEADMAN_S", "5.0"))
BMS_STALE_S = float(os.environ.get("TELEOP_BMS_STALE_S", "5.0"))
BMS_WARN_PCT = int(os.environ.get("TELEOP_BMS_WARN_PCT", "30"))
BMS_CRIT_PCT = int(os.environ.get("TELEOP_BMS_CRIT_PCT", "20"))
VEL_CONTAINER = os.environ.get("TELEOP_VEL_CONTAINER", "abotclaw_vel")
G1_IFACE = os.environ.get("G1_NETWORK_INTERFACE", "eth0")
TELEOP_BACKEND = os.environ.get("TELEOP_BACKEND", "ros2").strip().lower() or "ros2"
# ros2 腿速：rclpy（默认）或 udp（回滚，需 bridge 内 udp_in）
TELEOP_ROS2_CMD_VEL = os.environ.get("TELEOP_ROS2_CMD_VEL", "rclpy").strip().lower() or "rclpy"
CMD_VEL_UDP_PORT = int(os.environ.get("TELEOP_CMD_VEL_UDP_PORT", "21510"))
VEL_OWNER_FORCE = os.environ.get("TELEOP_VEL_OWNER_FORCE", "0") == "1"
# official 只读感知（2026-09-28）：TELEOP_READ_ONLY=1 时全部写口 POST 硬 403，GET 状态保留
READ_ONLY = os.environ.get("TELEOP_READ_ONLY", "0") == "1"
# 电量联锁（2026-09-30）：soc < crit 或电池数据缺失 → 写口 403（estop 例外）；fail-closed
SOC_CRIT_PCT = int(os.environ.get("TELEOP_SOC_CRIT_PCT", "20"))
# stand/estop：loco_ros2 → /api/sport/*（与 cmd_vel bridge 同栈；无 ChannelFactory）。
# TELEOP_LOCO_ROS2=0 回滚 Agent execute。
TELEOP_LOCO_ROS2 = os.environ.get("TELEOP_LOCO_ROS2", "1") == "1"

_HERE = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(_HERE, "static")
# 与 playground /g1/ 同款驾驶舱（deploy_pc2_lan_ui.sh → ui/）
UI_DIR = os.environ.get("TELEOP_UI_DIR") or os.path.join(_HERE, "ui")

# 手臂预设：子进程写 rt/arm_sdk（实测 FSM200 下关节会动）。
# 不经 Agent :8888——主进程 rclpy Domain 与 ChannelFactory 冲突，故隔离进程。
# ArmActionClient 需 FSM{500,501,801}（本机切不进→7404）；
# LocoClient.SetTaskId 返回 0 但关节无位移（本机固件无效）。
ARMS_ACTIONS: dict[str, str] = {
    "wave": "wave",
    "look left": "look_left",
    "look right": "look_right",
}
ARMS_ACTION_TIMEOUT_S = float(os.environ.get("TELEOP_ARMS_ACTION_TIMEOUT_S", "60"))
GO2RTC_EMBED = os.environ.get(
    "GO2RTC_EMBED_URL",
    "http://localhost:1984/stream.html?src=g1_videohub_front&mode=webrtc",
)
AGENT_BASE = os.environ.get("AGENT_BASE_URL", "http://127.0.0.1:8888").rstrip("/")
DOCKER_SOCK = os.environ.get("DOCKER_SOCK", "/var/run/docker.sock")
# 地图安全距离（与 g1-sim 同 walls.json）；需定位位姿才生效
MAP_SAFETY = os.environ.get("TELEOP_MAP_SAFETY", "1").strip().lower() not in (
    "0",
    "off",
    "false",
    "no",
)
MAP_POSE_TOPIC = os.environ.get("TELEOP_MAP_POSE_TOPIC", "").strip()
MAP_POSE_STALE_S = float(os.environ.get("TELEOP_MAP_POSE_STALE_S", "0.8"))

app = FastAPI(title="g1-teleop-proxy", version="0.3.0-t1")


class CmdVel(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class ArmBody(BaseModel):
    armed: bool = False


_lock = threading.Lock()
_armed = False
_busy = False  # stand/damp 进行中
_last_cmd = CmdVel()
_last_beat = 0.0
_pub = None
_ros_ok = False
_cmd_sock: socket.socket | None = None
_ros2_node = None
_ros2_pub = None
_mode_note = "idle"
_cached_loco_fsm: int | None = None
_LOCO_SKIP_STAND_FSMS = frozenset({801, 200, 500, 501})
# 手臂通道（与腿 armed 独立）
_arms_armed = False
_arms_beat = 0.0
_arms_note = "arms idle"
_arms_busy = False
# 电池（rt/lf/bmsstate 或 rt/bmsstate）
_bms_lock = threading.Lock()
_bms: dict[str, Any] = {
    "ok": False,
    "soc": None,
    "soh": None,
    "current_a": None,
    "voltage_v": None,
    "cycle": None,
    "topic": None,
    "note": "bms not started",
    "ts": 0.0,
}

# 地图 keepout（安全距离）；位姿来自可选 ROS odom
_map_keepout = None
_map_pose_lock = threading.Lock()
_map_pose: dict[str, Any] = {
    "ok": False,
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0,
    "ts": 0.0,
    "topic": None,
    "note": "map pose idle",
}


class ArmsEnableBody(BaseModel):
    armed: bool = False


class ArmsActionBody(BaseModel):
    name: str


def _clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def _to_bridge_twist(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    """g1_cmd_vel_bridge 期望归一化 [-1,1]×max_*；teleop 内部存 m/s。"""
    if TELEOP_BACKEND != "ros2":
        return vx, vy, wz

    def _norm(v: float, lim: float) -> float:
        if lim <= 0:
            return 0.0
        return _clamp(v / lim, 1.0)

    return _norm(vx, MAX_VX), _norm(vy, MAX_VY), _norm(wz, MAX_WZ)


def _auth(authorization: Optional[str], x_teleop_token: Optional[str]) -> None:
    got = None
    if x_teleop_token:
        got = x_teleop_token.strip()
    elif authorization and authorization.lower().startswith("bearer "):
        got = authorization[7:].strip()
    if got != TOKEN:
        raise HTTPException(status_code=401, detail="invalid teleop token")



def _battery_block_reason() -> str | None:
    """电量联锁：soc 缺失（fail-closed）或低于 crit → 返回阻断原因，否则 None。"""
    with _bms_lock:
        ok = bool(_bms.get("ok"))
        soc = _bms.get("soc")
    if not ok or soc is None:
        return "battery state unknown (fail-closed)"
    try:
        if int(soc) < SOC_CRIT_PCT:
            return f"battery critical soc={soc}% (<{SOC_CRIT_PCT}%)"
    except (TypeError, ValueError):
        return "battery state invalid (fail-closed)"
    return None


def _write_blocked_reason() -> str | None:
    if READ_ONLY:
        return "teleop read-only (official mode)"
    return _battery_block_reason()


def _write_guard(allow_low_battery: bool = False) -> None:
    """official 只读感知 + 电量联锁：写口硬 403。estop 传 allow_low_battery=True 放行电量闸。"""
    if READ_ONLY:
        raise HTTPException(status_code=403, detail="teleop read-only (official mode)")
    if not allow_low_battery:
        reason = _battery_block_reason()
        if reason:
            raise HTTPException(status_code=403, detail=f"teleop write blocked: {reason}")

def _init_ros2_rclpy() -> None:
    """Humble rclpy 直发 /cmd_vel（归一化 twist，与 bridge 标定一致）。"""
    global _ros2_node, _ros2_pub, _ros_ok
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    if not rclpy.ok():
        rclpy.init()
    node = Node("g1_teleop_proxy")
    # READ_ONLY（official 观摩）不能占 /cmd_vel：不创建 publisher，也不跑 _loop 零速循环。
    pub = None
    if not READ_ONLY:
        pub = node.create_publisher(Twist, "/cmd_vel", QoSProfile(depth=1))

    def _spin() -> None:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except Exception:
                break

    threading.Thread(target=_spin, daemon=True, name="ros2-spin").start()
    _ros2_node = node
    _ros2_pub = pub
    _ros_ok = True
    try:
        from joint_state import get_joint_hub

        get_joint_hub().attach_ros2(node)
    except Exception as e:
        print(f"[teleop] joint ros2 attach: {e}", flush=True)
    try:
        _attach_bms_ros2(node)
    except Exception as e:
        print(f"[teleop] bms ros2 attach: {e}", flush=True)
    try:
        import loco_ros2

        loco_ros2.attach_ros2(node)
    except Exception as e:
        print(f"[teleop] loco_ros2 sport attach: {e}", flush=True)
    if READ_ONLY:
        print("[teleop] ros2 backend: rclpy telemetry ready (READ_ONLY: no /cmd_vel publisher)", flush=True)
    else:
        print("[teleop] ros2 backend: rclpy /cmd_vel publisher ready", flush=True)


def _attach_bms_ros2(node: Any) -> None:
    """Humble：订 /lf/bmsstate（与 joint 共用 rclpy spin，勿走 unitree_sdk2py DDS）。"""
    from rclpy.qos import qos_profile_sensor_data
    from unitree_hg.msg import BmsState

    def _on(msg: BmsState) -> None:
        global _bms
        try:
            soc = int(msg.soc)
            soh = int(msg.soh)
            current_a = float(msg.current) / 1000.0
            vv = None
            try:
                raw = int(msg.bmsvoltage[0])
                vv = float(raw) / 1000.0 if raw > 100 else float(raw)
            except Exception:
                vv = None
            with _bms_lock:
                _bms.update(
                    {
                        "ok": True,
                        "soc": soc,
                        "soh": soh,
                        "current_a": round(current_a, 3),
                        "voltage_v": None if vv is None else round(vv, 2),
                        "cycle": int(msg.cycle),
                        "topic": "ros2:/lf/bmsstate",
                        "note": "ok",
                        "ts": time.time(),
                    }
                )
        except Exception as e:
            print(f"[teleop] bms ros2 handler: {e}", flush=True)

    node.create_subscription(BmsState, "/lf/bmsstate", _on, qos_profile_sensor_data)
    with _bms_lock:
        _bms["note"] = "ros2:/lf/bmsstate, waiting…"
    print("[teleop] bms subscribed ros2:/lf/bmsstate", flush=True)


def _init_ros() -> None:
    global _pub, _ros_ok, _map_keepout, _cmd_sock
    if TELEOP_BACKEND == "ros2":
        if TELEOP_ROS2_CMD_VEL == "udp":
            try:
                _cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                _ros_ok = True
                print(
                    f"[teleop] ros2 backend: UDP cmd_vel → 127.0.0.1:{CMD_VEL_UDP_PORT}",
                    flush=True,
                )
            except OSError as e:
                _ros_ok = False
                print(f"[teleop] ros2 UDP sock failed: {e}", flush=True)
        else:
            try:
                _init_ros2_rclpy()
            except Exception as e:
                _ros_ok = False
                print(f"[teleop] ros2 rclpy init failed: {e}", flush=True)
        return
    try:
        import rospy
        from geometry_msgs.msg import Twist

        if not rospy.core.is_initialized():
            rospy.init_node("g1_teleop_proxy", anonymous=True, disable_signals=True)
        _pub = None
        if not READ_ONLY:
            _pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        _ros_ok = True
        if READ_ONLY:
            print("[teleop] rospy telemetry ready (READ_ONLY: no /cmd_vel publisher)", flush=True)
        else:
            print("[teleop] rospy /cmd_vel publisher ready (TELEOP_BACKEND=ros1)", flush=True)
        if MAP_SAFETY:
            try:
                from map_keepout import try_load_walls

                _map_keepout = try_load_walls()
                if _map_keepout:
                    print(
                        f"[teleop] map keepout loaded boxes={_map_keepout.info()['boxes']} "
                        f"r={_map_keepout.radius_m:.2f}m",
                        flush=True,
                    )
                else:
                    print("[teleop] map keepout: walls.json missing", flush=True)
            except Exception as e:
                print(f"[teleop] map keepout load failed: {e}", flush=True)
            if MAP_POSE_TOPIC:
                _init_map_pose_sub(MAP_POSE_TOPIC)
            else:
                with _map_pose_lock:
                    _map_pose["note"] = (
                        "set TELEOP_MAP_POSE_TOPIC to enable live keepout "
                        "(nav_msgs/Odometry in map frame)"
                    )
                print(
                    "[teleop] map keepout armed but no TELEOP_MAP_POSE_TOPIC — "
                    "cmd_vel not filtered until pose",
                    flush=True,
                )
    except Exception as e:
        _ros_ok = False
        print(f"[teleop] rospy unavailable: {e}", flush=True)


def _init_map_pose_sub(topic: str) -> None:
    """Subscribe nav_msgs/Odometry (map or odom frame — caller must be map-aligned)."""
    import math

    import rospy
    from nav_msgs.msg import Odometry

    def _cb(msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw from quaternion (z-up)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with _map_pose_lock:
            _map_pose.update(
                {
                    "ok": True,
                    "x": float(p.x),
                    "y": float(p.y),
                    "yaw": float(yaw),
                    "ts": time.time(),
                    "topic": topic,
                    "note": "ok",
                }
            )

    rospy.Subscriber(topic, Odometry, _cb, queue_size=1)
    with _map_pose_lock:
        _map_pose["topic"] = topic
        _map_pose["note"] = f"subscribed {topic}"
    print(f"[teleop] map pose sub → {topic}", flush=True)


def _filter_cmd_for_map(vx: float, vy: float, wz: float) -> tuple[float, float, float, bool]:
    """Apply keepout if walls + fresh map pose available."""
    if not MAP_SAFETY or _map_keepout is None:
        return vx, vy, wz, False
    with _map_pose_lock:
        pose = dict(_map_pose)
    if not pose.get("ok") or (time.time() - float(pose.get("ts") or 0)) > MAP_POSE_STALE_S:
        return vx, vy, wz, False
    return _map_keepout.filter_twist(
        float(pose["x"]),
        float(pose["y"]),
        float(pose["yaw"]),
        vx,
        vy,
        wz,
    )


def _publish(vx: float, vy: float, wz: float) -> None:
    if READ_ONLY or not _ros_ok:
        return
    if TELEOP_BACKEND == "ros2":
        bx, by, bw = _to_bridge_twist(vx, vy, wz)
        if TELEOP_ROS2_CMD_VEL == "udp":
            if _cmd_sock is None:
                return
            try:
                _cmd_sock.sendto(
                    struct.pack("<fff", float(bx), float(by), float(bw)),
                    ("127.0.0.1", CMD_VEL_UDP_PORT),
                )
            except OSError:
                pass
            return
        if _ros2_pub is None:
            return
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = float(bx)
        msg.linear.y = float(by)
        msg.angular.z = float(bw)
        _ros2_pub.publish(msg)
        return
    if _pub is None:
        return
    from geometry_msgs.msg import Twist

    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    _pub.publish(msg)


def _loop() -> None:
    while True:
        with _lock:
            armed = _armed
            busy = _busy
            age = time.time() - _last_beat
            cmd = _last_cmd
        if busy or not armed or age > DEADMAN_S:
            _publish(0.0, 0.0, 0.0)
        else:
            vx, vy, wz, _clipped = _filter_cmd_for_map(
                _clamp(cmd.vx, MAX_VX),
                _clamp(cmd.vy, MAX_VY),
                _clamp(cmd.wz, MAX_WZ),
            )
            _publish(vx, vy, wz)
        time.sleep(0.05)


# ----- Docker pause vel (stand/damp 时避免 Move(0) 抢 FSM) -----


def _docker(method: str, path: str, timeout: float = 60.0) -> tuple[int, bytes]:
    class _UnixHTTPConnection(http.client.HTTPConnection):
        def connect(self) -> None:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(DOCKER_SOCK)

    conn = _UnixHTTPConnection("localhost")
    conn.timeout = timeout
    try:
        conn.request(method, path)
        resp = conn.getresponse()
        body = resp.read()
        return resp.status, body
    finally:
        conn.close()


def _vel_stop() -> None:
    """ros1：停 abotclaw_vel，避免 Move(0) 抢 FSM。ros2：零速已由 _busy/_publish 保证。"""
    if TELEOP_BACKEND != "ros1":
        return
    try:
        code, _ = _docker("POST", f"/containers/{VEL_CONTAINER}/stop?t=3")
        print(f"[teleop] vel stop http={code}", flush=True)
    except Exception as e:
        print(f"[teleop] vel stop failed: {e}", flush=True)


def _vel_start() -> None:
    if TELEOP_BACKEND != "ros1":
        return
    try:
        code, _ = _docker("POST", f"/containers/{VEL_CONTAINER}/start")
        print(f"[teleop] vel start http={code}", flush=True)
        time.sleep(1.5)
    except Exception as e:
        print(f"[teleop] vel start failed: {e}", flush=True)


# ----- Agent :8888 stand / damp -----


def _agent_json(method: str, path: str, body: Optional[dict] = None, headers: Optional[dict] = None) -> Any:
    data = None if body is None else json.dumps(body).encode()
    h = {"Content-Type": "application/json"}
    if headers:
        h.update(headers)
    req = urllib.request.Request(AGENT_BASE + path, data=data, headers=h, method=method)
    with urllib.request.urlopen(req, timeout=120) as resp:
        return json.loads(resp.read().decode())


def _agent_execute(code: str, purpose: str) -> str:
    """lease + execute + wait idle；返回 stdout 尾部。"""
    acq = _agent_json("POST", "/lease/acquire", {"holder": "teleop-proxy", "purpose": purpose})
    lease = acq["lease_id"]
    try:
        _agent_json(
            "POST",
            "/code/execute",
            {"code": code},
            headers={"X-Lease-Id": lease},
        )
        t0 = time.time()
        last: dict = {}
        while time.time() - t0 < 90:
            last = _agent_json("GET", "/code/status")
            if last.get("status") == "idle":
                break
            time.sleep(0.4)
        return (last.get("stdout") or "")[-1500:]
    finally:
        try:
            _agent_json("POST", "/lease/release", {"lease_id": lease})
        except Exception:
            pass


def _parse_fsm_from_stand_log(out: str) -> int | None:
    import re

    for pat in (
        r"fsm_after['\"]?\s*,?\s*(\d+)",
        r"already FSM (\d+)",
        r"already loco ready FSM (\d+)",
    ):
        m = re.search(pat, out)
        if m:
            try:
                return int(m.group(1))
            except ValueError:
                pass
    return None


def _loco_backend_name() -> str:
    return "ros2-loco" if (TELEOP_BACKEND == "ros2" and TELEOP_LOCO_ROS2) else "agent"


def _do_stand_dispatch() -> str:
    """stand 分流：ros2-loco 直连（LocoClient FSM 门禁）或 Agent 旧链。"""
    if _loco_backend_name() == "ros2-loco":
        import loco_ros2
        loco_ros2.stand(settle_s=3.0, start_retries=6)
        fsm = loco_ros2.get_fsm_id()
        return f"teleop-stand OK (ros2-loco) fsm_after {fsm}"
    return _do_stand()


def _do_estop_legs_dispatch() -> str:
    if _loco_backend_name() == "ros2-loco":
        import loco_ros2
        loco_ros2.estop_legs(squat_settle_s=5.0)
        return f"teleop-estop-legs OK (ros2-loco) fsm {loco_ros2.get_fsm_id()}"
    return _do_estop_legs()


def _do_stand() -> str:
    # 按当前 FSM 起身：已 4/200 则跳过 Damp（见 env.stand）
    code = (
        "fsm0 = env.get_fsm_id(); "
        "env.stand(settle_s=3.0, start_retries=6); "
        "print('teleop-stand OK fsm_before', fsm0, 'fsm_after', env.get_fsm_id())"
    )
    out = _agent_execute(code, "teleop-stand")
    global _cached_loco_fsm
    fsm = _parse_fsm_from_stand_log(out)
    if fsm is not None:
        _cached_loco_fsm = fsm
    return out


def _do_damp() -> str:
    """兼容旧调用；急停请走 ``_do_estop_legs``（Squat→Damp）。"""
    code = "env.damp(); print('teleop-damp OK')"
    return _agent_execute(code, "teleop-damp")


def _do_estop_legs() -> str:
    """急停腿：零速后 Squat(FSM2)→确认→Damp；禁止 Sit。"""
    code = (
        "env.estop_legs(squat_settle_s=5.0); "
        "print('teleop-estop-legs OK fsm', env.get_fsm_id())"
    )
    return _agent_execute(code, "teleop-estop-legs")


def _do_arms_action(name: str) -> str:
    """子进程跑 arms_sdk_action.py → rt/arm_sdk（不经 Agent；结束必释放 weight）。"""
    if name not in ARMS_ACTIONS:
        raise ValueError(f"action not in whitelist: {name}")
    kind = ARMS_ACTIONS[name]
    script = os.path.join(_HERE, "arms_sdk_action.py")
    env = os.environ.copy()
    env.setdefault("G1_NETWORK_INTERFACE", G1_IFACE)
    try:
        proc = subprocess.run(
            [sys.executable, script, kind],
            cwd=_HERE,
            env=env,
            capture_output=True,
            text=True,
            timeout=ARMS_ACTION_TIMEOUT_S,
            check=False,
        )
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(
            f"arms_sdk_action timeout after {ARMS_ACTION_TIMEOUT_S}s kind={kind}"
        ) from e
    out = ((proc.stdout or "") + (proc.stderr or "")).strip()
    if proc.returncode != 0:
        raise RuntimeError(
            f"arms_sdk_action rc={proc.returncode} kind={kind}: {out[-800:]}"
        )
    return out[-1500:] if out else f"arms_action {kind} ok"


def _arms_release_safe() -> None:
    """Loco 手臂任务无 hold；释放仅撤使能（无需 ArmAction 99）。"""
    global _arms_armed, _arms_note
    with _lock:
        _arms_armed = False
    _arms_note = "released"


def _arms_deadman_loop() -> None:
    global _arms_armed, _arms_note, _arms_busy
    while True:
        with _lock:
            armed = _arms_armed
            age = time.time() - _arms_beat
            busy = _arms_busy
        # 动作执行期间由动作路径自己续命；结束后先看 beat 再判超时
        if armed and not busy and age > ARM_DEADMAN_S:
            print(f"[teleop] arms deadman age={age:.2f}s → release", flush=True)
            _arms_busy = True
            try:
                _arms_release_safe()
                _arms_note = "deadman release"
            finally:
                _arms_busy = False
        time.sleep(0.1)


def _battery_snapshot() -> dict[str, Any]:
    """返回给 UI /health 的电池快照（含 stale / 阈值着色提示）。"""
    with _bms_lock:
        snap = dict(_bms)
    ts = float(snap.get("ts") or 0.0)
    age = (time.time() - ts) if ts > 0 else None
    soc = snap.get("soc")
    level = "unknown"
    if age is None or age > BMS_STALE_S or not snap.get("ok") or soc is None:
        level = "offline"
        snap["ok"] = False
        if snap.get("note") in (None, "", "ok"):
            snap["note"] = "无 BMS 数据（机器人关机/DDS 未通）"
    elif int(soc) <= BMS_CRIT_PCT:
        level = "critical"
    elif int(soc) <= BMS_WARN_PCT:
        level = "warn"
    else:
        level = "ok"
    snap["age_s"] = None if age is None else round(age, 2)
    snap["level"] = level
    snap["warn_pct"] = BMS_WARN_PCT
    snap["crit_pct"] = BMS_CRIT_PCT
    return snap


def _bms_loop() -> None:
    """订阅 G1 BMS；机器人关机时保持 offline，不阻塞主服务。"""
    global _bms
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_
    except Exception as e:
        with _bms_lock:
            _bms["note"] = f"sdk import failed: {e}"
        print(f"[teleop] bms import failed: {e}", flush=True)
        return

    try:
        ChannelFactoryInitialize(0, G1_IFACE)
    except Exception as e:
        # 可能已被其它路径初始化
        print(f"[teleop] bms ChannelFactoryInit: {type(e).__name__}: {e}", flush=True)

    def _on_bms(msg: Any) -> None:
        try:
            soc = int(msg.soc)
            soh = int(msg.soh)
            current_a = float(msg.current) / 1000.0  # mA → A（与官方示例一致时）
            # bmsvoltage[0] 常见为总线 mV
            vv = None
            try:
                raw = msg.bmsvoltage[0]
                vv = float(raw) / 1000.0 if raw > 100 else float(raw)
            except Exception:
                vv = None
            with _bms_lock:
                _bms.update(
                    {
                        "ok": True,
                        "soc": soc,
                        "soh": soh,
                        "current_a": round(current_a, 3),
                        "voltage_v": None if vv is None else round(vv, 2),
                        "cycle": int(getattr(msg, "cycle", 0) or 0),
                        "note": "ok",
                        "ts": time.time(),
                    }
                )
        except Exception as e:
            print(f"[teleop] bms handler: {e}", flush=True)

    subscribed = False
    for topic in ("rt/lf/bmsstate", "rt/bmsstate"):
        try:
            sub = ChannelSubscriber(topic, BmsState_)
            sub.Init(_on_bms, 1)
            with _bms_lock:
                _bms["topic"] = topic
                _bms["note"] = f"subscribed {topic}, waiting…"
            print(f"[teleop] bms subscribed {topic}", flush=True)
            subscribed = True
            break
        except Exception as e:
            print(f"[teleop] bms subscribe {topic} failed: {e}", flush=True)
    if not subscribed:
        with _bms_lock:
            _bms["note"] = "subscribe failed"
        return

    while True:
        time.sleep(2.0)


@app.on_event("startup")
def _startup() -> None:
    _init_ros()
    if not READ_ONLY:
        threading.Thread(target=_loop, daemon=True).start()
    threading.Thread(target=_arms_deadman_loop, daemon=True).start()
    if TELEOP_BACKEND != "ros2":
        threading.Thread(target=_bms_loop, daemon=True, name="bms").start()
    # T1：错开 DDS 初始化，避免与 bms 并发 ChannelFactory 竞态
    def _start_joint_hub() -> None:
        time.sleep(1.0)
        try:
            from joint_state import get_joint_hub

            get_joint_hub()
        except Exception as e:
            print(f"[teleop] joint hub: {e}", flush=True)

    threading.Thread(target=_start_joint_hub, daemon=True, name="joint-hub").start()
    try:
        from loco_knobs import get_loco

        get_loco()
    except Exception as e:
        print(f"[teleop] loco: {e}", flush=True)

    def _start_points_hub() -> None:
        try:
            from points_hub import get_points_hub

            get_points_hub()
        except Exception as e:
            print(f"[teleop] points hub: {e}", flush=True)

    threading.Thread(target=_start_points_hub, daemon=True, name="points-hub").start()


@app.get("/health")
def health():
    out = {
        "status": "ok",
        "read_only": READ_ONLY,
        "write_blocked_reason": _write_blocked_reason(),
        "soc_crit_pct": SOC_CRIT_PCT,
        "mode": ("official-readonly" if READ_ONLY else "prod"),
        "ros": _ros_ok,
        "teleop_backend": TELEOP_BACKEND,
        "loco_backend": _loco_backend_name(),
        "cmd_vel_transport": (
            TELEOP_ROS2_CMD_VEL if TELEOP_BACKEND == "ros2" else "rospy"
        ),
        "armed": _armed,
        "busy": _busy,
        "mode_note": _mode_note,
        "arms_armed": _arms_armed,
        "arms_note": _arms_note,
        "arms_whitelist": list(ARMS_ACTIONS.keys()),
        "auto_stand": AUTO_STAND,
        "auto_damp": AUTO_DAMP,
        "deadman_s": DEADMAN_S,
        "arm_deadman_s": ARM_DEADMAN_S,
        "limits": {"vx": MAX_VX, "vy": MAX_VY, "wz": MAX_WZ},
        "go2rtc_embed": GO2RTC_EMBED,
        "agent": AGENT_BASE,
        "battery": _battery_snapshot(),
        "t1": True,
        "map_safety": {
            "enabled": MAP_SAFETY,
            "walls": None if _map_keepout is None else _map_keepout.info(),
            "pose_topic": MAP_POSE_TOPIC or None,
            "pose": dict(_map_pose),
            "note": (
                "keepout filters cmd_vel when pose fresh"
                if MAP_SAFETY and _map_keepout and MAP_POSE_TOPIC
                else "need walls.json + TELEOP_MAP_POSE_TOPIC (map-frame Odometry)"
            ),
        },
    }
    try:
        from joint_state import get_joint_hub
        from loco_knobs import get_loco

        js = get_joint_hub().snapshot()
        out["joint"] = {
            "ok": js.get("ok"),
            "source": js.get("source"),
            "note": js.get("note"),
            "age_s": js.get("age_s"),
        }
        out["loco"] = get_loco().get().get("loco")
    except Exception as e:
        out["joint"] = {"ok": False, "note": str(e)}
    try:
        from points_hub import get_points_hub

        out["points"] = get_points_hub().health()
    except Exception as e:
        out["points"] = {"ok": False, "note": str(e)}
    try:
        from vel_owner import read_owner

        out["vel_owner"] = read_owner()
    except Exception as e:
        out["vel_owner"] = {"owner": "unknown", "note": str(e)}
    return out


@app.get("/teleop/battery")
def battery():
    """电池 SOC（DDS rt/lf/bmsstate）；机器人关机时 ok=false。"""
    return _battery_snapshot()


@app.get("/teleop/joint_state")
def joint_state(
    authorization: Optional[str] = Header(default=None),
    x_teleop_token: Optional[str] = Header(default=None, alias="X-Teleop-Token"),
):
    """T1：LowState 或 mock 关节快照（供 gateway 推送）。"""
    _auth(authorization, x_teleop_token)
    from joint_state import get_joint_hub

    return get_joint_hub().snapshot()


@app.get("/teleop/points")
def points_get(
    authorization: Optional[str] = Header(default=None),
    x_teleop_token: Optional[str] = Header(default=None, alias="X-Teleop-Token"),
):
    """P1：降采样 Livox 点云（供 gateway 推送）；TELEOP_POINTS=0 时 ok=false。"""
    _auth(authorization, x_teleop_token)
    from points_hub import get_points_hub

    return get_points_hub().snapshot()


class LocoBody(BaseModel):
    stand_height: Optional[float] = None
    swing_height: Optional[float] = None
    balance: Optional[float] = None


@app.get("/teleop/loco")
def loco_get(
    authorization: Optional[str] = Header(default=None),
    x_teleop_token: Optional[str] = Header(default=None, alias="X-Teleop-Token"),
):
    _auth(authorization, x_teleop_token)
    from loco_knobs import get_loco

    return get_loco().get()


@app.post("/teleop/loco")
def loco_set(
    body: LocoBody,
    authorization: Optional[str] = Header(default=None),
    x_teleop_token: Optional[str] = Header(default=None, alias="X-Teleop-Token"),
):
    _write_guard()
    _auth(authorization, x_teleop_token)
    from loco_knobs import get_loco

    patch = body.model_dump(exclude_none=True) if hasattr(body, "model_dump") else body.dict(exclude_none=True)
    return get_loco().set(patch)


class WaistBody(BaseModel):
    yaw: float = 0.0


@app.post("/teleop/waist")
def waist_set(
    body: WaistBody,
    authorization: Optional[str] = Header(default=None),
    x_teleop_token: Optional[str] = Header(default=None, alias="X-Teleop-Token"),
):
    """连续腰 yaw（需臂使能）。

    mock：可写 joint overlay 做可视化。
    dds 真机：默认拒绝（避免掩盖 LowState；真写路径未标定）。
    """
    _write_guard()
    _auth(authorization, x_teleop_token)
    if not _arms_armed:
        raise HTTPException(status_code=403, detail="arms not armed")
    yaw = _clamp(body.yaw, 0.55)
    from joint_state import get_joint_hub

    hub = get_joint_hub()
    snap = hub.snapshot()
    if snap.get("dds_locked") or snap.get("source") == "dds":
        raise HTTPException(
            status_code=501,
            detail="waist continuous write disabled on live LowState "
            "(use arms look left/right; refuse overlay mask)",
        )
    if not hub.set_overlay({"waist_yaw_joint": yaw}):
        raise HTTPException(status_code=501, detail="waist overlay refused")
    return {
        "ok": True,
        "waist_yaw": yaw,
        "note": "mock overlay only",
        "sim": False,
    }


class ArmsTargetBody(BaseModel):
    targets: dict[str, float] = {}


@app.post("/teleop/arms/target")
def arms_target(
    body: ArmsTargetBody,
    authorization: Optional[str] = Header(default=None),
    x_teleop_token: Optional[str] = Header(default=None, alias="X-Teleop-Token"),
):
    """连续臂目标；默认拒绝真写（防第二 DDS / 未评审 Agent 脚本）。"""
    _write_guard()
    _auth(authorization, x_teleop_token)
    from arms_continuous import handle_arms_target, sanitize_targets

    global _arms_beat
    with _lock:
        armed = _arms_armed
        if armed:
            _arms_beat = time.time()
    targets = sanitize_targets(body.targets)
    out = handle_arms_target(
        arms_armed=armed,
        targets=targets,
        agent_execute=None,
    )
    if out.get("code") == 403:
        raise HTTPException(status_code=403, detail=out.get("error") or "forbidden")
    if out.get("code") == 501:
        from fastapi.responses import JSONResponse

        return JSONResponse(out, status_code=501)
    return out


@app.get("/config.js")
def config_js():
    body = (
        "window.TELEOP_CONFIG = {"
        f"go2rtcEmbed: {GO2RTC_EMBED!r}, "
        f"requireArm: {str(REQUIRE_ARM).lower()}, "
        f"autoStand: {str(AUTO_STAND).lower()}, "
        f"autoDamp: {str(AUTO_DAMP).lower()}, "
        f"maxVx: {MAX_VX}, maxVy: {MAX_VY}, maxWz: {MAX_WZ}, "
        f"armsWhitelist: {list(ARMS_ACTIONS.keys())!r}, "
        f"armDeadmanS: {ARM_DEADMAN_S}"
        "};"
    )
    return PlainTextResponse(body, media_type="application/javascript")


@app.post("/teleop/arm")
def arm(
    body: ArmBody,
    authorization: Optional[str] = Header(None),
    x_teleop_token: Optional[str] = Header(None, alias="X-Teleop-Token"),
):
    """使能：可选自动 stand（预备+跑动 FSM）后再允许 cmd_vel。"""
    _write_guard()
    global _armed, _last_beat, _last_cmd, _busy, _mode_note
    _auth(authorization, x_teleop_token)

    if not body.armed:
        with _lock:
            _armed = False
            _last_cmd = CmdVel()
            _last_beat = time.time()
        _publish(0.0, 0.0, 0.0)
        try:
            from vel_owner import release

            release("teleop", actor="teleop-disarm")
        except Exception as e:
            print(f"[teleop] vel_owner release: {e}", flush=True)
        _mode_note = "disarmed"
        return {"armed": False, "mode_note": _mode_note}

    if _busy:
        raise HTTPException(status_code=409, detail="mode switch in progress")
    if globals().get('_homebody_gesture_active', False):
        raise HTTPException(status_code=409, detail="Official gesture running")

    try:
        from vel_owner import try_acquire

        ok, cur = try_acquire("teleop", actor="teleop-arm", force=VEL_OWNER_FORCE)
        if not ok:
            raise HTTPException(
                status_code=409,
                detail=f"vel owner held by {cur.get('owner')} (actor={cur.get('actor')})",
            )
    except HTTPException:
        raise
    except Exception as e:
        print(f"[teleop] vel_owner acquire: {e}", flush=True)

    out = ""
    if AUTO_STAND:
        global _cached_loco_fsm
        if _cached_loco_fsm in _LOCO_SKIP_STAND_FSMS:
            _mode_note = f"loco ready (FSM~{_cached_loco_fsm})"
            out = f"teleop-stand skip cached FSM {_cached_loco_fsm}"
        else:
            _busy = True
            _mode_note = "standing…"
            _publish(0.0, 0.0, 0.0)
            try:
                _vel_stop()
                out = _do_stand_dispatch()
                fsm = _parse_fsm_from_stand_log(out) or _cached_loco_fsm
                _mode_note = f"loco ready (FSM~{fsm or 801})"
            except Exception as e:
                _busy = False
                _mode_note = f"stand failed: {e}"
                _vel_start()
                raise HTTPException(status_code=502, detail=f"stand failed: {e}") from e
            finally:
                _vel_start()
                _busy = False

    with _lock:
        _armed = True
        _last_beat = time.time()
        _last_cmd = CmdVel()
    return {"armed": True, "mode_note": _mode_note, "stand_log": out[-500:]}


@app.post("/teleop/cmd_vel")
def cmd_vel(
    body: CmdVel,
    authorization: Optional[str] = Header(None),
    x_teleop_token: Optional[str] = Header(None, alias="X-Teleop-Token"),
):
    _write_guard()
    global _last_cmd, _last_beat, _armed
    _auth(authorization, x_teleop_token)
    if _busy:
        raise HTTPException(status_code=409, detail="mode switch in progress")
    if REQUIRE_ARM and not _armed:
        raise HTTPException(status_code=403, detail="not armed")
    if not _ros_ok:
        raise HTTPException(status_code=503, detail="ros /cmd_vel publisher not ready")
    with _lock:
        if globals().get('_homebody_gesture_active', False) and any(abs(v)>1e-5 for v in (body.vx,body.vy,body.wz)):
            raise HTTPException(status_code=409, detail="Official gesture running; stop arms before moving base")
        if not REQUIRE_ARM:
            _armed = True
        vx = _clamp(body.vx, MAX_VX)
        vy = _clamp(body.vy, MAX_VY)
        wz = _clamp(body.wz, MAX_WZ)
        vx, vy, wz, clipped = _filter_cmd_for_map(vx, vy, wz)
        _last_cmd = CmdVel(vx=vx, vy=vy, wz=wz)
        _last_beat = time.time()
    return {
        "ok": True,
        "cmd": _last_cmd.model_dump(),
        "armed": _armed,
        "map_keepout_clipped": clipped,
    }


@app.post("/teleop/estop")
def estop(
    authorization: Optional[str] = Header(None),
    x_teleop_token: Optional[str] = Header(None, alias="X-Teleop-Token"),
):
    """急停：零速 + 可选 Squat→Damp（禁止 Sit / 禁止站立直接阻尼）；并释放手臂。"""
    _write_guard(allow_low_battery=True)  # estop 例外：电量闸不拦（只读态仍 403）
    global _armed, _last_cmd, _last_beat, _busy, _mode_note, _arms_busy
    _auth(authorization, x_teleop_token)

    with _lock:
        _armed = False
        _last_cmd = CmdVel()
        _last_beat = time.time()
    _publish(0.0, 0.0, 0.0)

    # 手臂 release（不挡腿）
    if not _arms_busy:
        _arms_busy = True
        try:
            _arms_release_safe()
        finally:
            _arms_busy = False

    out = ""
    if AUTO_DAMP:
        if _busy:
            return {"ok": True, "armed": False, "mode_note": "busy", "damp_skipped": True}
        _busy = True
        _mode_note = "squat→damp…"
        try:
            _vel_stop()
            out = _do_estop_legs_dispatch()
            _mode_note = "estop squat→damped"
        except Exception as e:
            _mode_note = f"estop_legs failed: {e}"
            print(f"[teleop] estop_legs failed: {e}", flush=True)
        finally:
            _vel_start()
            _busy = False

    try:
        from vel_owner import release

        release("teleop", actor="teleop-estop")
    except Exception as e:
        print(f"[teleop] vel_owner release: {e}", flush=True)

    return {"ok": True, "armed": False, "mode_note": _mode_note, "damp_log": out[-500:]}


@app.post("/teleop/arms/enable")
def arms_enable(
    body: ArmsEnableBody,
    authorization: Optional[str] = Header(None),
    x_teleop_token: Optional[str] = Header(None, alias="X-Teleop-Token"),
):
    """手臂通道使能（与腿分离）；撤使能时仅清标志。"""
    _write_guard()
    global _arms_armed, _arms_beat, _arms_note, _arms_busy
    _auth(authorization, x_teleop_token)
    if body.armed:
        with _lock:
            _arms_armed = True
            _arms_beat = time.time()
        _arms_note = "arms armed"
        return {"arms_armed": True, "arms_note": _arms_note, "whitelist": list(ARMS_ACTIONS.keys())}

    if _arms_busy:
        raise HTTPException(status_code=409, detail="arms busy")
    _arms_busy = True
    try:
        _arms_release_safe()
        with _lock:
            _arms_armed = False
        try:
            from joint_state import get_joint_hub

            get_joint_hub().clear_overlay("waist_yaw_joint")
        except Exception:
            pass
        _arms_note = "arms disarmed + released"
    finally:
        _arms_busy = False
    return {"arms_armed": False, "arms_note": _arms_note}


@app.post("/teleop/arms/heartbeat")
def arms_heartbeat(
    authorization: Optional[str] = Header(None),
    x_teleop_token: Optional[str] = Header(None, alias="X-Teleop-Token"),
):
    """手臂死人开关心跳。未使能时返回 ok=false（避免 403 刷屏）。"""
    _write_guard()
    global _arms_beat
    _auth(authorization, x_teleop_token)
    with _lock:
        armed = _arms_armed
        if armed:
            _arms_beat = time.time()
    if not armed:
        return {"ok": False, "arms_armed": False, "arms_note": _arms_note}
    return {"ok": True, "arms_armed": True}


@app.post("/teleop/arms/action")
def arms_action(
    body: ArmsActionBody,
    authorization: Optional[str] = Header(None),
    x_teleop_token: Optional[str] = Header(None, alias="X-Teleop-Token"),
):
    """执行白名单手臂预设。"""
    _write_guard()
    global _arms_beat, _arms_note, _arms_busy
    _auth(authorization, x_teleop_token)
    if not _arms_armed:
        raise HTTPException(status_code=403, detail="arms not armed")
    name = body.name.strip()
    if name not in ARMS_ACTIONS:
        raise HTTPException(status_code=400, detail=f"not in whitelist: {name}")
    if _arms_busy or _busy:
        raise HTTPException(status_code=409, detail="busy")
    with _lock:
        _arms_beat = time.time()
    _arms_busy = True
    _arms_note = f"running {name}"
    try:
        # 在线程中跑阻塞子进程 DDS，并期间续命 beat，避免单 worker 卡死心跳后被 deadman 误杀
        box: dict[str, Any] = {"out": "", "err": None}

        def _run() -> None:
            try:
                box["out"] = _do_arms_action(name)
            except Exception as e:  # noqa: BLE001
                box["err"] = e

        th = threading.Thread(target=_run, name=f"arms-{name}", daemon=True)
        th.start()
        while th.is_alive():
            with _lock:
                _arms_beat = time.time()
            th.join(0.25)
        if box["err"] is not None:
            raise box["err"]
        _arms_note = f"done {name}"
        return {"ok": True, "name": name, "log": (box["out"] or "")[-400:]}
    except Exception as e:
        _arms_note = f"failed {name}: {e}"
        raise HTTPException(status_code=502, detail=str(e)) from e
    finally:
        with _lock:
            _arms_beat = time.time()
        _arms_busy = False


@app.post("/teleop/arms/estop")
def arms_estop(
    authorization: Optional[str] = Header(None),
    x_teleop_token: Optional[str] = Header(None, alias="X-Teleop-Token"),
):
    """仅手臂急停：撤使能，不影响腿通道。"""
    _write_guard()
    global _arms_busy
    _auth(authorization, x_teleop_token)
    if _arms_busy:
        raise HTTPException(status_code=409, detail="arms busy")
    _arms_busy = True
    try:
        _arms_release_safe()
    finally:
        _arms_busy = False
    return {"ok": True, "arms_armed": False, "arms_note": _arms_note}


@app.get("/legacy")
@app.get("/legacy/")
def legacy_ui():
    """旧键鼠简易页（应急）；主入口已换驾驶舱。"""
    legacy = os.path.join(STATIC_DIR, "legacy.html")
    if not os.path.isfile(legacy):
        legacy = os.path.join(STATIC_DIR, "index.html")
    return FileResponse(legacy, headers={"Cache-Control": "no-store, max-age=0"})


def _cockpit_index() -> str | None:
    p = os.path.join(UI_DIR, "index.html")
    return p if os.path.isfile(p) else None


@app.get("/")
def index():
    ui = _cockpit_index()
    if ui:
        return FileResponse(ui, headers={"Cache-Control": "no-store, max-age=0"})
    # 尚未 deploy_pc2_lan_ui 时回退 legacy
    legacy = os.path.join(STATIC_DIR, "legacy.html")
    if not os.path.isfile(legacy):
        legacy = os.path.join(STATIC_DIR, "index.html")
    return FileResponse(legacy, headers={"Cache-Control": "no-store, max-age=0"})


# Vite base=/ → /assets/*（目录可先空，deploy_pc2_lan_ui 后再填）
_ui_assets = os.path.join(UI_DIR, "assets")
os.makedirs(_ui_assets, exist_ok=True)
app.mount("/assets", StaticFiles(directory=_ui_assets), name="ui_assets")

app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

# Official arm actions are exposed as explicit, authenticated jobs only.
from official_gestures import install as install_official_gestures
install_official_gestures(app, globals())
