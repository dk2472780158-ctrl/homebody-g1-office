"""Read observation files and write map pose candidates. No robot/network interfaces."""
import argparse
import json
import os
import signal
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from office_live_geometry import HistoricalMatcher, structural, voxel, gravity_alignment


def atomic_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    os.replace(temp, path)


def validate_raw(raw, now):
    if not raw.get('running'):
        raise ValueError('上游定位已停止')
    transport=raw.get('sensor_transport')
    if transport is not None:
        heartbeat_age=now-float(transport.get('heartbeat_unix_s') or 0)
        if not transport.get('running') or not 0<=heartbeat_age<=.8:
            raise ValueError('传感器传输暂不可用，暂停定位输出')
        # ready synchronizes startup (including cloud/IMU skew < .5 s).
        # A throttled 5 Hz cloud can briefly miss that startup window while
        # initialized FAST-LIO still produces fresh observations. Require actual
        # output freshness and bounded source skew instead of trusting the flag.
        if not transport.get('ready'):
            ages=[now-float(raw.get(k) or 0) for k in
                  ('cloud_received_unix_s','odometry_received_unix_s')]
            stamps=transport.get('source_stamp') or {}
            skew=abs(float(stamps.get('cloud') or 0)-float(stamps.get('imu') or 0))
            if not all(0<=age<.8 for age in ages) or not np.isfinite(skew) or skew>.8:
                raise ValueError('传感器输出不够新鲜或时间差过大，暂停定位输出')
        if max((transport.get('backlog_growth_s') or {'unknown':1.}).values())>.75:
            raise ValueError('传感器数据积压，暂停定位输出')
        odom_stamp=float((raw.get('odometry') or {}).get('stamp_s') or 0)
        if float((transport.get('source_stamp') or {}).get('cloud') or 0)-odom_stamp>.8:
            raise ValueError('FAST-LIO 落后于当前点云，暂停定位输出')
    if raw.get('cloud_received_unix_s') is None or raw.get('odometry_received_unix_s') is None:
        raise ValueError('等待 G1 传感器数据，尚未产生本次定位输出')
    for key in ('heartbeat_unix_s', 'cloud_received_unix_s', 'odometry_received_unix_s'):
        age = now - float(raw.get(key) or 0)
        if not np.isfinite(age) or age < -1 or age > 3:
            raise ValueError('点云或里程计接收已中断')
    if not raw.get('cloud_stamps_nondecreasing') or not raw.get('odom_stamps_nondecreasing'):
        raise ValueError('源时间戳回退；等待重新启动定位')
    odom = raw.get('odometry') or {}
    if (raw.get('cloud_frame') != 'camera_init' or odom.get('frame_id') != 'camera_init'
            or odom.get('child_frame_id') != 'body'):
        raise ValueError('坐标系与只读定位配置不符')
    xyz, quat = np.asarray(odom['xyz'], float), np.asarray(odom['xyzw'], float)
    if (xyz.shape != (3,) or quat.shape != (4,) or not np.isfinite(xyz).all()
            or not np.isfinite(quat).all() or abs(np.linalg.norm(quat) - 1) > .02):
        raise ValueError('里程计位姿无效')
    if np.linalg.norm(xyz)>30:
        raise ValueError('里程计超出办公室范围，等待重新初始化')
    if abs(float(odom['stamp_s']) - float(raw['cloud_stamp_s'])) > 2:
        raise ValueError('点云与里程计时间不连续')
    return odom


def match_scan_windows(matcher,points,recent,latest,up):
    """Try bounded scan windows without changing match acceptance thresholds."""
    errors=[]
    for label,cloud in (('accumulated',points),('recent',recent),('latest',latest)):
        try:
            transform,quality=matcher.global_match(cloud,up=up)
            # A single useful Livox pattern can initialize matching, but it must
            # also explain the independent bounded recent window.
            structure=matcher.current_structure(recent,transform)
            if len(structure)<100:raise ValueError('当前结构点不足')
            check=matcher.metrics(structure,transform)
            if check['within_0_2m_fraction']<.65 or check['median_distance_m']>.16:
                raise ValueError('当前点云与历史地图不一致，等待重新定位')
            return transform,{**quality,'initial_cloud_source':label}
        except ValueError as exc:errors.append(str(exc))
    raise ValueError(errors[-1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--registration', type=Path, required=True)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=True)
    matcher = HistoricalMatcher(args.map)
    office_from_history = np.asarray(json.loads(args.registration.read_text())[
        'T_office_map_from_provided_pcd'], float)
    if office_from_history.shape != (4, 4) or not np.isfinite(office_from_history).all():
        raise ValueError('Invalid historical registration')
    running = True

    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    session, transform, quality = None, None, None
    checked_sequence, checked_at, retry_at = -1, 0., 0.
    match_error=''
    prior_path=args.directory/'onsite-prior.json'
    prior=json.loads(prior_path.read_text()) if prior_path.exists() else None
    output = args.directory / 'live-state.json'

    def state(status, message, raw=None, pose=None):
        raw = raw or {}
        atomic_json(output, dict(status=status, message=message, updated_unix_s=time.time(),
            session_id=raw.get('session_id'), readonly=True, candidate=True,
            navigation_ready=False, base_extrinsics_validated=False,
            pose=pose, quality=quality, geometry_checked_unix_s=checked_at,
            cloud_received_unix_s=raw.get('cloud_received_unix_s'),
            odometry_received_unix_s=raw.get('odometry_received_unix_s'),
            cloud_messages=raw.get('cloud_messages', 0),
            odometry_messages=raw.get('odometry_messages', 0),
            source_absolute_timestamp_offset='preserved, not corrected',
            onsite_approximate_region=prior.get('region_label','operator confirmed region') if prior else None,
            onsite_approximate_heading=prior.get('heading_degrees') if prior and prior.get('session_id')==raw.get('session_id') else None,
            onsite_confirmation=prior.get('statement') if prior else None,
            onsite_confirmed_at_unix_s=prior.get('confirmed_at_unix_s') if prior else None,
            T_office_from_current=(office_from_history@transform).tolist()
                if status=='live' and transform is not None else None))

    state('waiting', '等待本次启动的点云和里程计')
    try:
        while running:
            raw = {}
            try:
                raw = json.loads((args.directory / 'raw-state.json').read_text())
                if not isinstance(raw.get('session_id'), str) or not raw['session_id']:
                    raise ValueError('缺少本次定位会话标识')
                if raw['session_id'] != session:
                    session, transform, quality = raw['session_id'], None, None
                    checked_sequence, checked_at, retry_at = -1, 0., 0.
                    match_error=''
                odom = validate_raw(raw, time.time())
                if transform is None or time.time() - checked_at > 2:
                    with np.load(args.directory / 'clouds.npz', allow_pickle=False) as data:
                        seq = int(data['sequence'])
                        if str(data['session_id']) != session or str(data['frame_id']) != 'camera_init':
                            raise ValueError('等待与本次会话一致的点云')
                        # NPZ and JSON are replaced atomically but independently.
                        # One snapshot of skew is normal while the writer updates JSON.
                        if abs(seq-int(raw['cloud_sequence']))>1:
                            raise ValueError('正在更新点云快照')
                        snapshot_received = float(data['received_unix_s'])
                        if not -1<=time.time()-snapshot_received<=3:
                            raise ValueError('配准点云快照已过期')
                        if abs(float(data['stamp_s'])-float(odom['stamp_s']))>2:
                            raise ValueError('配准快照与里程计时间不连续')
                        points, latest = data['xyz'].copy(), data['latest_xyz'].copy()
                        recent=data['recent_xyz'].copy() if 'recent_xyz' in data else latest
                        recent_frames=int(data['recent_frames']) if 'recent_frames' in data else 1
                        recent_oldest=float(data['recent_oldest_received_unix_s']) if 'recent_oldest_received_unix_s' in data else snapshot_received
                        recent_stamp=float(data['recent_oldest_stamp_s']) if 'recent_oldest_stamp_s' in data else float(data['stamp_s'])
                        if (recent_frames>7 or snapshot_received-recent_oldest>2.5 or
                                not 0<=float(data['stamp_s'])-recent_stamp<=2.5):
                            raise ValueError('近期点云窗口时间不连续')
                    if not np.isfinite(points).all() or not np.isfinite(latest).all():
                        raise ValueError('点云包含无效数值')
                    if transform is None:
                        if time.time()<retry_at and match_error:
                            raise ValueError(match_error)
                        if raw.get('cloud_window_frames', 0) < 6 or time.time() < retry_at:
                            raise ValueError('正在积累本次定位点云')
                        state('matching', '正在将本次点云重新对齐历史办公室地图', raw)
                        retry_at = time.time() + 12
                        up=raw.get('initial_up_current_xyz')
                        if up is None:
                            raise ValueError('等待本次 IMU 重力方向初始化')
                        try:
                            if prior is not None and prior.get('session_id')==session:
                                if time.time()-float(prior['confirmed_at_unix_s'])>1800:
                                    raise ValueError('现场初始区域确认已过期，等待重新确认后定位')
                                transform,quality=matcher.match_with_prior(latest if prior.get('initial_cloud_source')=='latest_xyz' else recent if prior.get('initial_cloud_source')=='recent_xyz' else points,up,prior)
                            else:
                                transform, quality = match_scan_windows(matcher,points,recent,latest,up)
                            match_error=''
                        except ValueError as exc:
                            reason={'Global match ambiguous in searched candidates':'存在多个相近候选，暂不显示位置',
                                    'Global match quality too low':'当前点云与历史地图吻合度不足'}.get(str(exc),str(exc))
                            match_error='历史地图尚未匹配：'+reason
                            print(match_error,flush=True)
                            raise
                        print(json.dumps({'session': session, 'quality': quality,
                                          'T_history_from_current': transform.tolist()}), flush=True)
                    if seq != checked_sequence:
                        # Apply the fitted height as well as gravity before selecting walls.
                        # Livox scans are nonrepetitive: validate a bounded recent
                        # seven-scan window rather than one sparse sampling pattern.
                        source = matcher.current_structure(recent,transform)
                        if len(source) < 100:
                            raise ValueError('当前结构点不足')
                        check = matcher.metrics(source, transform)
                        if check['within_0_2m_fraction'] < .65 or check['median_distance_m'] > .16:
                            transform = None
                            raise ValueError('当前点云与历史地图不一致，等待重新定位')
                        check={**check,'window_frames':recent_frames,
                               'window_span_s':snapshot_received-recent_oldest,
                               'oldest_received_unix_s':recent_oldest}
                        quality = {**quality, 'latest_cloud': check}
                        checked_sequence, checked_at = seq, snapshot_received
                    # Matching may take seconds: re-read the current session and freshness.
                    raw = json.loads((args.directory / 'raw-state.json').read_text())
                    if raw.get('session_id') != session:
                        transform = None
                        raise ValueError('定位刚刚重新启动，等待重新对齐')
                    odom = validate_raw(raw, time.time())
                if time.time() - checked_at > 6:
                    raise ValueError('地图几何校验已过期')
                body = np.eye(4)
                body[:3, :3] = Rotation.from_quat(odom['xyzw']).as_matrix()
                body[:3, 3] = odom['xyz']
                pose = office_from_history @ transform @ body
                state('live', '实时接收 · 历史地图定位候选 · 只读', raw,
                    dict(frame='office_map', child_frame='FAST-LIO body',
                         xyz_m=pose[:3, 3].tolist(),
                         yaw_degrees=float(np.rad2deg(np.arctan2(pose[1, 0], pose[0, 0]))),
                         sensor_stamp_s=odom['stamp_s']))
            except (OSError, ValueError, KeyError, TypeError) as exc:
                # Never retain a drawable pose after interruption or a failed validation.
                state('waiting', str(exc), raw)
            time.sleep(.05)
    finally:
        state('stopped', '只读地图定位进程已停止')


if __name__ == '__main__':
    main()
