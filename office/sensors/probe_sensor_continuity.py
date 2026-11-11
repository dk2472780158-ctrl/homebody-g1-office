"""Bounded sensor-only probe; writes metadata, no motion or service requests."""
import hb_config
import argparse
import base64
import json
import time
from pathlib import Path
import numpy as np
import websocket

CLOUD = '/utlidar/cloud_livox_mid360'
IMU = '/utlidar/imu_livox_mid360'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    ws = websocket.create_connection(hb_config.expand('${HB_ROSBRIDGE_SOURCE}'), timeout=5)
    ws.settimeout(1)
    samples = {CLOUD: [], IMU: []}
    accel, angular, cloud_points = [], [], []
    first_cloud = None
    start = time.monotonic()
    try:
        for i, topic in enumerate(samples):
            ws.send(json.dumps(dict(op='subscribe', id=f'probe_{i}', topic=topic, queue_length=1)))
        while time.monotonic()-start < args.seconds:
            try:
                item = json.loads(ws.recv())
            except websocket.WebSocketTimeoutException:
                continue
            topic = item.get('topic')
            if item.get('op') != 'publish' or topic not in samples:
                continue
            msg = item['msg']
            stamp = msg['header']['stamp']
            samples[topic].append([time.time(), stamp['sec']+stamp['nanosec']/1e9])
            if topic == IMU:
                accel.append([msg['linear_acceleration'][a] for a in 'xyz'])
                angular.append([msg['angular_velocity'][a] for a in 'xyz'])
            else:
                cloud_points.append(msg['width']*msg['height'])
                if first_cloud is None:
                    fields = {f['name']: f for f in msg['fields']}
                    payload = base64.b64decode(msg['data'])
                    byte_order = '>' if msg['is_bigendian'] else '<'
                    def field(name, dtype):
                        return np.ndarray((msg['height'], msg['width']), dtype=byte_order+dtype,
                            buffer=payload, offset=fields[name]['offset'],
                            strides=(msg['row_step'], msg['point_step'])).reshape(-1)
                    xyz = np.column_stack([field(a, 'f4') for a in 'xyz'])
                    offsets = field('time', 'f4')
                    first_cloud = dict(fields=msg['fields'], frame_id=msg['header']['frame_id'],
                        finite_xyz=int(np.isfinite(xyz).all(axis=1).sum()),
                        xyz_min=xyz.min(axis=0).tolist(), xyz_max=xyz.max(axis=0).tolist(),
                        point_time_ns_min_max=[float(offsets.min()), float(offsets.max())])
    finally:
        for i, topic in enumerate(samples):
            try:
                ws.send(json.dumps(dict(op='unsubscribe', id=f'probe_{i}', topic=topic)))
            except Exception:
                pass
        ws.close()
    def intervals(a):
        return dict(min_s=float(a.min()), median_s=float(np.median(a)), max_s=float(a.max()),
                    nonincreasing=int((a <= 0).sum())) if len(a) else None
    report = dict(duration_s=time.monotonic()-start, scope='sensor subscribe/unsubscribe only',
                  topics={}, first_cloud=first_cloud)
    for topic, values in samples.items():
        a = np.asarray(values)
        report['topics'][topic] = dict(messages=len(a),
            receive_intervals=intervals(np.diff(a[:, 0])) if len(a) else None,
            stamp_intervals=intervals(np.diff(a[:, 1])) if len(a) else None,
            absolute_offset_s_median=float(np.median(a[:, 0]-a[:, 1])) if len(a) else None)
    for name, values in [('acceleration_raw', accel), ('angular_velocity', angular)]:
        a = np.asarray(values)
        if len(a):
            norm = np.linalg.norm(a, axis=1)
            report[name] = dict(mean_xyz=a.mean(axis=0).tolist(), std_xyz=a.std(axis=0).tolist(),
                                norm_min_med_max=[float(norm.min()), float(np.median(norm)), float(norm.max())])
    report['cloud_points_min_max'] = [min(cloud_points), max(cloud_points)] if cloud_points else None
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
