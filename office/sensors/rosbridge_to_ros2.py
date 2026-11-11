"""Relay only G1 Mid360 sensor messages into an isolated local ROS 2 domain.

The ROSbridge source is read-only: this process sends only subscribe and
unsubscribe requests, never publish, service calls, or robot commands.
"""

import argparse
import array
import base64
import json
import signal
import time
import threading
from pathlib import Path
import cbor2

from sensor_continuity import SensorContinuity, ContinuityError, BacklogGuard

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
import websocket


CLOUD_TOPIC = "/utlidar/cloud_livox_mid360"
IMU_TOPIC = "/utlidar/imu_livox_mid360"
running = True


def stop(*_args):
    global running
    running = False


def stamp_header(out, data):
    stamp = data["stamp"]
    out.stamp.sec = int(stamp["sec"])
    out.stamp.nanosec = int(stamp["nanosec"])
    out.frame_id = data.get("frame_id", "")


def cloud_from_json(data):
    out = PointCloud2()
    stamp_header(out.header, data["header"])
    out.height = int(data["height"])
    out.width = int(data["width"])
    out.fields = [PointField(name=f["name"], offset=int(f["offset"]),
                             datatype=int(f["datatype"]), count=int(f["count"]))
                  for f in data["fields"]]
    out.is_bigendian = bool(data["is_bigendian"])
    out.point_step = int(data["point_step"])
    out.row_step = int(data["row_step"])
    out.is_dense = bool(data["is_dense"])
    payload = data["data"]
    decoded = bytes(payload) if isinstance(payload, (bytes, bytearray)) else base64.b64decode(payload, validate=True)
    if len(decoded) != out.row_step * out.height:
        raise ValueError("PointCloud2 data length does not match row_step * height")
    fields = {f.name: f for f in out.fields}
    if fields.get("time") is None or fields.get("ring") is None:
        raise ValueError("G1 cloud lacks time or ring field")
    out.data = array.array("B", decoded)
    return out


def imu_from_json(data):
    out = Imu()
    stamp_header(out.header, data["header"])
    for name in ("x", "y", "z", "w"):
        setattr(out.orientation, name, float(data["orientation"][name]))
    for vector in ("angular_velocity", "linear_acceleration"):
        for name in ("x", "y", "z"):
            setattr(getattr(out, vector), name, float(data[vector][name]))
    for name in ("orientation_covariance", "angular_velocity_covariance",
                 "linear_acceleration_covariance"):
        setattr(out, name, [float(v) for v in data[name]])
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="ws://127.0.0.1:19090")
    parser.add_argument('--imu-source',default='ws://127.0.0.1:19091')
    parser.add_argument('--exit-on-discontinuity', action='store_true')
    parser.add_argument('--state', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Node("g1_mid360_rosbridge_input")
    pubs = {
        CLOUD_TOPIC: node.create_publisher(PointCloud2, CLOUD_TOPIC, qos_profile_sensor_data),
        IMU_TOPIC: node.create_publisher(Imu, IMU_TOPIC, qos_profile_sensor_data),
    }
    counts = {name: 0 for name in pubs}
    guard = SensorContinuity();backlog=BacklogGuard()
    lock=threading.Lock();shutdown=threading.Event();failures=[]
    source_state = {'started_unix_s': time.time(), 'running': True, 'ready': False,
                    'received': {}, 'source_stamp':{},'backlog_growth_s':{},'counts': counts,
                    'transport':'CBOR; independent SSH transports: 100 Hz IMU/feedback and 5 Hz cloud'}
    last_state_write = 0.

    def write_state(force=False):
        nonlocal last_state_write
        if args.state and (force or time.monotonic()-last_state_write > .5):
            with lock:
                source_state['heartbeat_unix_s'] = time.time()
                text=json.dumps(source_state)
            temp = args.state.with_suffix('.tmp')
            temp.write_text(text, encoding='utf-8')
            temp.replace(args.state)
            last_state_write = time.monotonic()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def receive(topic,index):
        ws=None
        try:
            endpoint=args.imu_source if topic==IMU_TOPIC else args.source
            ws=websocket.create_connection(endpoint,timeout=5);ws.settimeout(.5)
            # ROSbridge bounds its per-topic queue only when throttling is on.
            # Large cloud JSON must never queue in front of the IMU stream.
            ws.send(json.dumps({'op':'subscribe','topic':topic,'id':f'hb_sensors_{index}',
                                'queue_length':1,'compression':'cbor',
                                'throttle_rate':200 if topic==CLOUD_TOPIC else 10}))
            print(f'Subscribed read-only on separate connection: {topic}',flush=True)
            while running and not shutdown.is_set():
                try:
                    frame=ws.recv()
                    item=cbor2.loads(frame) if isinstance(frame,bytes) else json.loads(frame)
                except websocket.WebSocketTimeoutException:continue
                if item.get('op')!='publish' or item.get('topic')!=topic:continue
                data=item['msg'];header=data['header']['stamp']
                stamp=float(header['sec'])+float(header['nanosec'])/1e9
                kind='cloud' if topic==CLOUD_TOPIC else 'imu'
                received=time.monotonic()
                with lock:
                    if args.exit_on_discontinuity and not guard.observe(kind,stamp,received):
                        source_state['dropped_reordered_imu']=guard.dropped_reordered_imu
                        continue
                    if not args.exit_on_discontinuity:guard.observe(kind,stamp,received)
                    lag=backlog.observe(kind,stamp,received)
                    source_state['received'][kind]=time.time()
                    source_state['source_stamp'][kind]=stamp
                    source_state['backlog_growth_s'][kind]=lag
                    source_state['ready']=guard.ready(received)
                    can_forward=guard.can_forward(received)
                if not can_forward:continue
                message=cloud_from_json(data) if topic==CLOUD_TOPIC else imu_from_json(data)
                pubs[topic].publish(message)
                with lock:counts[topic]+=1
        except Exception as exc:
            with lock:failures.append(str(exc))
            shutdown.set()
        finally:
            if ws is not None:
                try:ws.send(json.dumps({'op':'unsubscribe','topic':topic,'id':f'hb_sensors_{index}'}))
                except Exception:pass
                ws.close()

    threads=[threading.Thread(target=receive,args=(topic,i),daemon=True) for i,topic in enumerate(pubs)]
    try:
        write_state(force=True)
        for thread in threads:thread.start()
        while running and not shutdown.wait(.1):
            with lock:
                if args.exit_on_discontinuity:guard.check_idle(time.monotonic())
            write_state()
        if failures:raise ContinuityError(failures[0])
    finally:
        shutdown.set()
        for thread in threads:thread.join(timeout=2)
        source_state.update(running=False, ready=False)
        if failures:source_state['error']=failures[0]
        write_state(force=True)
        print(f"Final ROS 2 relay counts: {counts}", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
