"""Subscribe only: validate binary sensor messages before relay deployment."""
import base64
import importlib.util
from importlib.machinery import SourceFileLoader
import json
import time

import cbor2
import websocket
from rclpy.serialization import serialize_message

spec=importlib.util.spec_from_loader('candidate',SourceFileLoader('candidate','/workspace/rosbridge_to_ros2.py.next'))
relay=importlib.util.module_from_spec(spec);spec.loader.exec_module(relay)
results=[]
for topic,endpoint,convert in ((relay.CLOUD_TOPIC,19090,relay.cloud_from_json),
                               (relay.IMU_TOPIC,19091,relay.imu_from_json)):
    ws=websocket.create_connection(f'ws://127.0.0.1:{endpoint}',timeout=5)
    try:
        ws.send(json.dumps(dict(op='subscribe',topic=topic,id='verify_cbor',
                                compression='cbor',queue_length=1,throttle_rate=200)))
        deadline=time.monotonic()+8
        count=0;binary=0;last=None
        while time.monotonic()<deadline and count<5:
            frame=ws.recv()
            item=cbor2.loads(frame) if isinstance(frame,bytes) else json.loads(frame)
            if item.get('op')=='status' and item.get('level')=='error':raise ValueError(item)
            if item.get('op')!='publish' or item.get('topic')!=topic:continue
            message=convert(item['msg'])
            assert message.header.stamp.sec==item['msg']['header']['stamp']['sec']
            assert message.header.stamp.nanosec==item['msg']['header']['stamp']['nanosec']
            if topic==relay.CLOUD_TOPIC:
                legacy=dict(item['msg']);legacy['data']=base64.b64encode(legacy['data']).decode('ascii')
                assert serialize_message(message)==serialize_message(convert(legacy)), 'binary cloud differs from JSON path'
            count+=1;binary+=isinstance(frame,bytes);last=message.header.stamp.sec
        assert count==5 and binary==5,(topic,count,binary)
        results.append(dict(topic=topic,messages=count,binary=binary,source_sec=last))
    finally:
        ws.send(json.dumps(dict(op='unsubscribe',topic=topic,id='verify_cbor')));ws.close()
print(json.dumps(dict(readonly=True,results=results)))
