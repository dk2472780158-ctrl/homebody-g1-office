"""Record walking API and leg targets/feedback for 15 minutes; subscriptions only."""
import json
import time
from pathlib import Path

import rclpy
from rclpy.qos import qos_profile_sensor_data
from unitree_api.msg import Request, Response
from unitree_hg.msg import LowCmd, LowState


def leg_values(msg, command=False):
    motors = msg.motor_cmd if command else msg.motor_state
    fields = ('q', 'dq', 'tau', 'kp', 'kd') if command else ('q', 'dq', 'tau_est')
    return {field: [float(getattr(m, field)) for m in motors[:12]] for field in fields}


def main():
    rclpy.init()
    node = rclpy.create_node('homebody_leg_trace_readonly')
    output = Path('/tmp/homebody-leg-trace.jsonl').open('a', buffering=1)
    active_until = 0.
    last = {}
    counts = {}
    pending = set()

    def write(kind, **data):
        counts[kind] = counts.get(kind, 0) + 1
        output.write(json.dumps(dict(at=time.time(), kind=kind, **data)) + '\n')

    def request(msg):
        nonlocal active_until
        if int(msg.header.identity.api_id) != 7105:
            return
        try:
            parameter = json.loads(msg.parameter)
        except (ValueError, TypeError):
            return
        velocity = parameter.get('velocity', [])
        if any(abs(float(x)) > 1e-6 for x in velocity):
            active_until = time.monotonic() + 3
        if time.monotonic() < active_until:
            rid = int(msg.header.identity.id)
            pending.add(rid)
            write('request', id=rid, velocity=velocity, duration=parameter.get('duration'))

    def response(msg):
        rid = int(msg.header.identity.id)
        if rid in pending:
            pending.remove(rid)
            write('response', id=rid, code=int(msg.header.status.code))

    def motor(msg, command=False):
        kind = 'leg_target' if command else 'leg_feedback'
        now = time.monotonic()
        interval = .05 if now < active_until else .2
        if now - last.get(kind, 0) < interval:
            return
        last[kind] = now
        data = leg_values(msg, command)
        if not command:
            data['motorstate'] = [int(m.motorstate) for m in msg.motor_state[:12]]
        write(kind, **data)

    subscriptions = [
        node.create_subscription(Request, '/api/sport/request', request, 100),
        node.create_subscription(Response, '/api/sport/response', response, 100),
        node.create_subscription(LowCmd, '/lowcmd', lambda m: motor(m, True), qos_profile_sensor_data),
        node.create_subscription(LowState, '/lowstate', motor, qos_profile_sensor_data),
    ]
    write('started', mode='subscriptions_only', duration_s=900)
    deadline = time.monotonic() + 900
    status_at = 0.
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() > status_at:
                write('status', counts=dict(counts))
                status_at = time.monotonic() + 5
    finally:
        write('finished', counts=dict(counts))
        output.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
