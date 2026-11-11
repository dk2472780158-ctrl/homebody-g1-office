"""15-minute subscription-only trace. Does not create any ROS publisher."""
import json,time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from unitree_api.msg import Request,Response
from unitree_go.msg import SportModeState

rclpy.init();node=rclpy.create_node('homebody_motion_trace_readonly')
out=Path('/tmp/homebody-motion-trace.jsonl').open('a',buffering=1)
counts={'twist':0,'request':0,'response':0};pending={};active_until=0.;last_velocity=None
def write(kind,**data):
    out.write(json.dumps(dict(at=time.time(),kind=kind,**data),ensure_ascii=False)+'\n')
def twist(msg):
    global active_until
    counts['twist']+=1;v=[msg.linear.x,msg.linear.y,msg.angular.z]
    if any(abs(x)>1e-6 for x in v):active_until=time.monotonic()+3
    if time.monotonic()<active_until:write('twist',velocity=v)
def request(msg):
    global active_until,last_velocity
    if msg.header.identity.api_id!=7105:return
    counts['request']+=1
    try:p=json.loads(msg.parameter)
    except (ValueError,TypeError):return
    v=p.get('velocity',[])
    if any(abs(float(x))>1e-6 for x in v):active_until=time.monotonic()+3
    rid=int(msg.header.identity.id)
    if time.monotonic()<active_until:
        pending[rid]=time.monotonic();write('request',id=rid,velocity=v,duration=p.get('duration'))
    last_velocity=v
def response(msg):
    counts['response']+=1;rid=int(msg.header.identity.id)
    sent=pending.pop(rid,None)
    if sent is not None:write('response',id=rid,api=int(msg.header.identity.api_id),code=int(msg.header.status.code),data=msg.data,latency_s=time.monotonic()-sent)
def odom(msg):
    if time.monotonic()<active_until:
        write('motion',position=[float(x) for x in msg.position],velocity=[float(x) for x in msg.velocity],yaw_speed=float(msg.yaw_speed))
subs=[node.create_subscription(Twist,'/cmd_vel',twist,100),
      node.create_subscription(Request,'/api/sport/request',request,100),
      node.create_subscription(Response,'/api/sport/response',response,100),
      node.create_subscription(SportModeState,'/odommodestate',odom,rclpy.qos.qos_profile_sensor_data)]
write('started',mode='subscriptions_only',duration_s=900)
deadline=time.monotonic()+900;next_status=0
try:
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
        if time.monotonic()>next_status:
            write('status',counts=counts,last_velocity=last_velocity,unanswered=list(pending))
            pending={k:v for k,v in pending.items() if time.monotonic()-v<10}
            next_status=time.monotonic()+2
finally:
    write('finished',counts=counts);out.close();node.destroy_node();rclpy.shutdown()
