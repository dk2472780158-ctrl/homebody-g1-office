"""Execute only official wave/handshake IDs; no locomotion or mode commands."""
import json,os,signal,sys,time

ACTION_IDS={'wave':26,'handshake':27}

def execute(name,arm,stopped):
    if name not in ACTION_IDS:raise ValueError('Unknown gesture')
    if stopped():raise RuntimeError('Gesture cancelled before start')
    cancelled=False
    try:
        code=arm.ExecuteAction(ACTION_IDS[name])
        if code!=0:raise RuntimeError(f'Official arm action {ACTION_IDS[name]} rejected, code={code}; no mode was changed')
        end=time.monotonic()+(3. if name=='wave' else 2.)
        while time.monotonic()<end:
            if stopped():cancelled=True;break
            time.sleep(.05)
    finally:
        release=arm.ExecuteAction(99)
        if release!=0:raise RuntimeError(f'Official arm release rejected, code={release}')
    if cancelled:raise RuntimeError('Gesture cancelled and arms released')
    return {'ok':True,'name':name,'action_id':ACTION_IDS[name],'released':True}

def main():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    name=sys.argv[1]
    if name not in ACTION_IDS:raise ValueError('Unknown gesture')
    stop=[False]
    def cancel(*_):stop[0]=True
    signal.signal(signal.SIGTERM,cancel);signal.signal(signal.SIGINT,cancel)
    ChannelFactoryInitialize(0,os.environ.get('G1_NETWORK_INTERFACE','eth0'))
    arm=G1ArmActionClient();arm.SetTimeout(2.);arm.Init()
    print(json.dumps(execute(name,arm,lambda:stop[0])),flush=True)

if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(json.dumps({'ok':False,'error':str(exc)}),flush=True)
        sys.exit(1)
