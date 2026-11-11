"""Authenticated, cancellable official gesture jobs using the existing teleop gate."""
import json,subprocess,sys,threading,time,uuid
from pathlib import Path
from fastapi import Header,HTTPException
from pydantic import BaseModel

class StartBody(BaseModel):
    name:str

class JobBody(BaseModel):
    job_id:str

def install(app,scope):
    lock=threading.Lock()
    job={'state':'idle'}
    proc=None
    beat=0.
    cancel=threading.Event()

    def auth(authorization,token,write=False):
        if write:scope['_write_guard']()
        scope['_auth'](authorization,token)

    def snapshot():
        with lock:return dict(job)

    def run(name,job_id):
        nonlocal proc
        child=None
        terminal='failed';result={'ok':False,'error':'Gesture worker did not finish'}
        try:
            if cancel.is_set():raise RuntimeError('Gesture cancelled before start')
            child=subprocess.Popen([sys.executable,str(Path(__file__).with_name('official_gesture_runner.py')),name],
                stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            with lock:proc=child
            begun=time.monotonic();terminated=False;stop_at=None
            while child.poll() is None:
                if (cancel.is_set() or not scope['_armed'] or scope['_busy'] or
                        time.monotonic()-beat>1.5 or time.monotonic()-begun>12):
                    if not terminated:
                        child.terminate();terminated=True;stop_at=time.monotonic()
                    elif time.monotonic()-stop_at>6:
                        child.kill()
                time.sleep(.05)
            stdout,stderr=child.communicate()
            try:result=json.loads(stdout.strip().splitlines()[-1])
            except (ValueError,IndexError):result={'ok':False,'error':(stderr or stdout)[-500:]}
            terminal='cancelled' if terminated else 'done' if child.returncode==0 and result.get('ok') else 'failed'
        except Exception as exc:
            result={'ok':False,'error':str(exc)}
        finally:
            if child is not None and child.poll() is None:
                child.terminate()
                try:child.wait(timeout=6)
                except subprocess.TimeoutExpired:child.kill();child.wait()
            with scope['_lock']:
                with lock:
                    proc=None
                    job.update(state=terminal,result=result,finished_unix_s=time.time())
                    scope['_arms_busy']=False
                    scope['_homebody_gesture_active']=False

    @app.get('/teleop/gestures/status')
    def status(authorization:str|None=Header(None),x_teleop_token:str|None=Header(None,alias='X-Teleop-Token')):
        auth(authorization,x_teleop_token)
        return {'available':['wave','handshake'],'job':snapshot()}

    @app.post('/teleop/gestures/start')
    def start(body:StartBody,authorization:str|None=Header(None),x_teleop_token:str|None=Header(None,alias='X-Teleop-Token')):
        nonlocal beat,job
        auth(authorization,x_teleop_token,True)
        if body.name not in ('wave','handshake'):raise HTTPException(400,'Unknown gesture')
        with scope['_lock']:
            if scope['_busy'] or scope['_arms_busy']:raise HTTPException(409,'Robot or arms busy')
            if not scope['_armed']:raise HTTPException(403,'Leg control not armed; stand under operator supervision first')
            cmd=scope['_last_cmd']
            if any(abs(v)>1e-5 for v in (cmd.vx,cmd.vy,cmd.wz)):raise HTTPException(409,'Stop base before gesture')
            with lock:
                if job.get('state')=='running':raise HTTPException(409,'Gesture busy')
                scope['_arms_busy']=True
                scope['_homebody_gesture_active']=True
                cancel.clear();beat=time.monotonic()
                job={'job_id':uuid.uuid4().hex,'name':body.name,'state':'running','started_unix_s':time.time()}
                current=dict(job)
            try:threading.Thread(target=run,args=(body.name,current['job_id']),daemon=True).start()
            except Exception:
                with lock:job.update(state='failed',result={'ok':False,'error':'Cannot start gesture worker'})
                scope['_arms_busy']=False;scope['_homebody_gesture_active']=False
                raise
        return current

    @app.post('/teleop/gestures/heartbeat')
    def heartbeat(body:JobBody,authorization:str|None=Header(None),x_teleop_token:str|None=Header(None,alias='X-Teleop-Token')):
        nonlocal beat
        auth(authorization,x_teleop_token,True)
        with lock:
            if job.get('job_id')!=body.job_id:raise HTTPException(409,'Gesture job changed')
            beat=time.monotonic()
            return dict(job)

    @app.post('/teleop/gestures/stop')
    def stop(body:JobBody,authorization:str|None=Header(None),x_teleop_token:str|None=Header(None,alias='X-Teleop-Token')):
        auth(authorization,x_teleop_token)
        with lock:
            if job.get('job_id')!=body.job_id:raise HTTPException(409,'Gesture job changed')
            cancel.set()
            return dict(job)

    @app.on_event('shutdown')
    def shutdown():
        cancel.set()
        child=proc
        if child is not None and child.poll() is None:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill()
