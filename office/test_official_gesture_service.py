"""Mocked route-level checks. Never imports the live app or official SDK."""
import ast
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from fastapi import FastAPI,HTTPException
import official_gestures as service

class ServiceChecks(unittest.TestCase):
    def setUp(self):
        self.authorized=[]
        self.scope={'_lock':threading.Lock(),'_busy':False,'_arms_busy':False,'_armed':True,
                    '_last_cmd':SimpleNamespace(vx=0.,vy=0.,wz=0.),
                    '_write_guard':lambda:self.authorized.append('write'),
                    '_auth':lambda a,t:self.authorized.append((a,t))}
        self.app=FastAPI();service.install(self.app,self.scope)
        self.routes={r.path.rsplit('/',1)[-1]:r.endpoint for r in self.app.routes if r.path.startswith('/teleop/gestures/')}

    def start(self,name='wave'):
        with patch.object(service.threading,'Thread'):
            return self.routes['start'](service.StartBody(name=name),'Bearer test','token')

    def test_start_requires_arm_stationary_base_and_exclusive_arms(self):
        for key,value in [('_armed',False),('_busy',True),('_arms_busy',True)]:
            original=self.scope[key];self.scope[key]=value
            with self.assertRaises(HTTPException):self.start()
            self.scope[key]=original
        self.scope['_last_cmd'].vx=.35
        with self.assertRaises(HTTPException):self.start()
        self.scope['_last_cmd'].vx=0
        job=self.start('handshake')
        self.assertEqual(job['name'],'handshake')
        self.assertTrue(self.scope['_homebody_gesture_active'])
        self.assertTrue(self.scope['_arms_busy'])
        with self.assertRaises(HTTPException):self.start()
        self.assertIn(('Bearer test','token'),self.authorized)

    def test_stop_works_even_when_write_gate_blocks(self):
        job=self.start()
        def rejected():raise HTTPException(403,'blocked')
        self.scope['_write_guard']=rejected
        stopped=self.routes['stop'](service.JobBody(job_id=job['job_id']),None,'token')
        self.assertEqual(stopped['job_id'],job['job_id'])
        with self.assertRaises(HTTPException):self.routes['heartbeat'](service.JobBody(job_id=job['job_id']),None,'token')
        with self.assertRaises(HTTPException):self.routes['stop'](service.JobBody(job_id='wrong'),None,'token')

    def test_thread_start_failure_releases_claim(self):
        with patch.object(service.threading,'Thread',side_effect=RuntimeError('no thread')):
            with self.assertRaises(RuntimeError):self.routes['start'](service.StartBody(name='wave'),None,'token')
        self.assertFalse(self.scope['_arms_busy'])
        self.assertFalse(self.scope['_homebody_gesture_active'])

    def test_cmd_vel_rejects_nonzero_during_gesture_but_accepts_stop(self):
        tree=ast.parse((Path(__file__).resolve().parent/'robot/teleop_app.reference.py').read_text(encoding='utf-8'))
        method=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='cmd_vel')
        method.decorator_list=[]
        for arg in method.args.args:arg.annotation=None
        method.args.defaults=[]
        class Cmd:
            def __init__(self,vx=0.,vy=0.,wz=0.):self.vx=vx;self.vy=vy;self.wz=wz
            def model_dump(self):return vars(self)
        env={'_write_guard':lambda:None,'_auth':lambda *_:None,'_busy':False,'_armed':True,
             'REQUIRE_ARM':True,'_ros_ok':True,'_lock':threading.Lock(),'_homebody_gesture_active':True,
             'HTTPException':HTTPException,'MAX_VX':.35,'MAX_VY':.35,'MAX_WZ':.45,'CmdVel':Cmd,
             '_clamp':lambda v,limit:max(-limit,min(limit,v)),
             '_filter_cmd_for_map':lambda x,y,w:(x,y,w,False),'time':__import__('time')}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<cmd_vel>','exec'),env)
        with self.assertRaises(HTTPException):env['cmd_vel'](Cmd(vx=.35),None,None)
        reply=env['cmd_vel'](Cmd(),None,None)
        self.assertEqual(reply['cmd'],{'vx':0.,'vy':0.,'wz':0.})

if __name__=='__main__':unittest.main()
