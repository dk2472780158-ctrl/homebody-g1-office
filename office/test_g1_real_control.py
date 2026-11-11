import tempfile,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import g1_real_control as control

class ControlChecks(unittest.TestCase):
    def client(self):
        client=object.__new__(control.Teleop);client.lock=threading.Lock();self.calls=[]
        client.request=lambda path,body=None,timeout=1:self.calls.append((path,body)) or {'ok':True}
        return client
    def test_bridge_signs(self):
        client=self.client()
        with tempfile.TemporaryDirectory() as folder,patch.object(control,'STOPFILE',Path(folder)/'stop'):
            client.velocity(.04,.05,.1,threading.Event())
        self.assertEqual(self.calls[0][1],{'vx':.04,'vy':-.05,'wz':-.1})
    def test_reject_speed_before_transport(self):
        client=self.client()
        with self.assertRaises(ValueError):client.velocity(.3,.3,0,threading.Event())
        self.assertEqual(self.calls,[])
    def test_stop_latch_blocks_resume(self):
        client=self.client()
        with tempfile.TemporaryDirectory() as folder:
            stop=Path(folder)/'stop';stop.touch()
            with patch.object(control,'STOPFILE',stop),self.assertRaises(RuntimeError):
                client.velocity(.04,0,0,threading.Event())
        self.assertEqual(self.calls,[])
    def test_cancel_blocks_write(self):
        client=self.client();cancel=threading.Event();cancel.set()
        with self.assertRaises(RuntimeError):client.velocity(.04,0,0,cancel)
        self.assertEqual(self.calls,[])
    def test_zero_is_zero(self):
        client=self.client();client.zero()
        self.assertEqual(self.calls[0][1],{'vx':0.,'vy':0.,'wz':0.})
    def test_clipped_command_is_stopped_and_rejected(self):
        client=self.client()
        client.request=lambda path,body=None,timeout=1:self.calls.append((path,body)) or {'ok':True,'cmd':{'vx':0.,'vy':0.,'wz':0.}}
        with tempfile.TemporaryDirectory() as folder,patch.object(control,'STOPFILE',Path(folder)/'stop'):
            with self.assertRaisesRegex(RuntimeError,'裁剪'):
                client.velocity(0.,0.,control.TURN_RATE,threading.Event())
        self.assertEqual(len(self.calls),2)
        self.assertEqual(self.calls[-1][1],{'vx':0.,'vy':0.,'wz':0.})
    def test_owner_required(self):
        h={'armed':True,'ros':True,'vel_owner':{'owner':'other'}}
        with self.assertRaises(RuntimeError):control.require_ready(h)
    def test_rejects_unsynced_robot_speed_limit(self):
        h={'armed':True,'ros':True,'vel_owner':{'owner':'teleop'},'limits':{'vx':.35,'vy':.25}}
        with self.assertRaisesRegex(RuntimeError,'行走限速低于任务速度'):
            control.require_ready(h)
    def test_low_battery_reason_is_explicit(self):
        h={'armed':True,'ros':True,'battery':{'soc':9},'soc_crit_pct':20,
           'write_blocked_reason':'battery critical soc=9% (<20%)'}
        with self.assertRaisesRegex(RuntimeError,'电量 9%.*门限 20%.*充电'):control.require_ready(h)
    def test_world_to_body(self):
        pose=np.eye(4);pose[:2,:2]=[[0,-1],[1,0]]
        np.testing.assert_allclose(control.world_to_body(np.array([0,.08]),pose),[.08,0],atol=1e-9)
    def test_failure_sends_stop(self):
        def fail():raise RuntimeError('lost pose')
        executor=object.__new__(control.Executor)
        executor.w=SimpleNamespace(snapshot=fail)
        stopped=[]
        executor.client=SimpleNamespace(health=lambda:{'armed':True,'ros':True,'vel_owner':{'owner':'teleop'}},zero=lambda:stopped.append(True))
        with self.assertRaisesRegex(RuntimeError,'lost pose'):executor.run([{'skill':'move_for'}],'session')
        self.assertEqual(stopped,[True])
    def test_curved_forward_command_checks_obstacles_on_the_arc(self):
        executor=object.__new__(control.Executor)
        now=time.time()
        state={'session_id':'same','odometry_received_unix_s':now,'cloud_received_unix_s':now,
               'T_office_from_current':np.eye(4).tolist()}
        with tempfile.TemporaryDirectory() as folder,patch.object(control,'STOPFILE',Path(folder)/'stop'):
            end=control.projected_travel([0.,0.],[.35,0.],.25)[-1]
            obstacles=np.array([[end[0]+dx,end[1]+.395,1.] for dx in (-.001,0.,.001)])
            np.savez(Path(folder)/'clouds.npz',session_id='same',received_unix_s=now,latest_xyz=obstacles)
            executor.w=SimpleNamespace(cancel=threading.Event(),snapshot=lambda:(state,np.eye(4)),
                planner=SimpleNamespace(radius=.35,clearance_at=lambda _:2.),floor=0.,live=Path(folder))
            executor.last_health=time.monotonic()
            executor.guard('same',np.array([.35,0.]))
            with self.assertRaisesRegex(RuntimeError,'实时点云前方有障碍'):
                executor.guard('same',np.array([.35,0.]),yaw_rate=.25)

    def test_reduced_static_margin_keeps_live_rotation_check(self):
        executor=object.__new__(control.Executor);now=time.time();clearance=[.53]
        state={'session_id':'same','odometry_received_unix_s':now,'cloud_received_unix_s':now,
               'T_office_from_current':np.eye(4).tolist()}
        with tempfile.TemporaryDirectory() as folder,patch.object(control,'STOPFILE',Path(folder)/'stop'):
            def cloud(points):
                np.savez(Path(folder)/'clouds.npz',session_id='same',received_unix_s=now,latest_xyz=np.asarray(points).reshape(-1,3))
            cloud([])
            executor.w=SimpleNamespace(cancel=threading.Event(),snapshot=lambda:(state,np.eye(4)),
                planner=SimpleNamespace(radius=.30,clearance_at=lambda _:clearance[0]),floor=0.,live=Path(folder))
            executor.last_health=time.monotonic()
            executor.guard('same',np.zeros(2),turning=True)
            clearance[0]=.47
            with self.assertRaisesRegex(RuntimeError,'静态净空'):
                executor.guard('same',np.zeros(2),turning=True)
            clearance[0]=.53
            cloud([[.52,-.01,1.],[.52,0.,1.],[.52,.01,1.]])
            with self.assertRaisesRegex(RuntimeError,'转身范围内有实时障碍'):
                executor.guard('same',np.zeros(2),turning=True)

    def test_new_session_refused(self):
        executor=object.__new__(control.Executor)
        executor.w=SimpleNamespace(cancel=threading.Event(),snapshot=lambda:({'session_id':'new'},np.eye(4)))
        with tempfile.TemporaryDirectory() as folder,patch.object(control,'STOPFILE',Path(folder)/'stop'):
            with self.assertRaisesRegex(RuntimeError,'会话'):executor.guard('old',np.array([.08,0]))

if __name__=='__main__':unittest.main()
