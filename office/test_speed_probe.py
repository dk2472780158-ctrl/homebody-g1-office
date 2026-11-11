import ast
import math
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import g1_real_control as control


def steps(duration=3.,direction='forward'):
    return [{'skill':'move_for','arguments':{'direction':direction,'duration_s':duration}}]


class SpeedProbeChecks(unittest.TestCase):
    def test_probe_cannot_extend_duration_or_navigate(self):
        for plan in [steps(2),steps(direction='east'),steps()+steps(),[{'skill':'navigate','arguments':{}}]]:
            with self.subTest(plan=plan),self.assertRaises(ValueError):control.task_speed(plan,True)

    def test_default_transport_accepts_point_three_five_and_rejects_excess(self):
        client=object.__new__(control.Teleop);client.lock=threading.Lock();calls=[]
        client.request=lambda *a,**k:calls.append(a) or {'ok':True}
        with self.assertRaises(ValueError):client.velocity(.36,0,0,threading.Event())
        self.assertEqual(calls,[])
        with tempfile.TemporaryDirectory() as d,patch.object(control,'STOPFILE',Path(d)/'stop'):
            client.velocity(.35,0,0,threading.Event())
            with self.assertRaises(ValueError):client.velocity(.36,0,0,threading.Event(),probe=True)
        self.assertEqual(len(calls),1)

    def test_probe_preserves_cancel_and_stop_latch(self):
        client=object.__new__(control.Teleop);client.lock=threading.Lock();calls=[]
        client.request=lambda *a,**k:calls.append(a) or {'ok':True}
        cancel=threading.Event();cancel.set()
        with self.assertRaises(RuntimeError):client.velocity(.35,0,0,cancel,probe=True)
        with tempfile.TemporaryDirectory() as d:
            stop=Path(d)/'stop';stop.touch()
            with patch.object(control,'STOPFILE',stop),self.assertRaises(RuntimeError):
                client.velocity(.35,0,0,threading.Event(),probe=True)
        self.assertEqual(calls,[])

    def prepare(self):
        tree=ast.parse(Path('office_real_workflow.py').read_text(encoding='utf-8'))
        method=next(m for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='RealWorkflow'
                    for m in c.body if isinstance(m,ast.FunctionDef) and m.name=='prepare')
        ns={'task_speed':control.task_speed,'pose_yaw':control.pose_yaw,'angle_diff':control.angle_diff,
            'np':np,'math':math,'PLACES':{},'COMPASS':{},'ALIASES':{},
            'TURN_RATE':control.TURN_RATE,'TURN_CLEARANCE':control.TURN_CLEARANCE,'TRAVEL_ALIGN':control.TRAVEL_ALIGN}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<prepare>','exec'),ns)
        return ns['prepare']

    def test_preview_checks_probe_distance_without_changing_default(self):
        inspected=[];pose=np.eye(4)
        w=SimpleNamespace(snapshot=lambda:({'session_id':'same'},pose),cancel=threading.Event(),
            planner=SimpleNamespace(radius=.35,clearance_at=lambda p:inspected.append(p.copy()) or 2.),
            floor=0.,route=SimpleNamespace(),goal=SimpleNamespace(),real_mode=True)
        prepared=self.prepare()(w,steps(),probe=True)
        self.assertAlmostEqual(prepared[0]['route_xy_m'][-1][0],1.05)
        self.assertAlmostEqual(max(p[0] for p in inspected),1.05)
        normal=self.prepare()(w,steps())
        self.assertAlmostEqual(normal[0]['route_xy_m'][-1][0],1.05)
        two_seconds=self.prepare()(w,steps(2))
        self.assertAlmostEqual(two_seconds[0]['route_xy_m'][-1][0],.7)
        self.assertIn('0.35 m/s',two_seconds[0]['description'])

    def test_invalid_probe_rejected_before_health_or_commands(self):
        executor=object.__new__(control.Executor)
        executor.client=SimpleNamespace(health=lambda:self.fail('invalid probe reached robot'))
        with self.assertRaises(ValueError):executor.run(steps(2),'session',probe=True)

    def test_three_second_probe_uses_same_guard_and_always_zeroes(self):
        clock=[0.];commands=[];stops=[];guarded=[];pose=np.eye(4)
        cancel=SimpleNamespace(wait=lambda seconds:clock.__setitem__(0,round(clock[0]+seconds,9)),is_set=lambda:False)
        w=SimpleNamespace(cancel=cancel,snapshot=lambda:({'session_id':'session','odometry_received_unix_s':control.time.time(),'cloud_received_unix_s':control.time.time()},pose))
        executor=object.__new__(control.Executor);executor.w=w
        executor.client=SimpleNamespace(
            health=lambda:{'armed':True,'ros':True,'vel_owner':{'owner':'teleop'}},
            velocity=lambda vx,vy,wz,cancel,**kw:commands.append((vx,vy,wz,kw)),zero=lambda:stops.append(True))
        executor.guard=lambda session,v:guarded.append(v.copy()) or pose
        plan=steps();plan[0]['route_xy_m']=[[0.,0.],[1.05,0.]]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=executor.run(plan,'session',probe=True)
        self.assertTrue(commands);self.assertEqual(len(commands),len(guarded))
        self.assertTrue(all(abs(c[0]-.35)<1e-9 and c[3]['probe'] for c in commands))
        self.assertAlmostEqual(clock[0],4.5);self.assertEqual(stops,[True,True])
        self.assertTrue(records[0]['speed_probe']);self.assertFalse(records[0]['motion_observed'])


if __name__=='__main__':unittest.main()
