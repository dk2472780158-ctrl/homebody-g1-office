import json
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import g1_real_control as control
import official_gesture_runner as runner
from office_landmarks import COMPASS_LABELS,approach_points,visible_marks
from test_turning import workflow_parts

ROOT=Path(__file__).resolve().parent

class LandmarkChecks(unittest.TestCase):
    def test_compass_hidden_but_navigation_and_aliases_preserved(self):
        points={label:[0,0] for label in (*COMPASS_LABELS,'门口')}
        shown=visible_marks(ROOT,points);goals=approach_points(ROOT,points)
        self.assertTrue(COMPASS_LABELS.isdisjoint(shown))
        self.assertTrue(COMPASS_LABELS.issubset(goals))
        parse=workflow_parts()['direct_steps']
        for text,skill in [('去西南','go_southwest'),('去桌子一旁边','go_table1'),('去桌子2','go_table2'),
                           ('去架子','go_shelf'),('去窗口','go_window'),('招手','wave'),('握手','handshake')]:
            self.assertEqual(parse(text),[{'skill':skill,'arguments':{}}])
        for label in ('桌子1','桌子2','架子','窗口'):
            self.assertIn(label,shown)
            self.assertNotEqual(shown[label],goals[label])

    def test_landmark_goals_are_free_in_static_map(self):
        import numpy as np
        from g1_readonly_geometry import StaticRoute
        planner=StaticRoute(ROOT/'office-map-runtime.npz')
        landmarks=json.loads((ROOT/'office-landmarks.json').read_text(encoding='utf-8'))['landmarks']
        for label,item in landmarks.items():
            self.assertTrue(planner.free[planner.cell(item['approach_xy_m'])],label)
            self.assertGreater(planner.clearance_at(item['approach_xy_m']),.55,label)

class RunnerChecks(unittest.TestCase):
    def arm(self,code=0):
        actions=[]
        def execute(action):actions.append(action);return code if action!=99 else 0
        return SimpleNamespace(ExecuteAction=execute),actions

    def test_official_ids_and_release(self):
        for name,action in [('wave',26),('handshake',27)]:
            arm,actions=self.arm();clock=[0.]
            with patch.object(runner.time,'monotonic',side_effect=lambda:clock[0]),patch.object(runner.time,'sleep',side_effect=lambda seconds:clock.__setitem__(0,clock[0]+seconds)):
                self.assertTrue(runner.execute(name,arm,lambda:False)['released'])
            self.assertEqual(actions,[action,99])

    def test_cancel_releases_and_rejection_never_switches_mode(self):
        arm,actions=self.arm()
        with self.assertRaisesRegex(RuntimeError,'cancelled before'):runner.execute('wave',arm,lambda:True)
        self.assertEqual(actions,[])
        checks=iter([False,True])
        with self.assertRaisesRegex(RuntimeError,'cancelled and arms released'):runner.execute('wave',arm,lambda:next(checks))
        self.assertEqual(actions,[26,99])
        arm,actions=self.arm(code=123)
        with self.assertRaisesRegex(RuntimeError,'rejected'):runner.execute('handshake',arm,lambda:False)
        self.assertEqual(actions,[27,99])

    def test_client_cancel_stops_started_job(self):
        client=object.__new__(control.Teleop);cancel=threading.Event();calls=[];started=[]
        def request(path,body=None,timeout=1.):
            calls.append(path)
            if path.endswith('/start'):cancel.set();return {'job_id':'test-job'}
            return {}
        client.request=request;client.zero=lambda:None
        with patch.object(control,'STOPFILE',ROOT/'nonexistent-test-stopfile'):
            with self.assertRaisesRegex(RuntimeError,'手势已停止'):
                client.gesture('wave',cancel,lambda:None,lambda:started.append(True))
        self.assertEqual(started,[True])
        self.assertEqual(calls,['/teleop/gestures/start','/teleop/gestures/stop'])

if __name__=='__main__':unittest.main()
