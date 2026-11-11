import ast,json,math,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import g1_real_control as control
from test_dynamic_navigation import DynamicNavigationChecks
from test_turning import TurningChecks
from office_live_panel import display_state

class AMCLTaskRecoveryChecks(unittest.TestCase):
    def setUp(self):
        self.maps=DynamicNavigationChecks();self.maps.setUp();self.addCleanup(self.maps.doCleanups)

    def fixture(self,unavailable=lambda t:False):
        e,clock,commands,stops=TurningChecks().executor();snapshot=e.w.snapshot
        e.w.cancel.is_set=lambda:False;e.w.planner=self.maps.planner;e.last_health=0.;e.localization_waits=0
        sent=[];velocity=e.client.velocity
        def send(*args,**kwargs):
            sent.append((clock[0],args[:3]));return velocity(*args,**kwargs)
        e.client.velocity=send
        def state():return {'session_id':'same','status':'matching','localization_backend':'nav2_amcl',
            'updated_unix_s':100.+clock[0],'cloud_received_unix_s':100.+clock[0],
            'odometry_received_unix_s':100.+clock[0]}
        def current():
            if unavailable(clock[0]):raise control.LocalizationUnavailable('AMCL validation',state())
            return state(),snapshot()[1]
        def observe(session,state,pose):
            if getattr(e,'dynamic_obstacles',None) is not None:
                e.dynamic_obstacles.observe([],100.+clock[0],100.+clock[0],session)
            return np.empty((0,3))
        e.w.snapshot=current;e.live_obstacles=observe;e.guard=control.Executor.guard.__get__(e)
        return e,clock,commands,stops,sent,state

    def timers(self,clock):
        return (patch.object(control.time,'monotonic',lambda:clock[0]),
                patch.object(control.time,'time',lambda:100.+clock[0]))

    def test_mid_task_validation_waits_without_velocity_replans_and_reaches_same_goal(self):
        e,clock,commands,stops,sent,state=self.fixture(lambda t:.7<t<1.5)
        first,second=self.timers(clock)
        with first,second:
            records=e.run([{'skill':'navigate','route_xy_m':[[0.,0.],[2.,0.]]}],'same')
        self.assertTrue(records[0]['target_reached'],records)
        self.assertEqual(e.localization_waits,1);self.assertEqual(len(e.replans),1)
        self.assertFalse(any(.7<t<2.3 for t,cmd in sent))
        np.testing.assert_array_equal(e.replans[0]['goal_xy_m'],[2.,0.])

    def test_same_session_fresh_matching_can_wait_but_timeout_never_moves(self):
        e,clock,commands,stops,sent,state=self.fixture(lambda _:True)
        first,second=self.timers(clock)
        with first,second,self.assertRaisesRegex(RuntimeError,'30 秒'):
            e.wait_for_localization('same',np.eye(4),100.,control.LocalizationUnavailable('matching',state()))
        self.assertLess(clock[0],30.2);self.assertTrue(stops);self.assertEqual(commands,[])

    def test_cancel_new_session_stale_control_and_large_relocation_remain_stops(self):
        for case in ('cancel','session','stale','control','jump'):
            with self.subTest(case=case):
                e,clock,commands,stops,sent,state=self.fixture()
                initial=state()
                if case=='cancel':e.w.cancel.is_set=lambda:True
                if case=='session':initial['session_id']='different'
                if case=='stale':initial['cloud_received_unix_s']=98.
                if case=='control':clock[0]=1.;e.client.health=lambda:{'armed':False}
                if case=='jump':
                    pose=np.eye(4);pose[0,3]=1.;e.w.snapshot=lambda:(state(),pose)
                first,second=self.timers(clock)
                with first,second,self.assertRaises(RuntimeError):
                    e.wait_for_localization('same',np.eye(4),100.,control.LocalizationUnavailable('matching',initial))
                self.assertEqual(commands,[]);self.assertTrue(stops)

    def test_timed_motion_is_not_automatically_replayed(self):
        e,clock,commands,stops,sent,state=self.fixture(lambda t:t>.2)
        first,second=self.timers(clock)
        with first,second,self.assertRaises(control.LocalizationUnavailable):
            e.run([{'skill':'move_for','arguments':{'direction':'forward','duration_s':2.},
                    'route_xy_m':[[0.,0.],[.7,0.]]}],'same')
        self.assertEqual(e.localization_waits,0);self.assertTrue(stops)

    def test_wait_count_bounded(self):
        e,clock,commands,stops,sent,state=self.fixture();e.localization_waits=3
        with self.assertRaisesRegex(RuntimeError,'反复中断'):
            e.wait_for_localization('same',np.eye(4),100.,control.LocalizationUnavailable('matching',state()))
        self.assertTrue(stops);self.assertEqual(commands,[])

    def test_snapshot_types_only_amcl_matching_as_recoverable(self):
        tree=ast.parse(Path('office_real_workflow.py').read_text(encoding='utf-8'))
        method=next(n for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='RealWorkflow'
                    for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='snapshot')
        ns=dict(json=json,np=np,time=control.time,display_state=display_state,LocalizationUnavailable=control.LocalizationUnavailable)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'<snapshot>','exec'),ns)
        with tempfile.TemporaryDirectory() as folder:
            live=Path(folder);(live/'session-id').write_text('same');(live/'real-view-state.json').write_text('{}')
            from types import SimpleNamespace
            w=SimpleNamespace(live=live)
            for status,error in [('matching',control.LocalizationUnavailable),('waiting',ValueError)]:
                (live/'live-state.json').write_text(json.dumps(dict(status=status,localization_backend='nav2_amcl')))
                with self.assertRaises(error) as raised:ns['snapshot'](w)
                self.assertEqual(isinstance(raised.exception,control.LocalizationUnavailable),status=='matching')

if __name__=='__main__':unittest.main()
