import math,unittest
from unittest.mock import patch
import numpy as np
import g1_real_control as control
from test_dynamic_navigation import DynamicNavigationChecks
from test_turning import TurningChecks

class PoseCorrectionChecks(unittest.TestCase):
    def setUp(self):
        self.maps=DynamicNavigationChecks();self.maps.setUp();self.addCleanup(self.maps.doCleanups)

    def fixture(self):
        e,clock,commands,stops=self.maps.fixture(lambda _:[])
        e.guard=control.Executor.guard.__get__(e)
        return e,clock,commands,stops

    def timers(self,clock):
        return (patch.object(control.time,'monotonic',lambda:clock[0]),
                patch.object(control.time,'time',lambda:100.+clock[0]))

    def test_small_deviation_stops_replans_and_reaches_original_goal(self):
        e,clock,commands,stops=TurningChecks().executor();snapshot=e.w.snapshot
        e.w.cancel.is_set=lambda:False;e.w.planner=self.maps.planner
        def corrected():
            state,pose=snapshot()
            if clock[0]>.7:pose[1,3]+=.21
            state.update(cloud_received_unix_s=100.+clock[0],odometry_received_unix_s=100.+clock[0])
            return state,pose
        def observe(session,state,pose):
            e.dynamic_obstacles.observe([],100.+clock[0],100.+clock[0],session)
            return np.empty((0,3))
        e.w.snapshot=corrected;e.live_obstacles=observe;e.guard=control.Executor.guard.__get__(e)
        e.observe_after_stop=lambda _:corrected()[1]
        first,second=self.timers(clock)
        with first,second:
            records=e.run([{'skill':'navigate','route_xy_m':[[0.,0.],[2.,0.]]}],'same')
        self.assertTrue(records[0]['target_reached'],records)
        self.assertEqual(len(e.replans),1)
        self.assertEqual(e.replans[0]['kind'],'pose_correction')
        np.testing.assert_allclose(e.replans[0]['goal_xy_m'],[2.,0.])
        self.assertTrue(all(vx>=0 and vy==0 for vx,vy,wz in commands))

    def test_large_deviation_still_stops_without_replanning(self):
        e,clock,commands,stops=TurningChecks().executor();snapshot=e.w.snapshot
        def corrected():
            state,pose=snapshot()
            if clock[0]>.7:pose[1,3]+=.6
            return state,pose
        e.w.snapshot=corrected
        with patch.object(control.time,'monotonic',lambda:clock[0]):
            with self.assertRaisesRegex(RuntimeError,'明显偏离'):
                e.run([{'skill':'navigate','route_xy_m':[[0.,0.],[2.,0.]]}],'same')
        self.assertTrue(stops);self.assertEqual(e.replans,[])

    def test_settling_requires_new_frames_and_sends_only_zero(self):
        e,clock,commands,stops=self.fixture();first,second=self.timers(clock)
        with first,second:
            e.settle_route_correction('same',control.RoutePoseCorrection('correction',np.eye(4)),10.)
        self.assertGreaterEqual(clock[0],.8);self.assertEqual(commands,[]);self.assertTrue(stops)

    def test_repeated_same_frame_cannot_establish_stability(self):
        e,clock,commands,stops=self.fixture();snapshot=e.w.snapshot
        e.w.snapshot=lambda:({'session_id':'same','odometry_received_unix_s':100.,
                             'cloud_received_unix_s':100.},snapshot()[1])
        first,second=self.timers(clock)
        with first,second,self.assertRaisesRegex(RuntimeError,'新鲜'):
            e.settle_route_correction('same',control.RoutePoseCorrection('correction',np.eye(4)),10.)
        self.assertEqual(commands,[])

    def test_unstable_pose_never_resumes(self):
        e,clock,commands,stops=self.fixture();snapshot=e.w.snapshot
        def unstable():
            state,pose=snapshot();pose[0,3]=.08 if int(clock[0]*10)%2 else 0.
            return state,pose
        e.w.snapshot=unstable;first,second=self.timers(clock)
        with first,second,self.assertRaisesRegex(RuntimeError,'未稳定'):
            e.settle_route_correction('same',control.RoutePoseCorrection('correction',np.eye(4)),10.)
        self.assertEqual(commands,[]);self.assertLess(clock[0],4.2)

    def test_collision_cancel_session_and_stale_remain_stops(self):
        for case in ('collision','cancel','session','stale'):
            with self.subTest(case=case):
                e,clock,commands,stops=self.fixture()
                if case=='collision':
                    e.w.planner.clearance_at=lambda _:.1
                if case=='cancel':e.w.cancel.is_set=lambda:True
                if case=='session':e.w.snapshot=lambda:({'session_id':'new'},np.eye(4))
                if case=='stale':e.w.snapshot=lambda:({'session_id':'same','cloud_received_unix_s':98.},np.eye(4))
                first,second=self.timers(clock)
                with first,second,self.assertRaises(RuntimeError):
                    e.replan('same',np.array([2.,0.]),10.,control.RoutePoseCorrection('correction',np.eye(4)))
                self.assertEqual(commands,[]);self.assertTrue(stops)

    def test_map_evidence_distinguishes_physical_motion_and_large_jump(self):
        old=np.eye(4);new=np.eye(4);new[0,3]=.2
        before={'T_office_from_current':old.tolist()};after={'T_office_from_current':new.tolist()}
        self.assertTrue(control.corroborated_map_correction(before,after,old,new))
        self.assertFalse(control.corroborated_map_correction(before,before,old,new))
        huge=new.copy();huge[0,3]=.6
        self.assertFalse(control.corroborated_map_correction(before,{'T_office_from_current':huge.tolist()},old,huge))

    def test_turn_jump_recovers_only_navigation_with_map_evidence(self):
        for evidence,navigation in ((False,True),(True,True),(True,False)):
            with self.subTest(evidence=evidence,navigation=navigation):
                e,clock,commands,stops=TurningChecks().executor();guard=e.guard
                e.w.planner=self.maps.planner;e.commands_sent=0
                def corrected(*args,**kwargs):
                    pose=guard(*args,**kwargs);transform=np.eye(4)
                    if clock[0]>.3:
                        pose[0,3]+=.2
                        if evidence:transform[0,3]=.2
                    e.last_guard_state={'T_office_from_current':transform.tolist()}
                    return pose
                e.guard=corrected
                e.w.snapshot=lambda:({'T_office_from_current':np.eye(4).tolist()},np.eye(4))
                exception=control.RoutePoseCorrection if evidence and navigation else RuntimeError
                with patch.object(control.time,'monotonic',lambda:clock[0]),self.assertRaises(exception) as raised:
                    e.rotate('same',0.,math.pi/2,1,np.zeros(2),target_xy=np.array([0.,2.]) if navigation else None)
                self.assertEqual(isinstance(raised.exception,control.RoutePoseCorrection),evidence and navigation)

    def test_correction_retries_are_bounded(self):
        e,clock,commands,stops=self.fixture();e.replans=[{'kind':'pose_correction'}]*3
        with self.assertRaisesRegex(RuntimeError,'反复修正'):
            e.replan('same',np.array([2.,0.]),100.,control.RoutePoseCorrection('correction',np.eye(4)))
        self.assertTrue(stops);self.assertEqual(commands,[])

if __name__=='__main__':unittest.main()
