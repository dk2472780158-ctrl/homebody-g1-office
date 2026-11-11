import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from g1_readonly_geometry import StaticRoute
from test_turning import TurningChecks
import g1_real_control as control
sys.path.insert(0,str(Path(__file__).resolve().parent/'sensors'))
from sensor_continuity import SensorContinuity,ContinuityError,BacklogGuard

class NavigationFeedbackChecks(unittest.TestCase):
    def test_shelf_arrival_keeps_room_for_next_navigation(self):
        planner=StaticRoute('real-office-navigation.npz')
        requested=np.array([2.125,.575])
        path,detail=planner.navigation_route([-.9,-1.4],requested)
        self.assertGreater(detail['goal_adjustment_m'],.01)
        self.assertLessEqual(detail['goal_adjustment_m'],.65)
        self.assertGreater(planner.clearance_at(path[-1]),.68)
        # Any pose inside the arrival tolerance still clears the next turn.
        for angle in np.linspace(0,2*math.pi,32):
            stopped=path[-1]+.12*np.array([math.cos(angle),math.sin(angle)])
            self.assertGreater(planner.clearance_at(stopped),control.TURN_CLEARANCE)
            self.assertTrue(planner.free[planner.cell(stopped)])
        next_path,_=planner.navigation_route(path[-1],[2.75,-2.575])
        self.assertTrue(all(planner.segment_free(a,b) for a,b in zip(next_path[:-1],next_path[1:])))

    def test_no_departable_stop_blocks_before_entering_tight_target(self):
        planner=StaticRoute('real-office-navigation.npz')
        with self.assertRaisesRegex(ValueError,'再次转向'):
            planner.departure_goal([-3.975,-2.125],clearance=2.)
        self.assertFalse(planner.segment_free([0.,-1.],[.575,1.175]))

    def test_window_and_northwest_grid_centres_keep_existing_clearance_margin(self):
        planner=StaticRoute('real-office-navigation.npz')
        for goal in ([-3.975,-2.125],[-2.575,.375]):
            stop=planner.departure_goal(goal)
            self.assertLessEqual(np.linalg.norm(stop-goal),.65)
            self.assertGreater(planner.clearance_at(stop),.68)
            for angle in np.linspace(0,2*math.pi,16):
                stopped=stop+.12*np.array([math.cos(angle),math.sin(angle)])
                self.assertGreater(planner.clearance_at(stopped),control.TURN_CLEARANCE)
        path,_=planner.navigation_route([2.5926,-2.7495],[-3.975,-2.125])
        reverse,_=planner.navigation_route(path[-1],[2.75,-2.575])
        for xy in (path,reverse):
            self.assertTrue(all(planner.segment_free(a,b) for a,b in zip(xy[:-1],xy[1:])))

    def test_arrival_in_tight_space_is_not_reported_as_complete(self):
        from types import SimpleNamespace
        e,clock,commands,stops=TurningChecks().executor()
        e.w.planner=SimpleNamespace(clearance_at=lambda _: .47)
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[1.,0.]},'route_xy_m':[[0.,0.],[1.,0.]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertLessEqual(records[0]['goal_error_m'],.12)
        self.assertFalse(records[0]['departure_ready'])
        self.assertFalse(records[0]['target_reached'])

    def test_residual_eight_degrees_hands_off_to_sustained_forward_start(self):
        e,clock,commands,stops=TurningChecks().executor(yaw=math.radians(8))
        velocity=e.client.velocity;zero=e.client.zero;first=[None];events=[]
        def slow_start(vx,vy,wz,cancel,**kwargs):
            events.append(('command',clock[0],vx,vy,wz))
            if vx>0 and first[0] is None:first[0]=clock[0]
            ready=first[0] is not None and clock[0]-first[0]>=.45
            return velocity(vx if ready else 0.,vy,wz,cancel,**kwargs)
        def stop():
            events.append(('stop',clock[0]));first[0]=None;return zero()
        e.client.velocity=slow_start;e.client.zero=stop
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[1.5,0.]},'route_xy_m':[[0.,0.],[1.5,0.]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(records[0]['target_reached'],records)
        walking=[event for event in events if event[0]=='command' and event[2]>0]
        self.assertGreater(len(walking),6)
        self.assertTrue(all(event[4]==0 for event in walking if event[1]-walking[0][1]<.6-1e-8))
        self.assertFalse(any(event[0]=='command' and event[2]==0 for event in events))
        self.assertGreater(e.forward_commands,0)

    def test_swaying_without_forward_progress_times_out(self):
        e,clock,commands,stops=TurningChecks().executor()
        actual=e.w.snapshot;velocity=e.client.velocity
        def sway():
            state,pose=actual();pose[1,3]=.05*math.sin(6*clock[0]);return state,pose
        e.w.snapshot=sway;e.guard=lambda *a,**kw:sway()[1]
        e.client.velocity=lambda vx,vy,wz,cancel,**kw:velocity(0.,0.,0.,cancel,**kw)
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[2.,0.]},'route_xy_m':[[0.,0.],[2.,0.]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]),self.assertRaisesRegex(RuntimeError,'沿路线的有效前进'):
            e.run(plan,'same')
        self.assertLess(clock[0],6.)
        self.assertTrue(stops)

    def test_walking_corrects_small_yaw_disturbance_without_midroute_stops(self):
        e,clock,commands,stops=TurningChecks().executor()
        velocity=e.client.velocity;zero=e.client.zero;events=[]
        def disturbed(vx,vy,wz,cancel,**kwargs):
            events.append(('move',clock[0],vx,vy,wz))
            return velocity(vx,vy,wz+(.12 if .5<clock[0]<1.8 else 0.),cancel,**kwargs)
        def stop():events.append(('stop',clock[0]));return zero()
        e.client.velocity=disturbed;e.client.zero=stop
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[2.,0.]},
               'route_xy_m':[[0.,0.],[.8,0.],[2.,0.]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        walking=[i for i,event in enumerate(events) if event[0]=='move' and event[2]>0]
        self.assertTrue(records[0]['target_reached'],records)
        self.assertTrue(any(vx>0 and abs(wz)>.01 for vx,vy,wz in commands))
        self.assertFalse(any(event[0]=='stop' for event in events[walking[0]:walking[-1]+1]),events)
        self.assertTrue(all(vx==control.SPEED and vy==0 for vx,vy,wz in commands))

    def test_collision_projection_follows_commanded_curve(self):
        travel=control.projected_travel([0.,0.],[.35,0.],.25)
        np.testing.assert_allclose(travel[-1],[.35/.25*math.sin(.2),.35/.25*(1-math.cos(.2))])
        self.assertGreater(travel[-1,1],.025)
        straight=control.projected_travel([1.,2.],[.35,0.],0.)
        np.testing.assert_allclose(straight[-1],[1.28,2.])

    def test_sparse_cloud_does_not_cut_initialized_imu_stream(self):
        guard=SensorContinuity()
        guard.observe('cloud',10.,0.);guard.observe('imu',10.,0.)
        self.assertFalse(guard.can_forward(0.))
        for i in range(1,11):
            guard.observe('imu',10.+i*.1,i*.1)
            if i in (4,8):guard.observe('cloud',10.+i*.1,i*.1)
        self.assertTrue(guard.can_forward(1.))
        guard.observe('imu',11.4,1.4)
        self.assertFalse(guard.ready(1.4))
        self.assertTrue(guard.can_forward(1.4))
        with self.assertRaisesRegex(ContinuityError,'timeout'):
            guard.check_idle(3.)

    def test_door_route_turns_then_walks_with_delayed_feedback(self):
        e,clock,commands,stops=TurningChecks().executor(yaw=math.radians(120))
        actual=e.w.snapshot;history=[]
        def delayed():
            state,pose=actual();history.append((clock[0],pose.copy()))
            old=[p for at,p in history if at<=clock[0]-.6]
            return state,(old[-1] if old else history[0][1]).copy()
        e.w.snapshot=delayed;e.guard=lambda *a,**kw:delayed()[1]
        e.observe_after_stop=lambda _:actual()[1]
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[2.35,-1.86]},
               'route_xy_m':[[0.,0.],[2.35,-1.86]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):
            records=e.run(plan,'same')
        self.assertTrue(any(wz for vx,vy,wz in commands))
        self.assertTrue(any(vx==control.SPEED for vx,vy,wz in commands))
        self.assertTrue(records[0]['target_reached'],records)

    def test_arriving_old_frames_cannot_pass_as_realtime(self):
        guard=BacklogGuard()
        guard.observe('cloud',100.,1000.)
        guard.observe('cloud',100.5,1000.6)
        with self.assertRaisesRegex(ContinuityError,'backlog'):
            guard.observe('cloud',101.,1002.)

    def test_fixed_clock_offset_is_preserved_and_accepted(self):
        guard=BacklogGuard()
        for i in range(50):self.assertLess(guard.observe('imu',i*.01,i*.01+55.),1e-8)

    def test_ninety_degree_turn_brakes_for_delayed_pose_feedback(self):
        e,clock,commands,stops=TurningChecks().executor()
        actual=e.w.snapshot;history=[]
        def delayed():
            state,pose=actual();history.append((clock[0],pose.copy()))
            old=[p for at,p in history if at<=clock[0]-.6]
            return state,(old[-1] if old else history[0][1]).copy()
        e.w.snapshot=delayed
        e.guard=lambda *a,**kw:delayed()[1]
        def final_pose(session):
            e.client.zero();e.w.cancel.wait(.8)
            return actual()[1]
        e.observe_after_stop=final_pose
        plan=[{'skill':'turn_by','arguments':{'direction':'left','angle_deg':90},
               'start_yaw_rad':0.,'target_yaw_rad':math.pi/2,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(records[0]['target_reached'],records)

    def test_collinear_cells_become_one_segment_without_crossing_obstacles(self):
        planner=StaticRoute('real-office-navigation.npz')
        path,info=planner.route([.85,-.9],[2.75,-2.575])
        self.assertLess(len(path),10)
        for a,b in zip(path[:-1],path[1:]):self.assertTrue(planner.segment_free(a,b))
        self.assertGreater(planner.clearance_at([2.75,-2.575]),.55)
        # Northeast table was missing in the old PGM, but must block routes now.
        self.assertLess(planner.clearance_at([.575,1.175]),.35)

    def test_long_segment_navigation_counts_only_verified_arrival(self):
        e,clock,commands,stops=TurningChecks().executor()
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[1.,0.]},'route_xy_m':[[0.,0.],[1.,0.]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(records[0]['target_reached'])
        self.assertLessEqual(records[0]['goal_error_m'],.12)
        self.assertTrue(all(vx>=0 and vy==0 for vx,vy,wz in commands))

    def test_motion_away_after_stop_cannot_count_as_arrival(self):
        e,clock,commands,stops=TurningChecks().executor()
        after=np.eye(4);after[0,3]=1.4
        e.observe_after_stop=lambda _:after
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[1.,0.]},'route_xy_m':[[0.,0.],[1.,0.]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertFalse(records[0]['target_reached'])
        self.assertTrue(records[0]['motion_observed'])

    def test_isolated_reordered_imu_is_dropped_without_rewriting_time(self):
        guard=SensorContinuity()
        self.assertTrue(guard.observe('imu',10.,0.))
        self.assertFalse(guard.observe('imu',9.98,.02))
        self.assertEqual(guard.last['imu'],(10.,0.))
        self.assertTrue(guard.observe('imu',10.02,.04))
        self.assertEqual(guard.dropped_reordered_imu,1)

    def test_clock_reset_and_repeated_regression_still_fail(self):
        guard=SensorContinuity();guard.observe('imu',10.,0.)
        with self.assertRaises(ContinuityError):guard.observe('imu',9.,.02)
        guard=SensorContinuity();guard.observe('imu',10.,0.)
        guard.observe('imu',9.98,.02);guard.observe('imu',9.99,.04)
        with self.assertRaises(ContinuityError):guard.observe('imu',9.98,.06)

if __name__=='__main__':unittest.main()
