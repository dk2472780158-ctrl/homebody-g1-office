import tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from g1_readonly_geometry import StaticRoute
from g1_dynamic_navigation import DynamicObstacleLayer,ObstacleBlocked,validate_route_turns
from g1_pointcloud_slice import height_slice,obstacle_slice,scan_projection
import g1_real_control as control
from test_turning import TurningChecks

class DynamicNavigationChecks(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        file=Path(self.folder.name)/'map.npz'
        occupied=np.zeros((121,141),bool);occupied[[0,-1],:]=True;occupied[:,[0,-1]]=True
        np.savez(file,occupancy=occupied,T_map_px=np.array([[.05,0,-2.],[0,.05,-3.],[0,0,1.]]))
        self.planner=StaticRoute(file)

    def test_new_obstacle_creates_detour_without_changing_target_or_static_map(self):
        before=self.planner.occupied.copy();clearance=self.planner.clearance.copy()
        layer=DynamicObstacleLayer(self.planner,'same');layer.observe([[1.2,0.]],100.,100.,'same')
        planner=layer.planner(100.);path,_=planner.route([0.,0.],[3.,0.],interior_clearance=.53)
        self.assertGreater(len(path),2)
        self.assertGreater(np.max(np.abs(path[:,1])),.4)
        np.testing.assert_allclose(path[-1],[3.,0.])
        self.assertTrue(all(planner.segment_free(a,b) for a,b in zip(path[:-1],path[1:])))
        validate_route_turns(planner,path,0.,.48,.38,control.TRAVEL_ALIGN)
        np.testing.assert_array_equal(before,self.planner.occupied)
        np.testing.assert_array_equal(clearance,self.planner.clearance)
    def test_obstacle_expiry_requires_fresh_observation(self):
        layer=DynamicObstacleLayer(self.planner,'same');layer.observe([[1.2,0.]],100.,100.,'same')
        layer.observe([],102.,102.,'same');self.assertFalse(layer.planner(102.).segment_free([0,0],[3,0]))
        layer.observe([],103.1,103.1,'same');self.assertTrue(layer.planner(103.1).segment_free([0,0],[3,0]))
        with self.assertRaisesRegex(RuntimeError,'过期'):layer.planner(104.)
        with self.assertRaisesRegex(RuntimeError,'会话'):layer.observe([],104.,104.,'new')
    def test_wall_cannot_be_crossed_or_diagonally_cut(self):
        layer=DynamicObstacleLayer(self.planner,'same')
        layer.observe(np.c_[np.full(121,1.2),np.linspace(-3,3,121)],100.,100.,'same')
        with self.assertRaises(ValueError):layer.planner(100.).route([0,0],[3,0],interior_clearance=.53)

    def test_safe_narrow_start_connects_to_wider_corridor(self):
        path,_=self.planner.route([-1.5,0.],[3.,0.],interior_clearance=.53)
        np.testing.assert_array_equal(path[0],[-1.5,0.])
        self.assertTrue(all(self.planner.segment_free(a,b) for a,b in zip(path[:-1],path[1:])))
    def test_new_observed_free_ray_clears_old_obstacle_without_erasing_static(self):
        layer=DynamicObstacleLayer(self.planner,'same')
        layer.observe([[1.,0.],[1.,1.]],100.,100.,'same')
        ahead=self.planner.cell([1,0]);aside=self.planner.cell([1,1])
        layer.clear_observed_rays([[2.,0.]],[0,0],100.2)
        self.assertNotIn(ahead,layer.cells);self.assertIn(aside,layer.cells)
        self.assertTrue(self.planner.occupied[0,0])
    def test_nearer_return_or_missing_return_does_not_clear_hidden_obstacle(self):
        layer=DynamicObstacleLayer(self.planner,'same');layer.observe([[1.,0.]],100.,100.,'same')
        cell=self.planner.cell([1,0])
        layer.clear_observed_rays([[2.,0.],[.8,0.]],[0,0],100.2)
        self.assertIn(cell,layer.cells)
        layer.clear_observed_rays([],[0,0],100.3);self.assertIn(cell,layer.cells)

    def fixture(self,obstacles):
        e,clock,commands,stops=TurningChecks().executor()
        e.w.cancel.is_set=lambda:False;e.w.planner=self.planner;e.replans=[]
        e.dynamic_obstacles=DynamicObstacleLayer(self.planner,'same');e.last_health=0.
        e.w.snapshot=lambda:({'session_id':'same','odometry_received_unix_s':100.+clock[0],
            'cloud_received_unix_s':100.+clock[0]},np.eye(4))
        def observe(session,state,pose):
            e.dynamic_obstacles.observe(obstacles(clock[0]),100.+clock[0],100.+clock[0],session)
            return np.empty((0,3))
        e.live_obstacles=observe
        return e,clock,commands,stops
    def test_replan_zeroes_first_and_does_not_send_movement(self):
        e,clock,commands,stops=self.fixture(lambda _: [[1.2,0.]])
        observe=e.live_obstacles
        def checked(*args):self.assertTrue(stops);return observe(*args)
        e.live_obstacles=checked
        with patch.object(control.time,'monotonic',lambda:clock[0]),patch.object(control.time,'time',lambda:100.+clock[0]):
            path=e.replan('same',np.array([3.,0.]),50.,ObstacleBlocked('obstacle'))
        self.assertGreater(len(path),2);self.assertEqual(commands,[])
        self.assertEqual(e.replans[0]['status'],'planned')
    def test_no_path_wait_is_bounded_and_remains_stopped(self):
        wall=np.c_[np.full(121,1.2),np.linspace(-3,3,121)]
        e,clock,commands,stops=self.fixture(lambda _:wall)
        with patch.object(control.time,'monotonic',lambda:clock[0]),patch.object(control.time,'time',lambda:100.+clock[0]):
            with self.assertRaisesRegex(RuntimeError,'无可行路线'):e.replan('same',np.array([3.,0.]),100.,ObstacleBlocked('obstacle'))
        self.assertLess(clock[0],8.3);self.assertTrue(stops);self.assertEqual(commands,[])
        self.assertEqual(e.replans[0]['status'],'no_path')
    def test_blocked_goal_waits_until_obstacle_clears_and_keeps_goal(self):
        e,clock,commands,stops=self.fixture(lambda t:[[3.,0.]] if t<.5 else [])
        with patch.object(control.time,'monotonic',lambda:clock[0]),patch.object(control.time,'time',lambda:100.+clock[0]):
            path=e.replan('same',np.array([3.,0.]),50.,ObstacleBlocked('goal blocked'))
        np.testing.assert_array_equal(path[-1],[3,0]);self.assertGreater(clock[0],3.)
        self.assertEqual(commands,[])
    def test_cancel_session_change_stale_data_never_resume(self):
        for case in ('cancel','session','stale','control'):
            e,clock,commands,stops=self.fixture(lambda _:[])
            if case=='cancel':e.w.cancel.is_set=lambda:True
            if case=='session':e.w.snapshot=lambda:({'session_id':'new'},np.eye(4))
            if case=='stale':e.w.snapshot=lambda:({'session_id':'same','odometry_received_unix_s':98.},np.eye(4))
            if case=='control':e.client.health=lambda:{'armed':False}
            with patch.object(control.time,'monotonic',lambda:1.+clock[0]),patch.object(control.time,'time',lambda:100.+clock[0]):
                with self.assertRaises(RuntimeError):e.replan('same',np.array([3.,0.]),50.,ObstacleBlocked('obstacle'))
            self.assertTrue(stops);self.assertEqual(commands,[])

    def test_settling_pose_discards_old_route_and_replans_without_movement(self):
        e,clock,commands,stops=self.fixture(lambda _:[])
        snapshot=e.w.snapshot;calls=[0]
        def settling():
            state,pose=snapshot();calls[0]+=1
            if calls[0]>7:pose[0,3]=.06
            return state,pose
        e.w.snapshot=settling
        with patch.object(control.time,'monotonic',lambda:clock[0]),patch.object(control.time,'time',lambda:100.+clock[0]):
            path=e.replan('same',np.array([3.,0.]),50.,ObstacleBlocked('obstacle'))
        self.assertEqual(commands,[]);self.assertTrue(stops)
        self.assertEqual(e.replans[0]['status'],'planned')
        np.testing.assert_allclose(path[0],[.06,0.])

    def test_navigation_resumes_forward_after_obstacle_recovery(self):
        e,clock,commands,stops=TurningChecks().executor();guard=e.guard;triggered=[];paths=[]
        def obstruction(session,velocity,**kwargs):
            if not triggered and clock[0]>.5:
                triggered.append(True);raise ObstacleBlocked('new obstacle')
            return guard(session,velocity,**kwargs)
        def recovery(session,goal,deadline,reason):
            e.client.zero();_,pose=e.w.snapshot();paths.append(goal.copy())
            return np.array([pose[:2,3],goal])
        e.guard=obstruction;e.replan=recovery
        with patch.object(control.time,'monotonic',lambda:clock[0]):
            records=e.run([{'skill':'navigate','route_xy_m':[[0.,0.],[1.5,0.]]}],'same')
        self.assertTrue(records[0]['target_reached']);self.assertEqual(len(paths),1)
        self.assertTrue(all(vx>=0 and vy==0 for vx,vy,wz in commands))

    def test_complete_executor_stops_detours_and_reaches_original_goal(self):
        e,clock,commands,stops=TurningChecks().executor();snapshot=e.w.snapshot
        e.w.cancel.is_set=lambda:False;e.w.planner=self.planner
        e.w.snapshot=lambda:({'session_id':'same','cloud_received_unix_s':100.+clock[0],
            'odometry_received_unix_s':100.+clock[0]},snapshot()[1])
        obstacles=np.array([[1.2,y,1.] for y in (-.01,0.,.01)])
        def observe(session,state,pose):
            e.dynamic_obstacles.observe(obstacles[:,:2],100.+clock[0],100.+clock[0],session)
            return obstacles
        e.live_obstacles=observe;e.guard=control.Executor.guard.__get__(e)
        with patch.object(control.time,'monotonic',lambda:clock[0]),patch.object(control.time,'time',lambda:100.+clock[0]):
            records=e.run([{'skill':'navigate','route_xy_m':[[0.,0.],[3.,0.]]}],'same')
        self.assertTrue(records[0]['target_reached'],records)
        self.assertGreaterEqual(len(e.replans),1)
        self.assertTrue(all(vx>=0 and vy==0 for vx,vy,wz in commands))

    def test_navigation_lookahead_does_not_check_beyond_the_next_stop(self):
        e,clock,commands,stops=TurningChecks().executor();snapshot=e.w.snapshot
        e.w.cancel.is_set=lambda:False;e.w.planner=self.planner
        e.w.snapshot=lambda:({'session_id':'same','cloud_received_unix_s':100.+clock[0],
            'odometry_received_unix_s':100.+clock[0]},snapshot()[1])
        # Obstacle is beyond the reviewed goal, with .7 m safe stop margin.
        obstacles=np.array([[2.7,y,1.] for y in (-.01,0.,.01)])
        def observe(session,state,pose):
            e.dynamic_obstacles.observe(obstacles[:,:2],100.+clock[0],100.+clock[0],session)
            return obstacles
        e.live_obstacles=observe;e.guard=control.Executor.guard.__get__(e)
        with patch.object(control.time,'monotonic',lambda:clock[0]),patch.object(control.time,'time',lambda:100.+clock[0]):
            records=e.run([{'skill':'navigate','route_xy_m':[[0.,0.],[2.,0.]]}],'same')
        self.assertTrue(records[0]['target_reached']);self.assertEqual(e.replans,[])

class PointCloudSliceChecks(unittest.TestCase):
    def test_floor_ceiling_self_returns_and_far_points_are_removed(self):
        xyz=np.array([[1,0,-1.26],[1,0,-1.16],[1,0,-.8],[1,0,.7],[.2,0,-.8],[13,0,-.8]])
        points=obstacle_slice(xyz,-1.26,[0,0])
        np.testing.assert_array_equal(points,[[1,0,-.8]])
    def test_projection_chooses_nearest_and_keeps_unknown_bins_infinite(self):
        points=np.array([[2,0,1],[1,0,.8],[0,2,.7]])
        cut,scan=scan_projection(points,[0,0],0.)
        self.assertEqual(len(cut),2);self.assertEqual(np.isfinite(scan).sum(),2)
        self.assertTrue(np.all(cut[:,2]==.08));self.assertEqual(scan[np.isfinite(scan)].min(),1.)
    def test_invalid_cloud_is_not_silently_turned_into_free_space(self):
        with self.assertRaises(ValueError):height_slice(np.array([[np.nan,0,1]]),0.)

if __name__=='__main__':unittest.main()
