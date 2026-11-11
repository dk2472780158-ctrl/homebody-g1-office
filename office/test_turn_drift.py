import math,unittest
from unittest.mock import patch
import g1_real_control as control
from test_turning import TurningChecks

class TurnDriftChecks(unittest.TestCase):
    def executor(self,offset):
        e,clock,commands,stops=TurningChecks().executor();guard=e.guard
        def displaced(*args,**kwargs):
            pose=guard(*args,**kwargs);pose[0,3]=offset(control.pose_yaw(pose));return pose
        e.guard=displaced
        return e,clock,commands,stops
    def test_bounded_turning_shift_uses_measured_segment_centres(self):
        e,clock,commands,stops=self.executor(lambda yaw:.20*math.sin(yaw))
        plan=[{'skill':'turn_by','arguments':{'angle_deg':90.},'start_yaw_rad':0.,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):e.run(plan,'same')
        self.assertGreater(len(e.turn_segments),1)
        self.assertGreater(e.last_turn['peak_displacement_m'],.15)
        self.assertLessEqual(e.last_turn['segment_peak_displacement_m'],.25)
        self.assertTrue(all(vx==vy==0 for vx,vy,wz in commands))
    def test_excessive_drift_is_stopped_and_logged(self):
        e,clock,commands,stops=self.executor(lambda yaw:.8*math.sin(yaw))
        plan=[{'skill':'turn_by','arguments':{'angle_deg':90.},'start_yaw_rad':0.,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]),self.assertRaisesRegex(RuntimeError,'单段转向位移异常'):
            e.run(plan,'same')
        self.assertTrue(stops);self.assertGreater(e.last_turn['segment_displacement_m'],.25)
    def test_half_turn_with_28cm_total_support_foot_shift_completes(self):
        e,clock,commands,stops=self.executor(lambda yaw:.28*math.sin(yaw/2))
        plan=[{'skill':'turn_by','arguments':{'angle_deg':180.},'start_yaw_rad':0.,'turn_delta_rad':math.pi}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(records[0]['target_reached'])
        self.assertGreater(e.last_turn['displacement_m'],.25)
        self.assertGreaterEqual(len(e.turn_segments),4)
        self.assertTrue(all(row['requested_deg']<=45 for row in e.turn_segments))
    def test_large_position_jump_is_not_hidden_by_new_turn_segment_centres(self):
        e,clock,commands,stops=self.executor(lambda yaw:.2 if yaw>.3 else 0.)
        plan=[{'skill':'turn_by','arguments':{'angle_deg':90.},'start_yaw_rad':0.,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]),self.assertRaisesRegex(RuntimeError,'位置跳变'):
            e.run(plan,'same')
        self.assertTrue(stops)
    def test_shift_across_an_obstacle_stops_below_absolute_drift_limit(self):
        from types import SimpleNamespace
        e,clock,commands,stops=self.executor(lambda yaw:.10*math.sin(yaw))
        e.w.planner=SimpleNamespace(segment_free=lambda a,b:b[0]<.04)
        plan=[{'skill':'turn_by','arguments':{'angle_deg':90.},'start_yaw_rad':0.,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]),self.assertRaisesRegex(RuntimeError,'静态障碍'):
            e.run(plan,'same')
        self.assertLess(e.last_turn['displacement_m'],.15);self.assertTrue(stops)

if __name__=='__main__':unittest.main()
