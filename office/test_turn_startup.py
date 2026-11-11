"""A walking turn needs a sustained request before its first actual step."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import g1_real_control as control


class YawPlant:
    def __init__(self,rate=.45,startup=.55,coast=0.,feedback_delay=0.):
        self.now=0.;self.yaw=0.;self.rate=rate;self.startup=startup;self.coast=coast
        self.request=0.;self.actual=0.;self.request_since=None;self.stop_until=0.
        self.pose=np.eye(4);self.commands=[];self.bursts=[];self.zeros=[]
        self.feedback_delay=feedback_delay;self.history=[(0.,0.)]
    def velocity(self,vx,vy,wz,cancel,**kwargs):
        assert vx==vy==0
        if not self.request:self.request_since=self.now
        self.request=wz;self.commands.append((self.now,wz))
    def zero(self):
        if self.request:
            self.bursts.append(self.now-self.request_since)
            self.stop_until=self.now+self.coast
        self.request=0.;self.zeros.append(self.now)
    def wait(self,seconds):
        end=self.now+max(0,seconds)
        while self.now<end-1e-9:
            dt=min(.01,end-self.now)
            if self.request and self.now-self.request_since>=self.startup:
                self.actual=math.copysign(self.rate,self.request)
            elif not self.request and self.now>=self.stop_until:self.actual=0.
            self.yaw+=self.actual*dt;self.now+=dt
            self.history.append((self.now,self.yaw))
        self.now=end
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        self.pose[:2,:2]=[[c,-s],[s,c]]
    def feedback(self):
        yaw=next((yaw for at,yaw in reversed(self.history) if at<=self.now-self.feedback_delay),0.)
        pose=self.pose.copy();c,s=math.cos(yaw),math.sin(yaw)
        pose[:2,:2]=[[c,-s],[s,c]]
        return pose


class TurnStartupChecks(unittest.TestCase):
    def run_turn(self,plant,degrees=90.,sign=1):
        e=object.__new__(control.Executor)
        e.w=SimpleNamespace(cancel=SimpleNamespace(wait=plant.wait),
            snapshot=lambda:({'session_id':'same'},plant.feedback()))
        e.client=SimpleNamespace(health=lambda:dict(armed=True,ros=True,vel_owner={'owner':'teleop'}),
                                velocity=plant.velocity,zero=plant.zero)
        e.guard=lambda *args,**kwargs:plant.feedback()
        e.observe_after_stop=lambda _:plant.pose.copy()
        plan=[dict(skill='turn_by',arguments={'angle_deg':degrees},
                   start_yaw_rad=0.,turn_delta_rad=sign*math.radians(degrees))]
        with patch.object(control.time,'monotonic',lambda:plant.now):records=e.run(plan,'same')
        return e,records
    def test_step_startup_does_not_stall_in_final_degrees(self):
        plant=YawPlant()
        e,records=self.run_turn(plant)
        self.assertTrue(records[0]['target_reached'])
        self.assertLess(abs(math.degrees(plant.yaw)-90),4.)
        self.assertTrue(all(wz==.45 for _,wz in plant.commands))
    def test_small_turn_can_finish_without_short_start_stop_pulses(self):
        plant=YawPlant()
        _,records=self.run_turn(plant,degrees=8.)
        self.assertTrue(records[0]['target_reached'])
        self.assertGreater(plant.bursts[0],plant.startup)
    def test_slow_startup_and_coasting_right_half_turn(self):
        plant=YawPlant(rate=.2,startup=.65,coast=.25)
        _,records=self.run_turn(plant,degrees=180.,sign=-1)
        self.assertTrue(records[0]['target_reached'])
        self.assertTrue(all(wz==-.45 for _,wz in plant.commands))
    def test_completely_unresponsive_turn_still_stops(self):
        plant=YawPlant(rate=0.)
        with self.assertRaisesRegex(RuntimeError,'响应不足'):self.run_turn(plant)
        self.assertEqual(plant.request,0.)
        self.assertLess(plant.now,6.)
    def test_startup_with_delayed_feedback_can_finish_final_correction(self):
        plant=YawPlant(rate=.2,startup=.55,feedback_delay=.35)
        _,records=self.run_turn(plant)
        self.assertTrue(records[0]['target_reached'])
    def test_large_turn_does_not_stop_at_internal_45_degree_checks(self):
        plant=YawPlant()
        e,records=self.run_turn(plant,degrees=175.)
        self.assertTrue(records[0]['target_reached'])
        first_stop=plant.zeros[0]
        stopped_at=next(yaw for at,yaw in reversed(plant.history) if at<=first_stop)
        self.assertGreater(math.degrees(stopped_at),145.)
        self.assertGreaterEqual(len(e.turn_segments),4)
    def test_delayed_scan_batches_do_not_abort_at_internal_angle_boundary(self):
        plant=YawPlant(rate=.45,startup=.55,feedback_delay=.6)
        original=plant.feedback;sample=[None];last=[-1.]
        def batched():
            if plant.now-last[0]>=.3:
                sample[0]=original();last[0]=plant.now
            return sample[0].copy()
        plant.feedback=batched
        _,records=self.run_turn(plant,degrees=175.,sign=-1)
        self.assertTrue(records[0]['target_reached'])
    def test_excessive_final_overrun_still_stops(self):
        plant=YawPlant(rate=.45,startup=.55,coast=1.,feedback_delay=.6)
        with self.assertRaisesRegex(RuntimeError,'超过最终目标角度'):
            self.run_turn(plant,degrees=90.)
        self.assertEqual(plant.request,0.)


if __name__=='__main__':unittest.main()
