import ast
import html
import math
import re
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import g1_real_control as control
from office_landmarks import LANDMARK_PLACES


def workflow_parts():
    tree=ast.parse(Path(__file__).with_name('office_real_workflow.py').read_text(encoding='utf-8'))
    names={'PLACES','COMPASS','ALIASES','SKILLS'}
    body=[]
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id in names for t in node.targets):
            body.append(node)
        if isinstance(node,ast.FunctionDef) and node.name in ('clean_task','validate_steps','direct_steps'):
            body.append(node)
        if isinstance(node,ast.ClassDef) and node.name=='RealWorkflow':
            body.extend(m for m in node.body if isinstance(m,ast.FunctionDef) and m.name=='prepare')
    ns={'html':html,'math':math,'re':re,'np':np,'LANDMARK_PLACES':LANDMARK_PLACES,'task_speed':control.task_speed,
        'pose_yaw':control.pose_yaw,'angle_diff':control.angle_diff,'TURN_RATE':control.TURN_RATE,
        'TURN_CLEARANCE':control.TURN_CLEARANCE,'ROUTE_TURN_CLEARANCE':control.ROUTE_TURN_CLEARANCE,
        'TRAVEL_ALIGN':control.TRAVEL_ALIGN}
    exec(compile(ast.Module(body=body,type_ignores=[]),'<workflow>','exec'),ns)
    return ns


def preview(yaw=0.,clearance=2.):
    pose=np.eye(4);pose[:2,:2]=[[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]]
    node=lambda:SimpleNamespace(visible=False,points=None,position=None)
    w=SimpleNamespace(snapshot=lambda:({'session_id':'same'},pose),cancel=threading.Event(),
        planner=SimpleNamespace(radius=.35,clearance_at=lambda _:clearance),floor=0.,
        route=node(),goal=node(),turn_preview=node(),turn_goal=node(),real_mode=True,points={},path=None)
    return w


class TurningChecks(unittest.TestCase):
    def test_natural_language_turn_and_absolute_heading(self):
        ns=workflow_parts();parse=ns['direct_steps']
        self.assertEqual(parse('原地左转90度'),[{'skill':'turn_by','arguments':{'direction':'left','angle_deg':90.}}])
        self.assertEqual(parse('向右转身'),[{'skill':'turn_by','arguments':{'direction':'right','angle_deg':180}}])
        self.assertEqual(parse('转向东南'),[{'skill':'face','arguments':{'direction':'southeast'}}])
        self.assertEqual(parse('向右走两秒')[0]['skill'],'move_for')
        with self.assertRaisesRegex(ValueError,'左转或右转'):parse('转身')
        with self.assertRaises(ValueError):ns['validate_steps']([{'skill':'turn_by','arguments':{'direction':'left','angle_deg':181}}])

    def test_preview_turns_then_moves_in_new_heading(self):
        ns=workflow_parts();w=preview()
        steps=[{'skill':'turn_by','arguments':{'direction':'left','angle_deg':90}},
               {'skill':'move_for','arguments':{'direction':'forward','duration_s':1}}]
        result=ns['prepare'](w,steps)
        self.assertAlmostEqual(result[0]['target_yaw_rad'],math.pi/2)
        self.assertAlmostEqual(result[1]['route_xy_m'][-1][0],0.,places=7)
        self.assertAlmostEqual(result[1]['route_xy_m'][-1][1],.35)
        self.assertTrue(w.turn_preview.visible)
        self.assertAlmostEqual(w.turn_goal.position[1],.65)
        self.assertAlmostEqual(w.path['yaw'],0.)

    def test_preview_blocks_tight_rotation_clearance(self):
        ns=workflow_parts();w=preview(clearance=.47)
        result=ns['prepare'](w,[{'skill':'face','arguments':{'direction':'west'}}])
        self.assertEqual(result[0]['status'],'blocked')
        self.assertFalse(w.turn_preview.visible)

    def test_absolute_heading_chooses_shortest_turn(self):
        ns=workflow_parts();w=preview(yaw=math.pi/2)
        result=ns['prepare'](w,[{'skill':'face','arguments':{'direction':'east'}}])
        self.assertAlmostEqual(result[0]['target_yaw_rad'],0.)
        self.assertAlmostEqual(result[0]['turn_delta_rad'],-math.pi/2)

    def test_compass_move_previews_turn_then_forward_travel(self):
        ns=workflow_parts();w=preview(yaw=math.pi)
        result=ns['prepare'](w,[{'skill':'move_for','arguments':{'direction':'east','duration_s':2}}])
        self.assertAlmostEqual(result[0]['face_yaw_rad'],0.)
        self.assertAlmostEqual(result[0]['route_xy_m'][-1][0],.7)
        self.assertTrue(w.turn_preview.visible)
        self.assertIn('正面',result[0]['description'])

    def test_backward_previews_half_turn_not_reverse_walk(self):
        ns=workflow_parts();w=preview()
        result=ns['prepare'](w,[{'skill':'move_for','arguments':{'direction':'backward','duration_s':1}}])
        self.assertAlmostEqual(abs(result[0]['face_yaw_rad']),math.pi)
        self.assertAlmostEqual(result[0]['route_xy_m'][-1][0],-.35)
        self.assertTrue(w.turn_preview.visible)

    def test_named_route_previews_turn_at_start(self):
        ns=workflow_parts();w=preview()
        w.planner.route=lambda start,goal,**kwargs:(np.array([[0.,0.],[-.35,0.]]),{'length_m':.35})
        w.planner.navigation_route=w.planner.route
        result=ns['prepare'](w,[{'skill':'navigate','arguments':{'goal_xy_m':[-.35,0.]}}])
        self.assertEqual(result[0]['status'],'planned')
        self.assertTrue(w.turn_preview.visible)
        self.assertIn('正面',result[0]['description'])

    def test_chained_navigation_uses_and_discloses_adjusted_stop(self):
        from g1_readonly_geometry import StaticRoute
        ns=workflow_parts();w=preview();pose=np.eye(4);pose[:2,3]=[.85,-.9]
        w.snapshot=lambda:({'session_id':'same'},pose.copy())
        w.planner=StaticRoute('real-office-navigation.npz')
        w.points={'架子':[2.125,.575],'门口':[2.75,-2.575]}
        result=ns['prepare'](w,[{'skill':'go_shelf','arguments':{}},{'skill':'go_door','arguments':{}}])
        self.assertEqual([r['status'] for r in result],['planned','planned'])
        self.assertIn('停车点调整',result[0]['description'])
        np.testing.assert_allclose(result[0]['requested_goal_xy_m'],[2.125,.575])
        np.testing.assert_allclose(result[1]['route_xy_m'][0],result[0]['stop_goal_xy_m'])
        for item in result:self.assertGreater(w.planner.clearance_at(item['stop_goal_xy_m']),.68)

    def executor(self,opposite=False,yaw=0.):
        clock=[0.];pose=np.eye(4);angular=[0.];linear=[0.];commands=[];stops=[]
        pose[:2,:2]=[[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]]
        def wait(seconds):
            clock[0]+=seconds
            yaw=control.pose_yaw(pose)
            pose[0,3]+=linear[0]*math.cos(yaw)*seconds
            pose[1,3]+=linear[0]*math.sin(yaw)*seconds
            yaw+=angular[0]*seconds
            pose[:2,:2]=[[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]]
        w=SimpleNamespace(cancel=SimpleNamespace(wait=wait),snapshot=lambda:({'session_id':'same'},pose.copy()))
        e=object.__new__(control.Executor);e.w=w
        def velocity(vx,vy,wz,cancel,**kwargs):
            commands.append((vx,vy,wz));linear[0]=vx;angular[0]=-wz if opposite else wz
        def zero():stops.append(True);linear[0]=angular[0]=0.
        e.client=SimpleNamespace(health=lambda:{'armed':True,'ros':True,'limits':{'vx':.35,'vy':.35,'wz':.45},'vel_owner':{'owner':'teleop'}},velocity=velocity,zero=zero)
        e.guard=lambda session,v,turning=False,**kwargs:pose.copy()
        e.observe_after_stop=lambda session:pose.copy()
        return e,clock,commands,stops

    def test_turn_commands_only_yaw_and_stops_at_target(self):
        e,clock,commands,stops=self.executor()
        plan=[{'skill':'turn_by','arguments':{'direction':'left','angle_deg':90},
               'start_yaw_rad':0.,'target_yaw_rad':math.pi/2,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(commands)
        self.assertTrue(all(vx==0 and vy==0 and wz==control.TURN_RATE for vx,vy,wz in commands))
        self.assertTrue(records[0]['target_reached'])
        self.assertTrue(records[0]['rotation_observed'])
        self.assertGreaterEqual(len(stops),2)

    def test_wrong_turn_direction_aborts_and_zeroes(self):
        e,clock,commands,stops=self.executor(opposite=True)
        plan=[{'skill':'turn_by','arguments':{'direction':'left','angle_deg':90},
               'start_yaw_rad':0.,'target_yaw_rad':math.pi/2,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]),self.assertRaisesRegex(RuntimeError,'实际转向'):
            e.run(plan,'same')
        self.assertTrue(commands)
        self.assertEqual(stops,[True])

    def test_slower_feedback_still_reaches_requested_ninety_degrees(self):
        e,clock,commands,stops=self.executor()
        velocity=e.client.velocity
        def slower(vx,vy,wz,cancel,**kwargs):
            return velocity(vx,vy,wz*(.1/control.TURN_RATE),cancel,**kwargs)
        e.client.velocity=slower
        plan=[{'skill':'turn_by','arguments':{'direction':'left','angle_deg':90},
               'start_yaw_rad':0.,'target_yaw_rad':math.pi/2,'turn_delta_rad':math.pi/2}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertGreater(clock[0],12.)
        self.assertTrue(records[0]['target_reached'])
        self.assertGreater(records[0]['observed_rotation_deg'],85.)
        self.assertGreaterEqual(len(stops),2)

    def test_right_half_turn_keeps_requested_direction(self):
        e,clock,commands,stops=self.executor()
        plan=[{'skill':'turn_by','arguments':{'direction':'right','angle_deg':180},
               'start_yaw_rad':0.,'target_yaw_rad':-math.pi,'turn_delta_rad':-math.pi}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(commands)
        self.assertTrue(all(wz==-control.TURN_RATE for _,_,wz in commands))
        self.assertTrue(records[0]['target_reached'])
        self.assertGreaterEqual(len(stops),2)

    def test_opposite_navigation_turns_then_walks_forward(self):
        e,clock,commands,stops=self.executor()
        plan=[{'skill':'navigate','arguments':{'goal_xy_m':[-.35,0.]},
               'route_xy_m':[[0.,0.],[-.35,0.]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(any(wz!=0 and vx==0 and vy==0 for vx,vy,wz in commands))
        self.assertTrue(any(vx==control.SPEED and vy==0 and abs(wz)<=control.TRAVEL_YAW_RATE for vx,vy,wz in commands))
        self.assertTrue(all(vx>=0 and vy==0 for vx,vy,wz in commands))
        self.assertTrue(records[0]['motion_observed'])
        self.assertEqual(stops[-1],True)

    def test_compass_move_turns_then_uses_forward_only(self):
        e,clock,commands,stops=self.executor(yaw=math.pi)
        plan=[{'skill':'move_for','arguments':{'direction':'east','duration_s':1},
               'route_xy_m':[[0.,0.],[.35,0.]],'face_yaw_rad':0.}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):records=e.run(plan,'same')
        self.assertTrue(any(wz!=0 for _,_,wz in commands))
        self.assertTrue(any(vx==control.SPEED and vy==0 and wz==0 for vx,vy,wz in commands))
        self.assertTrue(all(vx>=0 and vy==0 for vx,vy,wz in commands))
        self.assertTrue(records[0]['motion_observed'])

    def test_relative_left_stays_a_side_step(self):
        e,clock,commands,stops=self.executor()
        plan=[{'skill':'move_for','arguments':{'direction':'left','duration_s':1},
               'route_xy_m':[[0.,0.],[0.,.35]]}]
        with patch.object(control.time,'monotonic',lambda:clock[0]):e.run(plan,'same')
        self.assertTrue(commands)
        self.assertTrue(all(vx==0 and vy==control.SPEED and wz==0 for vx,vy,wz in commands))

    def test_live_obstacle_blocks_rotation(self):
        with tempfile.TemporaryDirectory() as folder:
            now=time.time();state={'session_id':'same','odometry_received_unix_s':now,
                'cloud_received_unix_s':now,'T_office_from_current':np.eye(4).tolist()}
            np.savez(Path(folder)/'clouds.npz',session_id='same',received_unix_s=now,
                latest_xyz=np.array([[.5,-.05,.8],[.5,0.,.8],[.5,.05,.8]]))
            w=SimpleNamespace(cancel=threading.Event(),snapshot=lambda:(state,np.eye(4)),
                planner=SimpleNamespace(radius=.35,clearance_at=lambda _:2.),live=Path(folder),floor=0.)
            e=object.__new__(control.Executor);e.w=w;e.last_health=0.
            e.client=SimpleNamespace(health=lambda:{'armed':True,'ros':True,'vel_owner':{'owner':'teleop'}})
            with self.assertRaisesRegex(RuntimeError,'转身范围内有实时障碍'):
                e.guard('same',np.zeros(2),turning=True)


if __name__=='__main__':unittest.main()
