"""Check semantic goals and prevent failed previews from retaining an old task."""
import ast
import html
import json
import tempfile
import threading
import time
import unittest
import uuid
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from g1_readonly_geometry import StaticRoute
from office_landmarks import approach_points
from test_turning import workflow_parts,preview

ROOT=Path(__file__).resolve().parent


class RoomCenterChecks(unittest.TestCase):
    def test_center_aliases_have_one_goal(self):
        ns=workflow_parts()
        for text in ('去房间的正中央','去房间中央','请走到办公室的中心','前往正中间',
                     '到中间','移动到屋子的正中位置','<div>去房间的正中央<br></div>'):
            with self.subTest(text=text):
                self.assertEqual(ns['direct_steps'](text),[{'skill':'go_center','arguments':{}}])
        self.assertIsNone(ns['direct_steps']('去桌子1和桌子2的中间'))
        self.assertIsNone(ns['direct_steps']('去房间中央，然后握手'))
        self.assertEqual(ns['direct_steps']('去门口')[0]['skill'],'go_door')

    def test_center_preview_uses_room_center_not_previous_door(self):
        ns=workflow_parts();w=preview()
        w.points=approach_points(ROOT,json.loads((ROOT/'office-waypoints.json').read_text(encoding='utf-8'))['points'])
        w.planner=StaticRoute(ROOT/'real-office-navigation.npz')
        pose=np.eye(4);pose[:2,3]=[2.62,-2.60]
        w.snapshot=lambda:({'session_id':'same'},pose.copy())
        result=ns['prepare'](w,ns['direct_steps']('去房间的正中央'))[0]
        self.assertEqual(result['status'],'planned')
        self.assertIn('前往房间中央',result['description'])
        np.testing.assert_allclose(result['requested_goal_xy_m'],[-.65,-.85])
        np.testing.assert_allclose(result['stop_goal_xy_m'],[-.65,-.85])
        self.assertGreater(w.planner.clearance_at(result['stop_goal_xy_m']),1.4)

    def test_failed_preview_clears_old_and_partially_built_plan(self):
        ns=workflow_parts()
        tree=ast.parse((ROOT/'office_real_workflow.py').read_text(encoding='utf-8'))
        submit=next(m for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='RealWorkflow'
                    for m in c.body if isinstance(m,ast.FunctionDef) and m.name=='submit')
        handle=lambda **kw:SimpleNamespace(**kw)
        for failure in ('parse','prepare'):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as directory:
                w=handle(root=Path(directory),lock=threading.Lock(),cancel=threading.Event(),
                    pending={'steps':[{'skill':'go_door'}]},path={'old':'door'},
                    execute_button=handle(disabled=False),execute_reason=handle(content='旧预览有效'),
                    ui=handle(thought=handle(content='前往门口'),status=handle(content='旧任务'),
                              submit_button=handle(disabled=False)),
                    mode=handle(disabled=False),probe_button=handle(disabled=False),result=handle(content='前往门口'))
                for name in ('route','goal','turn_preview','turn_goal'):setattr(w,name,handle(visible=True))
                w.provider=handle(generate=lambda *a,**kw:handle(arguments={'steps':[]}))
                def fail_prepare(*a,**kw):
                    w.path={'partial':True};w.route.visible=True;w.goal.visible=True
                    raise ValueError('测试规划失败')
                w.prepare=fail_prepare
                check=self
                class Thread:
                    def __init__(self,target,**kw):self.target=target
                    def start(self):
                        check.assertEqual(w.ui.thought.content,'')
                        check.assertIsNone(w.pending)
                        check.assertTrue(w.execute_button.disabled)
                        self.target()
                ns.update(threading=handle(Thread=Thread),uuid=uuid,time=time,json=json,html=html,PROMPT='')
                exec(compile(ast.Module(body=[submit],type_ignores=[]),'<submit>','exec'),ns)
                ns['submit'](w,'不支持的指令' if failure=='parse' else '去房间的正中央')
                self.assertIsNone(w.pending);self.assertIsNone(w.path)
                self.assertEqual(w.ui.thought.content,'')
                self.assertIn('任务预览未完成',w.result.content)
                self.assertIn('本次预览未完成',w.execute_reason.content)
                self.assertTrue(w.execute_button.disabled)
                self.assertTrue(all(not getattr(w,n).visible for n in ('route','goal','turn_preview','turn_goal')))
                self.assertFalse(w.ui.submit_button.disabled)
                self.assertFalse(w.lock.locked())


if __name__=='__main__':unittest.main()
