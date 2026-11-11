"""Exercise the real button recovery without starting a viewer or robot client."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time,unittest
import numpy as np

tree=ast.parse(Path(__file__).with_name('office_real_workflow.py').read_text(encoding='utf-8'))
method=next(n for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='RealWorkflow'
            for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='refresh_execute_state')
namespace={'time':time,'np':np}
exec(compile(ast.Module(body=[method],type_ignores=[]),'<refresh_execute_state>','exec'),namespace)
refresh=namespace['refresh_execute_state']

class ExecuteButtonChecks(unittest.TestCase):
    def workflow(self):
        return SimpleNamespace(executing=False,real_mode=True,control_reason='',
            pending={'at':time.monotonic(),'session':'current','pose':np.zeros(2)},
            execute_button=SimpleNamespace(disabled=True),execute_reason=SimpleNamespace(content=''))
    def test_fresh_pose_restores_disabled_button(self):
        w=self.workflow();refresh(w,{'session_id':'current'},np.eye(4))
        self.assertFalse(w.execute_button.disabled)
    def test_control_block_disables_valid_preview_and_recovers(self):
        w=self.workflow();w.control_reason='机器人电量 9%，请先充电'
        refresh(w,{'session_id':'current'},np.eye(4))
        self.assertTrue(w.execute_button.disabled)
        self.assertIn('9%',w.execute_reason.content)
        w.control_reason='';refresh(w,{'session_id':'current'},np.eye(4))
        self.assertFalse(w.execute_button.disabled)
    def test_session_change_stays_disabled(self):
        w=self.workflow();refresh(w,{'session_id':'new'},np.eye(4))
        self.assertTrue(w.execute_button.disabled)
        self.assertIn('会话',w.execute_reason.content)
    def test_expired_plan_stays_disabled(self):
        w=self.workflow();w.pending['at']-=31;refresh(w,{'session_id':'current'},np.eye(4))
        self.assertTrue(w.execute_button.disabled)
    def test_consumed_plan_explains_repreview(self):
        w=self.workflow();w.pending=None;refresh(w,{'session_id':'current'},np.eye(4))
        self.assertTrue(w.execute_button.disabled)
        self.assertIn('只执行一次',w.execute_reason.content)

if __name__=='__main__':unittest.main()
