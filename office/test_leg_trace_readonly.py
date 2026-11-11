import ast
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


class LegTraceChecks(unittest.TestCase):
    def test_no_robot_command_surface(self):
        tree = ast.parse(Path('g1_leg_trace_readonly.py').read_text())
        attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        self.assertFalse(attrs & {'create_publisher', 'publish', 'create_client', 'call_async'})

    def test_numpy_feedback_and_targets_are_serializable(self):
        tree = ast.parse(Path('g1_leg_trace_readonly.py').read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'leg_values')
        namespace = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]), '<leg_values>', 'exec'), namespace)
        motor = SimpleNamespace(**{k: np.float32(.25) for k in ['q', 'dq', 'tau', 'tau_est', 'kp', 'kd']})
        msg = SimpleNamespace(motor_cmd=[motor] * 35, motor_state=[motor] * 35)
        for command in [True, False]:
            data = json.loads(json.dumps(namespace['leg_values'](msg, command)))
            self.assertEqual(len(data['q']), 12)
            self.assertEqual(data['q'][0], .25)


if __name__ == '__main__':
    unittest.main()
