import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import g1_real_control as control


class PostStopChecks(unittest.TestCase):
    def setup_observer(self,snapshot):
        clock=[0.]
        cancel=SimpleNamespace(is_set=lambda:False,wait=lambda seconds:clock.__setitem__(0,round(clock[0]+seconds,9)))
        e=object.__new__(control.Executor)
        e.w=SimpleNamespace(cancel=cancel,snapshot=lambda:snapshot(clock[0]))
        e.client=SimpleNamespace(velocity=lambda *a,**kw:self.fail('observer sent velocity'),zero=lambda:self.fail('observer sent command'))
        return e,clock

    def state(self,session='same',fresh=True):
        now=control.time.time()
        return {'session_id':session,'odometry_received_unix_s':now if fresh else now-5,'cloud_received_unix_s':now if fresh else now-5}

    def test_observes_delayed_pose_without_extending_motion(self):
        def snapshot(t):
            p=np.eye(4);p[0,3]=.1 if t>=.5 else 0.
            return self.state(),p
        e,clock=self.setup_observer(snapshot)
        with tempfile.TemporaryDirectory() as d,patch.object(control,'STOPFILE',Path(d)/'stop'),patch.object(control.time,'monotonic',lambda:clock[0]):
            pose=e.observe_after_stop('same')
        self.assertAlmostEqual(pose[0,3],.1);self.assertAlmostEqual(clock[0],1.5)

    def test_new_session_cannot_count_as_displacement(self):
        e,clock=self.setup_observer(lambda t:(self.state('changed'),np.eye(4)))
        with tempfile.TemporaryDirectory() as d,patch.object(control,'STOPFILE',Path(d)/'stop'),patch.object(control.time,'monotonic',lambda:clock[0]):
            self.assertIsNone(e.observe_after_stop('same'))

    def test_stale_pose_is_not_motion_evidence(self):
        e,clock=self.setup_observer(lambda t:(self.state(fresh=False),np.eye(4)))
        with tempfile.TemporaryDirectory() as d,patch.object(control,'STOPFILE',Path(d)/'stop'),patch.object(control.time,'monotonic',lambda:clock[0]):
            self.assertIsNone(e.observe_after_stop('same'))

    def test_cancellation_ends_observation(self):
        e,clock=self.setup_observer(lambda t:self.fail('cancelled observer read pose'))
        e.w.cancel.is_set=lambda:True
        with patch.object(control.time,'monotonic',lambda:clock[0]):self.assertIsNone(e.observe_after_stop('same'))
        self.assertEqual(clock[0],0.)


if __name__=='__main__':unittest.main()
