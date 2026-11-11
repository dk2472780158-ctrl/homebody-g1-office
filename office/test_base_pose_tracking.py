"""Check fixed sensor mounting under dynamic acceleration, without network access."""
import copy
import time
import unittest
from pathlib import Path
import numpy as np
from g1_readonly_geometry import BaseModel

ROOT=Path(__file__).resolve().parent
URDF=ROOT.parent/'assets/robot/g1_29dof_with_hand.urdf'

class BasePoseTrackingChecks(unittest.TestCase):
    def setUp(self):
        self.model=BaseModel(URDF)
        self.samples={'/lowstate':(time.time(),{'motor_state':[{'q':0.} for _ in range(35)],
                                              'imu_state':{'accelerometer':[0.,0.,9.81]}}),
                      'virtual_imu':(time.time(),{'linear_acceleration':[0.,0.,9.81]})}

    def moving(self):
        samples=copy.deepcopy(self.samples)
        samples['/lowstate'][1]['imu_state']['accelerometer']=[5.,0.,9.81]
        samples['virtual_imu'][1]['linear_acceleration']=[-5.,0.,9.81]
        return samples

    def test_dynamic_acceleration_keeps_validated_mount_and_live_joints(self):
        initial,_=self.model.tracked(self.samples,'session')
        samples=self.moving()
        with self.assertRaises(ValueError):self.model.candidates(samples)
        same,_=self.model.tracked(samples,'session')
        np.testing.assert_allclose(initial['T_imu_from_pelvis'],same['T_imu_from_pelvis'])
        samples['/lowstate'][1]['motor_state'][12]['q']=.3
        turned,_=self.model.tracked(samples,'session')
        self.assertEqual(initial['name'],turned['name'])
        self.assertFalse(np.allclose(initial['T_imu_from_pelvis'],turned['T_imu_from_pelvis']))

    def test_new_session_requires_gravity_validation_again(self):
        self.model.tracked(self.samples,'first')
        with self.assertRaises(ValueError):self.model.tracked(self.moving(),'second')
        recovered,_=self.model.tracked(self.samples,'second')
        self.assertTrue(np.isfinite(recovered['T_imu_from_pelvis']).all())

    def test_stale_joints_and_imu_still_rejected(self):
        self.model.tracked(self.samples,'session')
        for key in ('/lowstate','virtual_imu'):
            samples=copy.deepcopy(self.samples)
            samples[key]=(time.time()-2,samples[key][1])
            with self.assertRaisesRegex(ValueError,'已过期'):self.model.tracked(samples,'session')

if __name__=='__main__':unittest.main()
