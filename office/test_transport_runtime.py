import unittest
import numpy as np
from test_readonly_windows import load_functions

class RuntimeTransportChecks(unittest.TestCase):
    def raw(self):
        return dict(running=True,heartbeat_unix_s=100.,cloud_received_unix_s=99.8,
            odometry_received_unix_s=99.9,cloud_stamps_nondecreasing=True,
            odom_stamps_nondecreasing=True,cloud_stamp_s=99.4,cloud_frame='camera_init',
            odometry=dict(frame_id='camera_init',child_frame_id='body',xyz=[0,0,0],xyzw=[0,0,0,1],stamp_s=99.5),
            sensor_transport=dict(running=True,ready=False,heartbeat_unix_s=99.8,
                source_stamp=dict(cloud=99.4,imu=99.99),backlog_growth_s=dict(cloud=.1,imu=.01)))
    def validate(self,raw):
        return load_functions('office-live-map-worker.py',{'validate_raw'},{'np':np})['validate_raw'](raw,100.)
    def test_sparse_cloud_startup_flag_does_not_invalidate_fresh_initialized_output(self):
        self.assertEqual(self.validate(self.raw())['stamp_s'],99.5)
    def test_false_ready_with_stale_output_is_rejected(self):
        raw=self.raw();raw['odometry_received_unix_s']=99.1
        with self.assertRaisesRegex(ValueError,'不够新鲜'):self.validate(raw)
    def test_real_transport_failure_and_excessive_skew_remain_blocked(self):
        for field,value in [('running',False),('heartbeat_unix_s',99.1)]:
            raw=self.raw();raw['sensor_transport'][field]=value
            with self.assertRaises(ValueError):self.validate(raw)
        raw=self.raw();raw['sensor_transport']['source_stamp']['imu']=100.4
        with self.assertRaises(ValueError):self.validate(raw)
    def test_backlog_and_processing_lag_remain_blocked(self):
        raw=self.raw();raw['sensor_transport']['backlog_growth_s']['cloud']=.76
        with self.assertRaisesRegex(ValueError,'积压'):self.validate(raw)
        raw=self.raw();raw['odometry']['stamp_s']=98.
        with self.assertRaisesRegex(ValueError,'落后'):self.validate(raw)

if __name__=='__main__':unittest.main()
