import unittest
import numpy as np
from test_readonly_windows import load_functions


class OdometryOutputGapChecks(unittest.TestCase):
    def setUp(self):
        self.check=load_functions(
            'sensors/live_ros_observer.py',
            {'odometry_step'},{'np':np})['odometry_step']

    def test_logged_stationary_output_gaps_do_not_restart_the_map(self):
        for gap,distance in ((1.209441,.014611),(1.705183,.005516),(2.410167,.007142)):
            self.assertAlmostEqual(self.check([0,0,0],[distance,0,0],gap),distance)

    def test_timestamp_regression_and_real_position_reset_remain_rejected(self):
        for gap in (0,-.001,float('nan')):
            with self.assertRaisesRegex(ValueError,'timestamp'):
                self.check([0,0,0],[0,0,0],gap)
        for gap,distance in ((.01,.4),(.2,.8),(2.,2.),(100.,2.)):
            with self.assertRaisesRegex(ValueError,'discontinuity'):
                self.check([0,0,0],[distance,0,0],gap)

    def test_normal_forward_motion_is_accepted(self):
        self.assertAlmostEqual(self.check([0,0,0],[.035,0,0],.1),.035)


if __name__=='__main__':unittest.main()
