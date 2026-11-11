"""Recovery must preserve geometric quality and multi-scan validation."""
import unittest
import numpy as np
from test_readonly_windows import load_functions

class ScanRecoveryChecks(unittest.TestCase):
    def matcher(self):
        class Matcher:
            def __init__(self):self.calls=[];self.recent_good=True
            def global_match(self,cloud,up=None):
                self.calls.append(cloud)
                if cloud!='latest':raise ValueError('Global match quality too low')
                return np.eye(4),{'within_0_2m_fraction':.74,'median_distance_m':.053}
            def current_structure(self,cloud,transform):return np.zeros((120,3))
            def metrics(self,cloud,transform):return {'within_0_2m_fraction':.72 if self.recent_good else .64,'median_distance_m':.08}
        return Matcher()
    def match(self,m):
        fn=load_functions('office-live-map-worker.py',{'match_scan_windows'}, {})['match_scan_windows']
        return fn(m,'accumulated','recent','latest',[0,0,1])
    def test_latest_unambiguous_fit_can_recover_after_accumulated_failure(self):
        m=self.matcher();t,q=self.match(m)
        self.assertEqual(m.calls,['accumulated','recent','latest'])
        self.assertEqual(q['initial_cloud_source'],'latest')
        self.assertEqual(q['within_0_2m_fraction'],.74)
    def test_latest_fit_alone_cannot_override_failed_recent_geometry(self):
        m=self.matcher();m.recent_good=False
        with self.assertRaisesRegex(ValueError,'当前点云与历史地图不一致'):self.match(m)
    def test_ambiguous_candidate_is_never_accepted(self):
        m=self.matcher()
        def ambiguous(*args,**kwargs):raise ValueError('Global match ambiguous in searched candidates')
        m.global_match=ambiguous
        with self.assertRaisesRegex(ValueError,'ambiguous'):self.match(m)

if __name__=='__main__':unittest.main()
