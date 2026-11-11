"""Regression maps for narrow passages and a heading-constrained departure."""
import tempfile
import unittest
from pathlib import Path
import numpy as np
from g1_readonly_geometry import StaticRoute
from g1_dynamic_navigation import validate_route_turns


class ClearanceRecoveryChecks(unittest.TestCase):
    def planner(self,occupied):
        folder=tempfile.TemporaryDirectory();self.addCleanup(folder.cleanup)
        path=Path(folder.name)/'map.npz'
        np.savez(path,occupancy=occupied,T_map_px=np.array([[.05,0,-2.],[0,.05,-2.],[0,0,1.]]))
        return StaticRoute(path)

    def bay(self):
        occupied=np.zeros((101,121),bool)
        occupied[[0,-1],:]=True;occupied[:,[0,-1]]=True
        # Walls end at x=.6. East is the only forward exit before turning north.
        occupied[31,:53]=True;occupied[49,:53]=True
        return self.planner(occupied)

    def test_narrow_throat_connects_open_rooms_without_wide_hard_limit(self):
        occupied=np.zeros((101,121),bool)
        occupied[[0,-1],:]=True;occupied[:,[0,-1]]=True
        occupied[:,59:62]=True;occupied[31:50,59:62]=False
        planner=self.planner(occupied)
        with self.assertRaises(ValueError):planner.route([0.,0.],[2.5,0.],interior_clearance=.53)
        path,_=planner.navigation_route([0.,0.],[2.5,0.],yaw=0.)
        self.assertTrue(all(planner.segment_free(a,b) for a,b in zip(path[:-1],path[1:])))
        np.testing.assert_allclose(path[-1],[2.5,0.])

    def test_tight_start_exits_forward_before_turning(self):
        planner=self.bay()
        self.assertLess(planner.clearance_at([0.,0.]),.48)
        path,detail=planner.navigation_route([0.,0.],[2.,1.2],yaw=0.)
        self.assertGreater(detail['forward_exit_m'],.15)
        self.assertAlmostEqual(path[1,1],0.)
        self.assertGreater(path[1,0],0.)
        self.assertGreater(planner.clearance_at(path[1]),.48)
        validate_route_turns(planner,path,0.,.48,.38,np.deg2rad(5))

    def test_blocked_forward_exit_cannot_sidestep_reverse_or_turn_in_place(self):
        planner=self.bay()
        # Facing west, forward goes farther into the bay. No approved escape.
        with self.assertRaises(ValueError):
            planner.navigation_route([0.,0.],[2.,1.2],yaw=np.pi)

    def test_passage_smaller_than_footprint_still_blocks(self):
        occupied=np.zeros((101,121),bool)
        occupied[[0,-1],:]=True;occupied[:,[0,-1]]=True
        occupied[:,59:62]=True;occupied[36:45,59:62]=False
        planner=self.planner(occupied)
        with self.assertRaises(ValueError):planner.navigation_route([0.,0.],[2.5,0.],yaw=0.)


if __name__=='__main__':unittest.main()
