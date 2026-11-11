import math,unittest
import numpy as np
from geometry import ros_map,scan_ranges,scan_quality,candidate_ready,planar,historical_consistent,seed_ready,particle_mass

class AMCLGeometryChecks(unittest.TestCase):
    def test_ros_grid_centres_match_office_grid_after_row_flip(self):
        T=np.array([[.05,0,-9.425],[0,-.05,3.575],[0,0,1.]])
        occupied=np.zeros((214,407),bool);occupied[100,100]=True
        grid,origin,res=ros_map(occupied,T,bounds=(-20,20,-20,20))
        self.assertEqual(grid[113,100],100)
        np.testing.assert_allclose(np.array(origin)+res*(np.array([100,113])+.5),(T@[100,100,1])[:2])

    def test_unmapped_outside_office_is_unknown_not_global_seed_free_space(self):
        T=np.array([[.1,0,-6],[0,-.1,2],[0,0,1.]])
        grid,_,_=ros_map(np.zeros((80,120),bool),T)
        self.assertEqual(grid[-1,0],-1);self.assertTrue((grid==0).any())

    def test_slice_uses_closest_return_and_leaves_missing_angles_unknown(self):
        points=np.array([[1.,0.,.4],[2.,0.,.5],[0.,1.,0.],[.1,0.,.4],[1.,1.,2.]])
        ranges=scan_ranges(points,np.zeros(2),0.,0.)
        self.assertEqual(np.isfinite(ranges).sum(),1)
        self.assertEqual(ranges[360],1.)

    def test_wrong_location_or_distributed_particles_never_pass(self):
        q={'points':100,'within_0_2m_fraction':.9,'median_distance_m':.05}
        cov=np.diag([.01,.01,0,0,0,math.radians(5)**2]).ravel()
        self.assertTrue(candidate_ready(cov,q,.95))
        self.assertFalse(candidate_ready(cov,q,.89))
        self.assertFalse(candidate_ready(cov,{**q,'within_0_2m_fraction':.64},.99))
        cov[0]=.10;self.assertFalse(candidate_ready(cov,q,.99))

    def test_scan_geometry_rejects_pose_away_from_walls(self):
        occ=np.zeros((81,81),bool);occ[:,60]=True
        T=np.array([[.05,0,-2],[0,-.05,2],[0,0,1.]])
        ranges=np.full(720,np.inf);angles=-np.pi+np.arange(720)*math.radians(.5)
        valid=np.abs(angles)<.5;ranges[valid]=1/np.cos(angles[valid])
        good=scan_quality(ranges,planar(0,0,0),occ,T)
        bad=scan_quality(ranges,planar(-.5,0,0),occ,T)
        self.assertGreater(good['within_0_2m_fraction'],.99)
        self.assertLess(bad['within_0_2m_fraction'],.01)

    def test_3d_rejects_symmetric_wrong_amcl_collapse_even_with_good_median(self):
        wrong={'points':2600,'mean_clipped_m':.24,'median_distance_m':.15,'within_0_2m_fraction':.58}
        self.assertFalse(historical_consistent(wrong))

    def test_seed_requires_distinct_alternative_and_current_independent_geometry(self):
        q={'points':3400,'mean_clipped_m':.16,'median_distance_m':.08,'within_0_2m_fraction':.64}
        q['recent']={**q,'within_0_2m_fraction':.59}
        self.assertTrue(seed_ready(q,.24))
        self.assertFalse(seed_ready(q,.20))
        self.assertFalse(seed_ready(q,None))
        self.assertFalse(seed_ready({**q,'recent':{**q['recent'],'mean_clipped_m':.25}},.24))

    def test_stationary_particle_cloud_remains_valid_and_follows_odometry(self):
        particles=[[0.,0.,0.],[.01,0.,0.]];weights=[.6,.4]
        self.assertEqual(particle_mass(particles,weights,np.eye(4),planar(0,0,0)),1.)
        self.assertEqual(particle_mass(particles,weights,planar(1.,0.,0.),planar(1.,0.,0.)),1.)
        self.assertEqual(particle_mass(particles,weights,np.eye(4),planar(1.,0.,0.)),0.)

    def test_partial_2d_map_requires_stronger_current_3d_corroboration(self):
        cov=np.diag([.01,.01,0,0,0,math.radians(5)**2]).ravel()
        q={'points':600,'within_0_2m_fraction':.60,'median_distance_m':.09}
        h={'within_0_2m_fraction':.71,'median_distance_m':.08,'mean_clipped_m':.14}
        self.assertTrue(candidate_ready(cov,q,.99,h))
        self.assertFalse(candidate_ready(cov,q,.99))
        self.assertFalse(candidate_ready(cov,q,.99,{**h,'mean_clipped_m':.24}))
        self.assertFalse(candidate_ready(cov,{**q,'within_0_2m_fraction':.5},.99,h))

if __name__=='__main__':unittest.main()
