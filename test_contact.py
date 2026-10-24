import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from contact import plants,solve_leg,correct_feet


def fixture():
    n=120;t=np.arange(n)/30
    drift=np.c_[.05*np.sin(t),.025*np.sin(t*.7),np.zeros(n)]
    legs={}
    for side,x in [('Left',.15),('Right',-.15)]:
        hip=np.tile([x,1.7,0.],(n,1))+drift
        knee=np.tile([x,.9,.20],(n,1))+drift
        ankle=np.tile([x,.15,0],(n,1))+drift
        sa=np.tile([x,.15,0],(n,1));st=np.tile([x,0,-.2],(n,1))
        sa[40:80,1]+=1;st[40:80,1]+=1
        legs[side]=dict(source_ankle=sa,source_toe=st,source_length=1.6,hip=hip,knee=knee,ankle=ankle,upper_q=R.identity(n),lower_q=R.identity(n),foot_q=R.identity(n),rest_ankle_height=.15)
    return legs,np.zeros((n,3))


class ContactTests(unittest.TestCase):
    def test_near_straight_knee_does_not_flip(self):
        h=np.array([0.,2.,0.]);a=np.array([0.,0.,0.]);goal=np.array([.15,.05,0.])
        knees=[]
        for z in np.linspace(-.015,.015,41):
            k=np.array([0.,1.,z])
            _,_,nk,_,_=solve_leg(h,k,a,goal,R.identity(),R.identity())
            knees.append(nk)
        self.assertLess(np.linalg.norm(np.diff(knees,axis=0),axis=1).max(),.01)
    def test_folded_leg_has_finite_solution(self):
        h=np.array([0.,1.,0.]);k=np.array([0.,0.,.2]);a=h.copy()
        u,l,nk,na,e=solve_leg(h,k,a,np.array([.2,1.,0.]),R.identity(),R.identity())
        self.assertTrue(np.isfinite(np.r_[u.as_quat(),l.as_quat(),nk,na,e]).all())
    def test_tilted_shoe_heel_rests_on_floor(self):
        legs,shift=fixture()
        samples=np.array([[0,-.15,-.2],[0,-.15,.1],[.03,-.14,.1]])
        for v in legs.values():
            v['sole_points']=samples
            v['foot_q']=R.from_euler('x',np.full((len(shift),1),15),degrees=True)
        result,_,_=correct_feet(legs,shift,floor=.07)
        for v in result.values():
            for frame in np.flatnonzero(v['weight']==1):
                bottom=np.min(v['foot_q'][frame].apply(samples)[:,1])+v['solved_ankle'][frame,1]
                self.assertAlmostEqual(bottom,.07,places=6)

    def test_jump_and_transition_scope(self):
        legs,shift=fixture();result,new,report=correct_feet(legs,shift)
        for side,v in result.items():
            self.assertTrue(np.all(v['weight'][40:80]==0))
            np.testing.assert_array_equal(v['upper_q'].as_quat()[40:80],legs[side]['upper_q'].as_quat()[40:80])
            np.testing.assert_array_equal(v['solved_ankle'][40:80],legs[side]['ankle'][40:80])
            self.assertGreater(np.count_nonzero(v['weight']==1),20)
            lock=v['weight']==1
            np.testing.assert_allclose(v['solved_ankle'][lock],v['goal'][lock],atol=1e-6)
        np.testing.assert_array_equal(new[40:80],shift[40:80])
        self.assertLess(report['sides']['Left']['mean_full_lock_error_after'],1e-6)
        self.assertGreater(report['sides']['Left']['mean_full_lock_error_before'],.005)
    def test_zero_strength_preserves_every_frame(self):
        legs,shift=fixture();result,new,_=correct_feet(legs,shift,strength=0)
        np.testing.assert_array_equal(new,shift)
        for side,v in result.items():
            np.testing.assert_array_equal(v['upper_q'].as_quat(),legs[side]['upper_q'].as_quat())
            np.testing.assert_array_equal(v['foot_q'].as_quat(),legs[side]['foot_q'].as_quat())
    def test_elevated_foot_not_locked(self):
        legs,shift=fixture()
        for k in ('source_ankle','source_toe'):legs['Left'][k][:,1]+=2
        result,_,_=correct_feet(legs,shift)
        self.assertEqual(np.count_nonzero(result['Left']['weight']),0)
    def test_fast_floor_motion_is_not_planted(self):
        t=np.arange(100)/30;a=np.c_[t,np.zeros(100),np.zeros(100)]
        w,segments,_=plants(a,a,1)
        self.assertFalse(w.any());self.assertEqual(segments,[])
    def test_bone_lengths_and_unreachable_goal(self):
        h=np.array([0.,2,0]);k=np.array([0.,1,.2]);a=np.array([0.,0,0])
        u,l,nk,na,error=solve_leg(h,k,a,np.array([0.,-10,0]),R.identity(),R.identity())
        self.assertGreater(error,5)
        self.assertAlmostEqual(np.linalg.norm(nk-h),np.linalg.norm(k-h),places=6)
        self.assertAlmostEqual(np.linalg.norm(na-nk),np.linalg.norm(a-k),places=6)
        np.testing.assert_allclose(h+u.apply(k-h),nk,atol=1e-6)
        np.testing.assert_allclose(nk+l.apply(a-k),na,atol=1e-6)
    def test_invalid_settings(self):
        legs,shift=fixture()
        for strength in (-1,2,float('nan')):
            with self.assertRaises(ValueError):correct_feet(legs,shift,strength=strength)
    def test_foot_spin_and_short_contact_are_not_locked(self):
        ankle=np.tile([0.,.15,0.],(50,1));toe=np.tile([0.,0.,-.2],(50,1))
        rotation=R.from_euler('y',(np.arange(50)*5)[:,None],degrees=True)
        w,_,_=plants(ankle,toe,1.6,rotation=rotation)
        self.assertFalse(w.any())
        w,_,_=plants(ankle[:4],toe[:4],1.6)
        self.assertFalse(w.any())


if __name__=='__main__':unittest.main()
