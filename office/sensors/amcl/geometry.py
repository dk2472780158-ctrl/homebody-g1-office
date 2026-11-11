"""Pure AMCL map/scan transformations and independent quality gates."""
import math
import numpy as np
from scipy import ndimage

def planar(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,0],[0,0,0,1]],float)

def ros_map(occupancy,T,bounds=(-5.2,3.5,-3.5,1.8)):
    """NPZ rows point south; ROS rows point north; origins are cell corners."""
    occ=np.asarray(occupancy,bool);res=float(T[0,0]);h,w=occ.shape
    if not np.allclose(T[:2,:2],[[res,0],[0,-res]]) or res<=0:raise ValueError('Unexpected map axes')
    x=T[0,2]+res*np.arange(w);y=T[1,2]-res*np.arange(h)
    known=(x[None,:]>=bounds[0])&(x[None,:]<=bounds[1])&(y[:,None]>=bounds[2])&(y[:,None]<=bounds[3])
    data=np.where(known,np.where(occ,100,0),-1).astype(np.int8)
    return data[::-1].copy(),[float(x[0]-res/2),float(y[-1]-res/2)],res

def scan_ranges(points,base_xy,yaw,floor):
    delta=points[:,:2]-base_xy;r=np.linalg.norm(delta,axis=1)
    valid=(points[:,2]>floor+.15)&(points[:,2]<floor+1.8)&(r>.42)&(r<12.)
    delta=delta[valid];r=r[valid]
    angle=(np.arctan2(delta[:,1],delta[:,0])-yaw+np.pi)%(2*np.pi)-np.pi
    index=np.minimum(719,np.floor((angle+np.pi)/math.radians(.5)).astype(int))
    scan=np.full(720,np.inf);np.minimum.at(scan,index,r)
    return scan

def scan_quality(ranges,map_from_base,occupancy,T):
    finite=np.isfinite(ranges);angles=-np.pi+np.arange(len(ranges))*math.radians(.5)
    points=np.c_[ranges[finite]*np.cos(angles[finite]),ranges[finite]*np.sin(angles[finite])]
    world=points@map_from_base[:2,:2].T+map_from_base[:2,3]
    px=np.c_[world,np.ones(len(world))]@np.linalg.inv(T).T
    distance=ndimage.distance_transform_edt(~occupancy)*float(T[0,0])
    d=ndimage.map_coordinates(distance,[px[:,1],px[:,0]],order=1,mode='constant',cval=2.)
    return dict(points=len(d),median_distance_m=float(np.median(d)) if len(d) else 2.,
        within_0_2m_fraction=float(np.mean(d<.2)) if len(d) else 0.)

def candidate_ready(covariance,quality,dominant_mass,historical=None):
    cov=np.asarray(covariance,float).reshape(6,6)
    corroborated=(historical is not None and quality['within_0_2m_fraction']>=.55
        and historical.get('within_0_2m_fraction',0)>=.65
        and historical.get('median_distance_m',2.)<=.12 and historical.get('mean_clipped_m',2.)<=.18)
    return bool(np.isfinite(cov).all() and (np.diag(cov)>=0).all()
        and max(cov[0,0],cov[1,1])<=.04 and cov[5,5]<=math.radians(12)**2
        and quality['points']>=60 and (quality['within_0_2m_fraction']>=.65 or corroborated)
        and quality['median_distance_m']<=.16 and dominant_mass>=.90)

def historical_consistent(q):
    # Furniture changes can leave unmatched points. Limit the clipped mean as
    # well as the median and overlap: a collapsed, symmetric 2-D hypothesis
    # must not pass merely because its best half fits a wall.
    return (q.get('points',0)>=100 and q.get('mean_clipped_m',2.)<=.20
        and q.get('median_distance_m',2.)<=.16 and q.get('within_0_2m_fraction',0.)>=.55)

def tracking_ready(covariance,quality,dominant_mass,historical,transform,reference):
    """Hysteresis only near a verified transform, never a new global guess."""
    if reference is None or not np.isfinite(transform).all() or not np.isfinite(reference).all():return False
    if (np.linalg.norm(transform[:2,3]-reference[:2,3])>.12 or
        abs(np.angle(np.exp(1j*(math.atan2(transform[1,0],transform[0,0])-
                               math.atan2(reference[1,0],reference[0,0])))))>math.radians(5)):
        return False
    # Keep the original covariance, particle, sample count and median gates.
    # Live 2-D must have >=60% overlap. Keep the original independent 3-D
    # overlap gate, tightening its median/mean and requiring a trusted prior.
    return bool(quality['within_0_2m_fraction']>=.60 and quality['median_distance_m']<=.12
        and historical_consistent(historical)
        and historical['mean_clipped_m']<=.18 and historical['median_distance_m']<=.12
        and candidate_ready(covariance,{**quality,'within_0_2m_fraction':.65},dominant_mass))

def seed_ready(q,alternate):
    return (historical_consistent(q) and q['median_distance_m']<=.12
        and q['within_0_2m_fraction']>=.60 and alternate is not None
        and alternate-q['mean_clipped_m']>=.05 and historical_consistent(q['recent']))

def particle_mass(poses,weights,odom_delta,map_base):
    poses=np.asarray(poses,float).reshape(-1,3);weights=np.asarray(weights,float)
    if not len(poses) or not np.isfinite(poses).all() or not np.isfinite(weights).all() or (weights<0).any() or weights.sum()<=0:return 0.
    c,s=np.cos(poses[:,2]),np.sin(poses[:,2]);dx,dy=odom_delta[:2,3]
    xy=poses[:,:2]+np.c_[c*dx-s*dy,s*dx+c*dy]
    yaw=poses[:,2]+math.atan2(odom_delta[1,0],odom_delta[0,0])
    error=np.angle(np.exp(1j*(yaw-math.atan2(map_base[1,0],map_base[0,0]))))
    near=(np.linalg.norm(xy-map_base[:2,3],axis=1)<.5)&(np.abs(error)<math.radians(20))
    return float(weights[near].sum()/weights.sum())
