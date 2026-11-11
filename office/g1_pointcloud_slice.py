"""Gravity-aligned obstacle height slice and optional LaserScan-style projection.

g1_nav2 reference 99e977a: pointcloud_to_laserscan uses lidar-relative
-1.10..0.30 m, 0.5 degree bins, 0.35..12 m. This installation uses the
registered office floor; preserve the existing 0.15..1.80 m protection band
and 0.42 m self-return mask rather than copying another frame's Z values.
"""
import math
import numpy as np

MIN_HEIGHT=.15
MAX_HEIGHT=1.8
SELF_RADIUS=.42
MAX_RANGE=12.
ANGLE_INCREMENT=math.radians(.5)

def height_slice(xyz,floor,*,min_height=MIN_HEIGHT,max_height=MAX_HEIGHT):
    points=np.asarray(xyz,float)
    if points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all():
        raise ValueError('点云包含无效数值或维度错误')
    if not all(math.isfinite(v) for v in (floor,min_height,max_height)) or min_height>=max_height:
        raise ValueError('点云切割高度无效')
    return points[(points[:,2]>floor+min_height)&(points[:,2]<floor+max_height)]

def obstacle_slice(xyz,floor,origin_xy):
    points=height_slice(xyz,floor)
    distance=np.linalg.norm(points[:,:2]-np.asarray(origin_xy),axis=1)
    return points[(distance>SELF_RADIUS)&(distance<=MAX_RANGE)]

def scan_projection(points,origin_xy,floor,yaw=0.):
    """Nearest return per 0.5-degree bin; infinity means unobserved, not free."""
    points=np.asarray(points,float).reshape(-1,3);origin=np.asarray(origin_xy,float)
    delta=points[:,:2]-origin;ranges=np.linalg.norm(delta,axis=1)
    angle=(np.arctan2(delta[:,1],delta[:,0])-yaw+math.pi)%(2*math.pi)-math.pi
    bins=int(round(2*math.pi/ANGLE_INCREMENT))
    index=np.clip(np.floor((angle+math.pi)/ANGLE_INCREMENT).astype(int),0,bins-1)
    scan=np.full(bins,np.inf);np.minimum.at(scan,index,ranges)
    # Display the actual selected points, avoiding a half-bin position error.
    order=np.argsort(ranges,kind='stable');_,first=np.unique(index[order],return_index=True)
    selected=points[order[first]].copy();selected[:,2]=floor+.08
    return selected,scan
