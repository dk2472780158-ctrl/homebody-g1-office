"""Bounded live obstacle costmap; no robot interfaces.

Adapted to the current executor from g1_nav2's static + obstacle + inflation
layers. Old static cells are never cleared; live cells expire after 3 seconds
and every resumed command still checks the latest independent cloud.
"""
import copy
import math
import numpy as np
from scipy import ndimage

LIVE_TRAVEL_RADIUS=.4

class ObstacleBlocked(RuntimeError):
    """Only collision guards may request route recovery."""

class DynamicObstacleLayer:
    def __init__(self,planner,session,retention_s=3.):
        self.static=planner;self.session=session;self.retention_s=retention_s
        self.cells={};self.last_received=None

    def observe(self,xy,received,now,session):
        if session!=self.session:raise RuntimeError('定位会话已改变，禁止重规划')
        if not math.isfinite(received) or not 0<=now-received<.8:
            raise RuntimeError('实时障碍点云已过期')
        if self.last_received is not None and received<self.last_received:
            raise RuntimeError('实时障碍点云时间回退')
        points=np.asarray(xy,float).reshape(-1,2)
        if not np.isfinite(points).all():raise RuntimeError('实时障碍点云包含无效数值')
        if len(points):
            px=np.c_[points,np.ones(len(points))]@self.static.inverse.T
            cells=np.unique(np.rint(px[:,:2]).astype(int),axis=0)
            height,width=self.static.occupied.shape
            for col,row in cells:
                if 0<=row<height and 0<=col<width:self.cells[(int(row),int(col))]=received
        self.last_received=received
        self.cells={cell:at for cell,at in self.cells.items() if now-at<=self.retention_s}

    def clear_observed_rays(self,xy,origin_xy,received):
        """Clear only positively observed free ray prefixes, like Nav2 clearing.

        A missing return never clears a cell. Nearest height-slice return per
        half-degree bin prevents a distant surface clearing a nearer obstacle.
        """
        if received==getattr(self,'last_cleared',None):return
        self.last_cleared=received
        points=np.asarray(xy,float).reshape(-1,2);origin=np.asarray(origin_xy,float)
        if not len(points) or not self.cells:return
        delta=points-origin;ranges=np.linalg.norm(delta,axis=1)
        angles=np.arctan2(delta[:,1],delta[:,0]);bins=np.floor((angles+math.pi)/math.radians(.5)).astype(int)
        order=np.argsort(ranges,kind='stable');_,first=np.unique(bins[order],return_index=True)
        selected=order[first];samples=[]
        for index in selected:
            # Keep endpoint quantization and obstacle surface thickness.
            end=ranges[index]-.12
            if end<=.42:continue
            lengths=np.arange(.42,end,self.static.resolution/2)
            samples.append(origin+lengths[:,None]*delta[index]/ranges[index])
        if not samples:return
        points=np.vstack(samples);px=np.c_[points,np.ones(len(points))]@self.static.inverse.T
        cells=np.unique(np.rint(px[:,:2]).astype(int),axis=0)
        for col,row in cells:self.cells.pop((int(row),int(col)),None)

    def planner(self,now):
        if self.last_received is None or not 0<=now-self.last_received<.8:
            raise RuntimeError('实时障碍点云已过期')
        layer=copy.copy(self.static)
        occupied=np.zeros_like(self.static.occupied)
        for (row,col),at in self.cells.items():
            if now-at<=self.retention_s:occupied[row,col]=True
        if occupied.any():
            distance=np.maximum(0.,ndimage.distance_transform_edt(~occupied)*layer.resolution-layer.half_diagonal)
            # Preserve the static footprint and use a 0.40 m live corridor.
            clearance=distance-max(0.,LIVE_TRAVEL_RADIUS-layer.radius)
            layer.clearance=np.minimum(self.static.clearance,clearance)
            # Turn thresholds (.48/.38) differ from their live counterparts
            # (.55/.45) by .07 m; avoid inflating twice by the travel margin.
            turn_clearance=np.minimum(self.static.clearance,distance-.07)
        else:layer.clearance=self.static.clearance.copy()
        if not occupied.any():turn_clearance=self.static.clearance.copy()
        def turn_at(xy):
            row,col=layer.cell(xy);centre=(layer.T@[col,row,1])[:2]
            return max(0.,float(turn_clearance[row,col]-np.linalg.norm(np.asarray(xy)-centre)))
        layer.turn_clearance_at=turn_at
        layer.free=layer.clearance>layer.radius+layer.half_diagonal
        layer.occupied=self.static.occupied|occupied
        return layer

def validate_route_turns(planner,path,yaw,initial_clearance,route_clearance,align):
    heading=float(yaw)
    for index,(a,b) in enumerate(zip(path[:-1],path[1:])):
        if not planner.segment_free(a,b):raise ValueError('新路线存在障碍')
        delta=np.asarray(b)-a
        if np.linalg.norm(delta)<1e-6:continue
        desired=math.atan2(delta[1],delta[0])
        error=math.atan2(math.sin(desired-heading),math.cos(desired-heading))
        if abs(error)>align:
            limit=initial_clearance if index==0 else route_clearance
            if getattr(planner,'turn_clearance_at',planner.clearance_at)(a)<=limit:
                raise ValueError('新路线转弯处净空不足')
        heading=desired
