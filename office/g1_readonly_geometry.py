"""Model kinematics, subscribe-only telemetry and static route preview."""
import heapq,json,threading,time
import xml.etree.ElementTree as ET
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation

class Telemetry:
    def __init__(self):
        self.samples={};self.lock=threading.Lock();self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        import websocket
        topics=['/lowstate']
        while not self.stop.is_set():
            ws=None
            try:
                ws=websocket.create_connection('ws://127.0.0.1:19091',timeout=3);ws.settimeout(1)
                for i,topic in enumerate(topics):
                    ws.send(json.dumps({'op':'subscribe','topic':topic,'id':f'base_view_{i}',
                        'queue_length':1,'throttle_rate':100}))
                while not self.stop.is_set():
                    try:item=json.loads(ws.recv())
                    except websocket.WebSocketTimeoutException:continue
                    if item.get('op')=='publish' and item.get('topic') in topics:
                        with self.lock:self.samples[item['topic']]=(time.time(),item['msg'])
            except Exception:self.stop.wait(2)
            finally:
                if ws is not None:
                    for i,topic in enumerate(topics):
                        try:ws.send(json.dumps({'op':'unsubscribe','topic':topic,'id':f'base_view_{i}'}))
                        except Exception:pass
                    ws.close()
    def get(self):
        with self.lock:return self.samples.copy()
    def close(self):self.stop.set();self.thread.join(timeout=4)

class BaseModel:
    def __init__(self,path):
        self.model=ET.parse(path).getroot()
        self.joints={j.attrib['name']:j for j in self.model.findall('joint')}
    def joint(self,name,angle=0):
        joint=self.joints[name];o=joint.find('origin');t=np.eye(4)
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.attrib.get('rpy','0 0 0'),sep=' ')).as_matrix()
        t[:3,3]=np.fromstring(o.attrib.get('xyz','0 0 0'),sep=' ')
        if joint.attrib['type']!='fixed':
            axis=np.fromstring(joint.find('axis').attrib['xyz'],sep=' ')
            t[:3,:3]=t[:3,:3]@Rotation.from_rotvec(axis*angle).as_matrix()
        return t
    def candidates(self,samples):
        now=time.time();low_at,low=samples['/lowstate'];imu_at,imu=samples['virtual_imu']
        if not all(-.1<=now-at<=1.5 for at in (low_at,imu_at)):raise ValueError('关节或 IMU 数据已过期')
        angles=np.array([low['motor_state'][i]['q'] for i in (12,13,14)])
        if not np.isfinite(angles).all():raise ValueError('腰部角度无效')
        pelvis_torso=np.eye(4)
        for name,a in zip(['waist_yaw_joint','waist_roll_joint','waist_pitch_joint'],angles):pelvis_torso=pelvis_torso@self.joint(name,a)
        pelvis_mid=pelvis_torso@self.joint('mid360_joint')
        pelvis_up=np.asarray(low['imu_state']['accelerometer'],float)
        sensor_up=np.asarray(imu['linear_acceleration'],float)
        if not 7<np.linalg.norm(pelvis_up)<12 or not 7<np.linalg.norm(sensor_up)<12:
            raise ValueError('加速度不是稳定重力观测')
        imu_lidar=np.eye(4);imu_lidar[:3,3]=[-.011,.02329,-.04412]
        roll=np.diag([1.,-1.,-1.,1.]);candidates=[]
        for label,extra in [('原始雷达坐标假设',np.eye(4)),('倒置安装转换假设',roll)]:
            t=pelvis_mid@extra@roll@np.linalg.inv(imu_lidar)
            a=t[:3,:3]@(sensor_up/np.linalg.norm(sensor_up));b=pelvis_up/np.linalg.norm(pelvis_up)
            error=float(np.rad2deg(np.arccos(np.clip(a@b,-1,1))))
            candidates.append({'name':label,'T_imu_from_pelvis':np.linalg.inv(t),
                'gravity_error_degrees':error})
        candidates.sort(key=lambda c:c['gravity_error_degrees'])
        if candidates[0]['gravity_error_degrees']>10 or candidates[1]['gravity_error_degrees']-candidates[0]['gravity_error_degrees']<100:
            raise ValueError('模型安装方向与实测重力不一致')
        return candidates,angles

    def tracked(self,samples,session):
        """Validate the fixed mounting once per session; keep live waist kinematics.

        Accelerometers during locomotion include body acceleration and cannot be
        used to repeatedly infer which way a bolted-down sensor was installed.
        Fresh samples and finite joint angles remain mandatory on every call.
        """
        if not session:raise ValueError('缺少底座定位会话')
        if getattr(self,'_mount_session',None)!=session:
            self._mount_session=session;self._validated_mount=None
        if self._validated_mount is None:
            candidates,angles=self.candidates(samples)
            self._validated_mount={k:v for k,v in candidates[0].items() if k!='T_imu_from_pelvis'}
            self._validated_mount['alternative_gravity_error_degrees']=candidates[1]['gravity_error_degrees']
            self._validated_mount['mount_validated_unix_s']=time.time()
        now=time.time();low_at,low=samples['/lowstate'];imu_at,imu=samples['virtual_imu']
        if not all(-.1<=now-at<=1.5 for at in (low_at,imu_at)):raise ValueError('关节或 IMU 数据已过期')
        acceleration=np.asarray(imu['linear_acceleration'],float)
        if acceleration.shape!=(3,) or not np.isfinite(acceleration).all():raise ValueError('IMU 数据无效')
        angles=np.array([low['motor_state'][i]['q'] for i in (12,13,14)])
        if not np.isfinite(angles).all():raise ValueError('腰部角度无效')
        pelvis_torso=np.eye(4)
        for name,a in zip(['waist_yaw_joint','waist_roll_joint','waist_pitch_joint'],angles):
            pelvis_torso=pelvis_torso@self.joint(name,a)
        pelvis_mid=pelvis_torso@self.joint('mid360_joint')
        imu_lidar=np.eye(4);imu_lidar[:3,3]=[-.011,.02329,-.04412]
        roll=np.diag([1.,-1.,-1.,1.])
        extra=np.eye(4) if self._validated_mount['name']=='原始雷达坐标假设' else roll
        t=pelvis_mid@extra@roll@np.linalg.inv(imu_lidar)
        return {**self._validated_mount,'T_imu_from_pelvis':np.linalg.inv(t),
                'lowstate_received_unix_s':low_at,'imu_received_unix_s':imu_at},angles

class StaticRoute:
    def __init__(self,path,radius=.28):
        with np.load(path,allow_pickle=False) as data:
            self.occupied=data['occupancy'].astype(bool);self.T=data['T_map_px'].copy()
        self.inverse=np.linalg.inv(self.T)
        self.resolution=float(np.linalg.norm(self.T[:2,0]));self.radius=radius
        padded=np.pad(~self.occupied,1,constant_values=False)
        self.half_diagonal=self.resolution/np.sqrt(2)
        self.clearance=np.maximum(0,ndimage.distance_transform_edt(padded)[1:-1,1:-1]*self.resolution-self.half_diagonal)
        # A grid centre being free does not ensure its connecting edge is free.
        # Reserve the cell half diagonal for between-cell interpolation.
        self.free=self.clearance>radius+self.half_diagonal
        rows,cols=np.nonzero(self.occupied)
        self.obstacle_xy=(self.T@np.stack([cols,rows,np.ones(len(rows))]))[:2].T
    def clearance_at(self,xy):
        row,col=self.cell(xy)
        center=(self.T@[col,row,1])[:2]
        return max(0.,float(self.clearance[row,col]-np.linalg.norm(np.asarray(xy)-center)))
    def cell(self,xy):
        col,row=np.rint((self.inverse@[*xy,1])[:2]).astype(int)
        if not(0<=row<self.free.shape[0] and 0<=col<self.free.shape[1]):raise ValueError('位置超出地图')
        return int(row),int(col)
    def segment_free(self,start,goal):
        n=max(2,int(np.linalg.norm(np.asarray(goal)-start)/(self.resolution/3))+2)
        try:return all(self.clearance_at(p)>self.radius for p in np.linspace(start,goal,n))
        except ValueError:return False
    def simplify(self,path):
        """Use collision-checked long segments instead of chasing 5 cm cells."""
        path=np.asarray(path,float)
        path=path[np.r_[True,np.linalg.norm(np.diff(path,axis=0),axis=1)>1e-6]]
        out=[path[0]];i=0
        while i<len(path)-1:
            j=len(path)-1
            while j>i+1 and not self.segment_free(path[i],path[j]):j-=1
            if not self.segment_free(path[i],path[j]):raise ValueError('路线连接段净空不足')
            out.append(path[j]);i=j
        return np.asarray(out)
    def departure_goal(self,goal,clearance=.68,max_shift=.65):
        """Choose a nearby stop with room for arrival error and another turn."""
        goal=np.asarray(goal,float)
        if self.clearance_at(goal)<=self.radius:raise ValueError('目标净空不足')
        if self.clearance_at(goal)>clearance:return goal.copy()
        # These candidates are exact cell centres. clearance already subtracts
        # the occupied-cell half diagonal; no interpolation reserve is needed.
        rows,cols=np.nonzero(self.clearance>clearance)
        xy=(self.T@np.stack([cols,rows,np.ones(len(rows))]))[:2].T
        distance=np.linalg.norm(xy-goal,axis=1)
        for i in np.argsort(distance):
            if distance[i]>max_shift:break
            if self.clearance_at(xy[i])>clearance and self.segment_free(goal,xy[i]):return xy[i]
        raise ValueError(f'目标附近 {max_shift:.2f} m 内没有可停车并再次转向的位置，请选更开阔的点')

    def maneuver_route(self,start,goal,yaw,*,initial_clearance=.48,route_clearance=.38,align=np.deg2rad(5)):
        """Search a traversable route, including a forward exit from a tight bay.

        Wide corridors are preferred costs, not a requirement on every cell.
        A tight start may leave straight ahead before its first checked turn;
        neither the footprint nor any turning guard is relaxed.
        """
        from g1_dynamic_navigation import validate_route_turns
        start=np.asarray(start,float);goal=np.asarray(goal,float)
        errors=[]
        def search(origin,heading,initial):
            for corridor in dict.fromkeys((route_clearance,.53)):
                try:
                    path,detail=self.route(origin,goal,interior_clearance=corridor,preferred_clearance=.53)
                    validate_route_turns(self,path,heading,initial,route_clearance,align)
                    return path,detail
                except ValueError as exc:errors.append(str(exc))
            return None
        result=search(start,yaw,initial_clearance)
        if result is not None:return result
        turn_at=getattr(self,'turn_clearance_at',self.clearance_at)
        if self.clearance_at(start)>self.radius and turn_at(start)<=initial_clearance:
            direction=np.array([np.cos(yaw),np.sin(yaw)])
            for distance in np.arange(.15,1.051,.10):
                exit_xy=start+distance*direction
                if not self.segment_free(start,exit_xy):break
                if turn_at(exit_xy)<=initial_clearance:continue
                result=search(exit_xy,yaw,route_clearance)
                if result is None:continue
                path,detail=result;path=np.vstack([start,path])
                validate_route_turns(self,path,yaw,initial_clearance,route_clearance,align)
                return path,{**detail,'length_m':detail['length_m']+float(distance),
                    'minimum_clearance_m':min(detail['minimum_clearance_m'],self.clearance_at(start)),
                    'forward_exit_m':float(distance)}
        raise ValueError(errors[-1] if errors else '没有可安全行走并转向的路线')

    def navigation_route(self,start,goal,*,yaw=None):
        goal=np.asarray(goal,float);stop=self.departure_goal(goal)
        if yaw is None:
            xy,detail=self.route(start,stop,interior_clearance=.38,preferred_clearance=.53)
        else:xy,detail=self.maneuver_route(start,stop,yaw)
        return xy,{**detail,'requested_goal_xy_m':goal.tolist(),
                   'stop_goal_xy_m':stop.tolist(),'goal_adjustment_m':float(np.linalg.norm(stop-goal)),
                   'stop_clearance_m':self.clearance_at(stop)}

    def route(self,start,goal,*,interior_clearance=None,preferred_clearance=None):
        a,b=self.cell(start),self.cell(goal)
        if self.clearance_at(start)<=self.radius:raise ValueError(f'起点保守净空 {self.clearance_at(start):.2f} m，不足 {self.radius:.2f} m')
        if not self.free[a]:raise ValueError('起点净空不足，禁止自动吸附到其他位置')
        if self.clearance_at(goal)<=self.radius:raise ValueError('目标净空不足')
        if not self.free[b]:raise ValueError('目标净空不足')
        search_free=self.free.copy()
        if interior_clearance is not None:
            search_free &= self.clearance>interior_clearance+self.half_diagonal
            # Exact endpoints must connect to the wider corridor through
            # collision-checked segments. Simply restoring their one grid cell
            # isolates a valid start whenever neighbouring cells are narrower.
            for endpoint,xy in ((a,start),(b,goal)):
                if search_free[endpoint]:continue
                rows,cols=np.nonzero(search_free)
                centres=(self.T@np.stack([cols,rows,np.ones(len(rows))]))[:2].T
                distances=np.linalg.norm(centres-np.asarray(xy),axis=1)
                connected=False
                for i in np.argsort(distances):
                    if distances[i]>.65:break
                    if not self.segment_free(xy,centres[i]):continue
                    n=max(2,int(distances[i]/(self.resolution/3))+2)
                    for point in np.linspace(xy,centres[i],n):
                        cell=self.cell(point)
                        if self.free[cell]:search_free[cell]=True
                    connected=True;break
                search_free[endpoint]=self.free[endpoint]
                if not connected:raise ValueError('起终点附近没有可安全连接的开阔通路')
        queue=[(0.,0.,a)];cost={a:0.};parent={}
        while queue:
            _,g,u=heapq.heappop(queue)
            if g!=cost[u]:continue
            if u==b:break
            for dr,dc in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                v=(u[0]+dr,u[1]+dc)
                if not(0<=v[0]<self.free.shape[0] and 0<=v[1]<self.free.shape[1]) or not search_free[v]:continue
                if dr and dc and not(search_free[u[0]+dr,u[1]] and search_free[u[0],u[1]+dc]):continue
                penalty=0. if preferred_clearance is None else 3*max(0.,preferred_clearance-float(self.clearance[v]))/preferred_clearance
                new=g+np.hypot(dr,dc)*(1+penalty)
                if new<cost.get(v,float('inf')):
                    cost[v]=new;parent[v]=u
                    heapq.heappush(queue,(new+np.hypot(v[0]-b[0],v[1]-b[1]),new,v))
        if b not in cost:raise ValueError('旧地图内没有足够净空的通路')
        cells=[b]
        while cells[-1]!=a:cells.append(parent[cells[-1]])
        cells.reverse();xy=np.array([(self.T@[c,r,1])[:2] for r,c in cells])
        xy=np.vstack([start,xy,goal])
        xy=self.simplify(xy)
        return xy,{'length_m':float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),
            'minimum_clearance_m':float(min(self.clearance_at(start),self.clearance_at(goal),min(self.clearance[c] for c in cells))),
            'radius_m':self.radius,'source':'static old occupancy map; preview only'}
