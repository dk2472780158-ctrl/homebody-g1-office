"""Authenticated access to the existing G1 teleop owner; no direct ROS publisher."""
import hb_config
import argparse,json,math,stat,sys,threading,time
from pathlib import Path
import numpy as np
from g1_dynamic_navigation import DynamicObstacleLayer,ObstacleBlocked,validate_route_turns
from g1_pointcloud_slice import obstacle_slice
from g1_control_http import ControlHTTP,ControlTransportError

TOKEN=Path(hb_config.expand('${HB_TELEOP_TOKEN_FILE}'))
URL='http://127.0.0.1:18098'
SPEED=.35
PROBE_SPEED=SPEED
PROBE_DURATION=3.
TURN_RATE=.45
TURN_TOLERANCE=math.radians(4)
TURN_CLEARANCE=.48
ROUTE_TURN_CLEARANCE=.38
LIVE_TURN_CLEARANCE=.55
LIVE_ROUTE_TURN_CLEARANCE=.45
TRAVEL_ALIGN=math.radians(5)
TRAVEL_REALIGN=math.radians(20)
TRAVEL_YAW_RATE=.25
NAVIGATION_TURN_TOLERANCE=math.radians(10)
WALK_START_SECONDS=.6
TURN_SOFT_DRIFT=.15
TURN_MAX_DRIFT=.25
TURN_SEGMENT_ANGLE=math.radians(45)
ROUTE_CORRECTION_DISTANCE=.30
ROUTE_CORRECTION_ANGLE=math.radians(30)
STOPFILE=Path(hb_config.expand('${HB_OFFICE_ROOT}/real-motion-stop'))

class RoutePoseCorrection(RuntimeError):
    """A bounded route correction requiring a stopped, verified new start."""
    def __init__(self,message,pose):
        super().__init__(message);self.pose=np.asarray(pose,float).copy()

class LocalizationUnavailable(ValueError):
    """Fresh same-session AMCL validation may recover while navigation waits."""
    def __init__(self,message,state):
        super().__init__(message);self.state=state

def corroborated_map_correction(before,after,previous,current):
    """Separate a map-frame correction from physical sliding or wrong turning."""
    try:
        old=np.asarray(before['T_office_from_current'],float)
        new=np.asarray(after['T_office_from_current'],float)
        if old.shape!=(4,4) or new.shape!=(4,4):return False
        predicted=new@np.linalg.inv(old)@previous
        shift=float(np.linalg.norm(predicted[:2,3]-previous[:2,3]))
        angle=abs(angle_diff(pose_yaw(predicted),pose_yaw(previous)))
        return bool(np.isfinite(predicted).all() and
            (shift>.04 or angle>math.radians(3)) and
            shift<=ROUTE_CORRECTION_DISTANCE and angle<=ROUTE_CORRECTION_ANGLE and
            np.linalg.norm(current[:2,3]-previous[:2,3])<=ROUTE_CORRECTION_DISTANCE and
            abs(angle_diff(pose_yaw(current),pose_yaw(previous)))<=ROUTE_CORRECTION_ANGLE and
            np.linalg.norm(current[:2,3]-predicted[:2,3])<=.06 and
            abs(angle_diff(pose_yaw(current),pose_yaw(predicted)))<=math.radians(5))
    except (KeyError,TypeError,ValueError,np.linalg.LinAlgError):return False

def task_speed(steps,probe=False):
    if not probe:return SPEED
    if len(steps)!=1 or steps[0].get('skill')!='move_for' or steps[0].get('arguments')!={'direction':'forward','duration_s':PROBE_DURATION}:
        raise ValueError(f'短时验证仅允许向机身前方移动 {PROBE_DURATION:g} 秒')
    return PROBE_SPEED

class Teleop:
    def __init__(self):
        if stat.S_IMODE(TOKEN.stat().st_mode)!=0o600:raise RuntimeError('机器人凭据文件权限应为 600')
        self._token=TOKEN.read_text().strip();self.lock=threading.Lock()
        self.transport=ControlHTTP(URL,self._token)
    def request(self,path,body=None,timeout=1.):
        return self.transport.request(path,body,timeout)
    def health(self):return self.request('/health')
    def enable(self):
        h=self.health();owner=h.get('vel_owner',{}).get('owner')
        if owner not in ('none','teleop'):raise RuntimeError(f'控制权正在由 {owner} 持有')
        if h.get('write_blocked_reason'):raise RuntimeError(control_block_reason(h))
        if h.get('busy'):raise RuntimeError('机器人正在切换状态，请等待')
        return self.request('/teleop/arm',{'armed':True},timeout=35.)
    def velocity(self,vx,vy,wz,cancel,*,probe=False):
        limit=PROBE_SPEED if probe else SPEED
        if not all(math.isfinite(v) for v in (vx,vy,wz)) or math.hypot(vx,vy)>limit+1e-6 or abs(wz)>TURN_RATE+1e-6:
            raise ValueError('验证速度越界')
        # Existing bridge negates lateral and yaw before Unitree SetVelocity.
        with self.lock:
            if cancel.is_set() or STOPFILE.exists():raise RuntimeError('任务已停止')
            reply=self.request('/teleop/cmd_vel',{'vx':vx,'vy':-vy,'wz':-wz},timeout=.35)
        if not reply.get('ok'):raise RuntimeError('机器人未接受速度')
        actual=reply.get('cmd')
        if actual is not None:
            expected={'vx':vx,'vy':-vy,'wz':-wz}
            if any(not math.isfinite(float(actual.get(k,float('nan')))) or abs(float(actual[k])-v)>.005 for k,v in expected.items()):
                self.zero()
                raise RuntimeError('机器人接口裁剪了速度指令，已停止；请检查控制链限速和地图限制')
        return reply
    def zero(self):
        with self.lock:
            for attempt in range(2):
                try:return self.request('/teleop/cmd_vel',{'vx':0.,'vy':0.,'wz':0.},timeout=.7)
                except ControlTransportError:
                    if attempt:raise
    def gesture_capabilities(self):
        return self.request('/teleop/gestures/status')
    def stop_gesture(self,job_id):
        return self.request('/teleop/gestures/stop',{'job_id':job_id},timeout=.7)
    def gesture(self,name,cancel,check,on_start=None):
        if name not in ('wave','handshake'):raise ValueError('不支持的手势')
        if cancel.is_set() or STOPFILE.exists():raise RuntimeError('任务已停止')
        self.zero();check()
        reply=self.request('/teleop/gestures/start',{'name':name},timeout=1.)
        job_id=reply['job_id'];deadline=time.monotonic()+16.
        try:
            if on_start is not None:on_start()
            while True:
                if cancel.is_set() or STOPFILE.exists():raise RuntimeError('手势已停止')
                if time.monotonic()>deadline:raise RuntimeError('手势执行超时')
                check()
                job=self.request('/teleop/gestures/heartbeat',{'job_id':job_id},timeout=.7)
                if job['state']=='done':return job['result']
                if job['state']!='running':raise RuntimeError('官方手势执行失败：'+str(job.get('result',{}).get('error',job['state'])))
                cancel.wait(.2)
        finally:
            self.stop_gesture(job_id)

def control_block_reason(h):
    reason=str(h.get('write_blocked_reason') or '')
    if reason.startswith('battery critical'):
        battery=h.get('battery',{});soc=battery.get('soc','未知');threshold=h.get('soc_crit_pct',20)
        return f'机器人电量 {soc}%，低于行走门限 {threshold}%，请先充电'
    return '机器人控制被阻止：'+reason

def require_ready(h):
    if h.get('write_blocked_reason'):raise RuntimeError(control_block_reason(h))
    limits=h.get('limits') or {}
    if limits and any(float(limits.get(axis,0))<SPEED-1e-6 for axis in ('vx','vy')):
        raise RuntimeError(f'机器人行走限速低于任务速度 {SPEED:.2f} m/s，请先同步控制链配置')
    if not h.get('armed'):raise RuntimeError('请先点击“使能腿控制”；该操作可能触发站立')
    if h.get('busy'):raise RuntimeError('机器人正在切换状态，请等待')
    if h.get('read_only'):raise RuntimeError('机器人控制服务处于只读状态')
    if not h.get('ros'):raise RuntimeError('机器人 ROS 控制连接不可用')
    if h.get('vel_owner',{}).get('owner')!='teleop':raise RuntimeError('现有遥操作通道未持有控制权')

def world_to_body(velocity,pose):
    yaw=math.atan2(pose[1,0],pose[0,0]);c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,s],[-s,c]])@velocity

def pose_yaw(pose):return math.atan2(pose[1,0],pose[0,0])
def angle_diff(target,current):return math.atan2(math.sin(target-current),math.cos(target-current))

def projected_travel(start,velocity,yaw_rate,horizon=.8):
    times=np.linspace(0.,horizon,6)
    velocity=np.asarray(velocity,float)
    if abs(yaw_rate)<1e-9:return np.asarray(start)+times[:,None]*velocity
    angles=times*yaw_rate
    side=np.array([-velocity[1],velocity[0]])
    return (np.asarray(start)+np.sin(angles)[:,None]/yaw_rate*velocity
            +(1-np.cos(angles))[:,None]/yaw_rate*side)

class Executor:
    def __init__(self,workflow):self.w=workflow;self.client=Teleop();self.last_health=0.;self.commands_sent=0;self.gestures_sent=0;self.gestures_started=0;self.last_turn=None
    def gesture_started(self):self.gestures_started+=1
    def observe_after_stop(self,session,timeout=1.5):
        deadline=time.monotonic()+timeout;latest=None
        while time.monotonic()<deadline:
            if self.w.cancel.is_set() or STOPFILE.exists():break
            try:
                state,pose=self.w.snapshot();now=time.time()
                if state['session_id']!=session:return None
                if all(0<=now-float(state.get(k) or 0)<.8 for k in ('odometry_received_unix_s','cloud_received_unix_s')):
                    latest=(pose.copy(),min(float(state[k]) for k in ('odometry_received_unix_s','cloud_received_unix_s')))
            except (OSError,ValueError,KeyError,TypeError):pass
            self.w.cancel.wait(min(.1,max(0,deadline-time.monotonic())))
        return latest[0] if latest is not None and 0<=time.time()-latest[1]<.8 else None
    def fresh_snapshot(self,session):
        w=self.w
        if w.cancel.is_set() or STOPFILE.exists():raise RuntimeError('任务已停止')
        state,pose=w.snapshot();now=time.time()
        if state['session_id']!=session:raise RuntimeError('定位会话已改变，已停止')
        for key in ('odometry_received_unix_s','cloud_received_unix_s'):
            if not 0<=now-float(state.get(key) or 0)<.8:raise RuntimeError('定位或点云接收不够新鲜，已停止')
        if time.monotonic()-self.last_health>.7:
            require_ready(self.client.health());self.last_health=time.monotonic()
        return state,pose
    def live_obstacles(self,session,state,pose):
        w=self.w;now=time.time();start=pose[:2,3]
        with np.load(w.live/'clouds.npz',allow_pickle=False) as data:
            received=float(data['received_unix_s'])
            if str(data['session_id'])!=session:raise RuntimeError('实时障碍点云会话不符')
            if not 0<=now-received<.8:raise RuntimeError('实时障碍点云已过期')
            xyz=data['latest_xyz'].copy()
        transform=np.array(state['T_office_from_current']);xyz=xyz@transform[:3,:3].T+transform[:3,3]
        if not np.isfinite(xyz).all():raise RuntimeError('实时障碍点云包含无效数值')
        obstacles=obstacle_slice(xyz,w.floor,start)
        if getattr(self,'dynamic_obstacles',None) is not None:
            self.dynamic_obstacles.clear_observed_rays(obstacles[:,:2],start,received)
            self.dynamic_obstacles.observe(obstacles[:,:2],received,now,session)
        return obstacles
    def guard(self,session,world_velocity,*,turning=False,yaw_rate=0.):
        w=self.w;state,pose=self.fresh_snapshot(session)
        self.last_guard_state=state
        self.last_guard_pose=pose.copy()
        horizon=1.5 if getattr(self,'dynamic_obstacles',None) is not None else .8
        remaining=getattr(self,'navigation_remaining_m',None)
        if remaining is not None and np.linalg.norm(world_velocity)>1e-9:
            horizon=min(horizon,max(0.,remaining)/np.linalg.norm(world_velocity))
        start=pose[:2,3];travel=projected_travel(start,world_velocity,yaw_rate,horizon=horizon)
        for point in travel:
            limit=max(w.planner.radius+.1,ROUTE_TURN_CLEARANCE if turning=='route' else TURN_CLEARANCE) if turning else w.planner.radius
            clearance=w.planner.clearance_at(point)
            if clearance<=limit:
                raise ObstacleBlocked(f'静态净空 {clearance:.2f} m，不足 {limit:.2f} m，已停止')
        obstacles=self.live_obstacles(session,state,pose)
        if len(obstacles):
            if turning:
                limit=LIVE_ROUTE_TURN_CLEARANCE if turning=='route' else LIVE_TURN_CLEARANCE
                if np.count_nonzero(np.linalg.norm(obstacles[:,:2]-start,axis=1)<limit)>=3:
                    raise ObstacleBlocked('转身范围内有实时障碍，已停止')
            else:
                distance=np.full(len(obstacles),np.inf)
                for a,b in zip(travel[:-1],travel[1:]):
                    delta=b-a;u=np.clip((obstacles[:,:2]-a)@delta/(delta@delta+1e-12),0,1)
                    distance=np.minimum(distance,np.linalg.norm(obstacles[:,:2]-(a+u[:,None]*delta),axis=1))
                if np.count_nonzero(distance<.4)>=3:raise ObstacleBlocked('实时点云前方有障碍，已停止')
        return pose
    def navigation_update(self,phase,message,path=None):
        callback=getattr(self.w,'navigation_update',None)
        if callback is not None:callback(phase,message,path)
    def wait_for_localization(self,session,reference,deadline,initial):
        self.client.zero()
        if self.localization_waits>=3:raise RuntimeError('定位反复中断，已停止，请重新预览')
        self.localization_waits+=1
        self.navigation_update('localizing','定位暂时复核中，已停车等待；恢复稳定后重新规划到原目标（最多 30 秒）。')
        until=min(deadline,time.monotonic()+30.);stable_since=None;anchor=None;frames=set()
        pending=initial
        while time.monotonic()<until:
            self.client.zero()
            if self.w.cancel.is_set() or STOPFILE.exists():raise RuntimeError('任务已停止')
            if pending is not None:
                state=pending.state;now=time.time()
                if state.get('session_id')!=session:raise RuntimeError('定位会话已改变，已停止')
                if state.get('localization_backend')!='nav2_amcl' or state.get('status')!='matching':raise pending
                if any(not 0<=now-float(state.get(k) or 0)<.8 for k in
                       ('updated_unix_s','cloud_received_unix_s','odometry_received_unix_s')):
                    raise RuntimeError('等待定位时传感器不够新鲜，已停止')
                pending=None
            if time.monotonic()-self.last_health>.7:
                require_ready(self.client.health());self.last_health=time.monotonic()
            try:
                pose=self.guard(session,np.zeros(2));state=self.last_guard_state
            except LocalizationUnavailable as exc:
                pending=exc;stable_since=None;anchor=None;frames.clear()
                self.w.cancel.wait(.1);continue
            if (np.linalg.norm(pose[:2,3]-reference[:2,3])>.40 or
                abs(angle_diff(pose_yaw(pose),pose_yaw(reference)))>math.radians(30)):
                raise RuntimeError('重定位结果与停车位置差异过大，已停止，请重新预览')
            if (anchor is None or np.linalg.norm(pose[:2,3]-anchor[:2,3])>.04 or
                abs(angle_diff(pose_yaw(pose),pose_yaw(anchor)))>math.radians(4)):
                anchor=pose.copy();stable_since=time.monotonic();frames.clear()
            frames.add((state['odometry_received_unix_s'],state['cloud_received_unix_s']))
            if time.monotonic()-stable_since>=.8 and len(frames)>=3:return pose
            self.w.cancel.wait(.1)
        raise RuntimeError('停车等待定位超过 30 秒或任务时限，已停止，请重新预览')
    def recover_navigation_localization(self,session,goal,reference,deadline,initial):
        while True:
            try:
                restored=self.wait_for_localization(session,reference,deadline,initial)
                return self.replan(session,goal,deadline,
                    RoutePoseCorrection('定位恢复稳定，从当前停车点重新规划',restored))
            except LocalizationUnavailable as exc:
                initial=exc
                reference=getattr(self,'last_guard_pose',None) if getattr(self,'last_guard_pose',None) is not None else reference
    def settle_route_correction(self,session,reason,deadline):
        # Every sample is fresh and collision checked. Repeated copies of one
        # sensor frame cannot establish stability; no velocity is sent here.
        self.client.zero();until=min(deadline,time.monotonic()+4.)
        anchor=reason.pose.copy();stable_since=time.monotonic();frames=set()
        while time.monotonic()<until:
            self.client.zero();pose=self.guard(session,np.zeros(2));state=self.last_guard_state
            if (np.linalg.norm(pose[:2,3]-reason.pose[:2,3])>ROUTE_CORRECTION_DISTANCE or
                abs(angle_diff(pose_yaw(pose),pose_yaw(reason.pose)))>ROUTE_CORRECTION_ANGLE):
                raise RuntimeError('停车后定位仍明显跳变，已停止，请重新预览')
            if (np.linalg.norm(pose[:2,3]-anchor[:2,3])>.04 or
                abs(angle_diff(pose_yaw(pose),pose_yaw(anchor)))>math.radians(4)):
                anchor=pose.copy();stable_since=time.monotonic();frames.clear()
            frames.add((state['odometry_received_unix_s'],state['cloud_received_unix_s']))
            if time.monotonic()-stable_since>=.8 and len(frames)>=3:return
            self.w.cancel.wait(.1)
        raise RuntimeError('停车后定位未稳定，已停止，请重新预览')
    def replan(self,session,goal,deadline,reason):
        # Zero before snapshot, costmap construction, A*, or any retry.
        self.client.zero()
        correction=isinstance(reason,RoutePoseCorrection)
        if correction and sum(e.get('kind')=='pose_correction' for e in self.replans)>=3:
            raise RuntimeError('定位反复修正，已停止，请重新预览')
        if sum(e.get('kind')!='pose_correction' for e in self.replans)>=8:
            raise RuntimeError('遇障重规划次数已达上限，已停止，请重新预览')
        event={'started_unix_s':time.time(),'reason':str(reason),'goal_xy_m':np.asarray(goal).tolist(),
               'status':'waiting','attempts':0,'kind':'pose_correction' if correction else 'obstacle'}
        self.replans.append(event)
        self.last_navigation={'phase':'replanning','reason':str(reason)}
        self.navigation_update('replanning','定位有小幅修正，已停车；正在确认稳定位置并重新规划。' if correction else
                               '前方受阻，已停车；正在更新障碍地图并重新规划。')
        if correction:
            try:self.settle_route_correction(session,reason,deadline)
            except (RuntimeError,ValueError) as exc:
                event.update(status='unstable',finished_unix_s=time.time(),error=str(exc))
                raise
            # Previously cached live obstacles used the old map transform.
            # Rebuild from fresh cloud; the static map remains intact.
            self.dynamic_obstacles=DynamicObstacleLayer(self.w.planner,session)
        until=min(deadline,time.monotonic()+8.);last_error='没有可用通路'
        settle_until=min(until,time.monotonic()+.6)
        while time.monotonic()<until:
            state,pose=self.fresh_snapshot(session)
            self.client.zero();obstacles=self.live_obstacles(session,state,pose)
            planner=self.dynamic_obstacles.planner(time.time());event['attempts']+=1
            event['start_xy_m']=pose[:2,3].tolist()
            event['static_start_clearance_m']=self.w.planner.clearance_at(pose[:2,3])
            event['combined_start_clearance_m']=planner.clearance_at(pose[:2,3])
            event['obstacle_cells']=len(self.dynamic_obstacles.cells)
            event['nearest_live_obstacle_m']=float(np.linalg.norm(obstacles[:,:2]-pose[:2,3],axis=1).min()) if len(obstacles) else None
            if time.monotonic()<settle_until:
                self.w.cancel.wait(.1);continue
            try:
                # The reviewed stopping target remains fixed across recovery.
                if getattr(planner,'turn_clearance_at',planner.clearance_at)(goal)<=TURN_CLEARANCE:
                    raise ValueError('目标停车区域被障碍占据')
                path,detail=planner.maneuver_route(pose[:2,3],goal,pose_yaw(pose),
                    initial_clearance=TURN_CLEARANCE,route_clearance=ROUTE_TURN_CLEARANCE,align=TRAVEL_ALIGN)
                validate_route_turns(planner,path,pose_yaw(pose),TURN_CLEARANCE,
                                     ROUTE_TURN_CLEARANCE,TRAVEL_ALIGN)
            except ValueError as exc:
                last_error=str(exc);event['last_block_reason']=last_error
                self.navigation_update('waiting','已停车，暂时无可行绕行路线：'+last_error+'；等待障碍变化（最多 8 秒）。')
                self.w.cancel.wait(.2);continue
            # Planning itself may have taken long enough for telemetry to expire.
            _,after=self.fresh_snapshot(session)
            if (np.linalg.norm(after[:2,3]-pose[:2,3])>.05 or
                abs(angle_diff(pose_yaw(after),pose_yaw(pose)))>math.radians(5)):
                # Walking stop and delayed pose feedback can still settle after
                # the first search. Discard this path and plan from a stable
                # fresh start; no movement is sent while waiting.
                last_error='停车后位置仍在更新，正在重新取得稳定起点'
                event['last_block_reason']=last_error
                self.navigation_update('waiting',last_error)
                self.client.zero();settle_until=min(until,time.monotonic()+.3)
                self.w.cancel.wait(.1);continue
            try:
                delta=path[-1]-path[0] if len(path)<2 else path[1]-path[0]
                needs_turn=np.linalg.norm(delta)>1e-6 and abs(angle_diff(
                    math.atan2(delta[1],delta[0]),pose_yaw(after)))>TRAVEL_ALIGN
                self.guard(session,np.zeros(2),turning=True if needs_turn else False)
            except ObstacleBlocked as exc:
                last_error=str(exc);event['last_block_reason']=last_error;self.w.cancel.wait(.2);continue
            if time.monotonic()>=until:break
            event.update(status='planned',finished_unix_s=time.time(),route_xy_m=path.tolist(),
                         length_m=detail['length_m'])
            self.navigation_update('resuming','新路线检查通过，按新路线继续前往原目标。',path)
            return path if len(path)>1 else np.vstack([path[0],goal])
        event.update(status='no_path',finished_unix_s=time.time(),error=last_error)
        self.navigation_update('blocked','已停车，持续无可行绕行路线；本次任务结束。')
        raise RuntimeError('遇障停车后仍无可行路线：'+last_error)
    def rotate(self,session,start_yaw,goal,turn_sign,start,*,route=False,tolerance=TURN_TOLERANCE,target_xy=None):
        """Turn continuously; 45-degree support windows are observation checks.

        Crossing a support window does not brake or change the requested turn.
        Collision and displacement are checked before advancing each window.
        """
        if goal<=tolerance:return
        begun=time.monotonic()
        if not hasattr(self,'turn_segments'):self.turn_segments=[]
        self._turn_context=dict(target_xy=target_xy,deadline=begun+min(55.,goal/min(.08,TURN_RATE)+15.))
        try:self.rotate_segment(session,start_yaw,goal,turn_sign,start,route=route,tolerance=tolerance)
        finally:self._turn_context=None
    def rotate_segment(self,session,start_yaw,goal,turn_sign,start,*,route=False,tolerance=TURN_TOLERANCE):
        # The accepted yaw rate is an upper bound; allow slower measured turning
        # to reach the angle, while the independent no-progress guard still stops.
        begun=time.monotonic();deadline=begun+min(55.,goal/min(.08,TURN_RATE)+15.)
        context=getattr(self,'_turn_context',None)
        if context is not None:deadline=min(deadline,context['deadline'])
        anchor=start_yaw;last_yaw=start_yaw;progress=0.;last_sample=begun
        brake_until=0.;settled=False;measured_rate=0.;brake_count=0
        command_active=False;active_without_progress=0.;commanded_seconds=0.
        previous_position=np.asarray(start).copy();drift_paused=False;drift_pause_until=0.;peak_drift=0.
        previous_pose=np.eye(4);previous_pose[:2,3]=start
        previous_pose[:2,:2]=[[math.cos(start_yaw),-math.sin(start_yaw)],[math.sin(start_yaw),math.cos(start_yaw)]]
        previous_state=self.w.snapshot()[0]
        window_start=np.asarray(start).copy();window_progress=0.;window_peak=0.;segment_index=1
        any_drift_pause=False;rate_samples=[(begun,0.)]
        while True:
            tick=time.monotonic();pose=self.guard(session,np.zeros(2),turning='route' if route else True)
            current=pose_yaw(pose);increment=angle_diff(current,last_yaw)
            position_jump=np.linalg.norm(pose[:2,3]-previous_position)>max(.12,.2*max(.01,tick-last_sample))
            if (abs(increment)>math.radians(15) or position_jump) and context is not None and context.get('target_xy') is not None:
                if corroborated_map_correction(previous_state,getattr(self,'last_guard_state',None),previous_pose,pose):
                    planner=getattr(self.w,'planner',None)
                    if planner is not None and planner.segment_free(previous_position,pose[:2,3]):
                        self.client.zero()
                        raise RoutePoseCorrection('转身时地图坐标小幅修正，从稳定位置更新路线',pose)
            if abs(increment)>math.radians(15):raise RuntimeError('定位朝向跳变，已停止')
            progress+=increment*turn_sign;last_yaw=current
            dt=max(.01,tick-last_sample);last_sample=tick
            # FAST-LIO poses arrive in batches. Estimate rate across a window;
            # duplicate frames must not alternate zero and exaggerated rates.
            rate_samples.append((tick,progress))
            while len(rate_samples)>2 and rate_samples[1][0]<tick-.6:rate_samples.pop(0)
            span=tick-rate_samples[0][0]
            if span>.1:measured_rate=max(0.,(progress-rate_samples[0][1])/span)
            if command_active:
                active_without_progress+=dt;commanded_seconds+=dt
            if angle_diff(current,anchor)*turn_sign>=math.radians(2):
                anchor=current;active_without_progress=0.
            total_drift=float(np.linalg.norm(pose[:2,3]-start));peak_drift=max(peak_drift,total_drift)
            drift=float(np.linalg.norm(pose[:2,3]-window_start));window_peak=max(window_peak,drift)
            effective_goal=goal
            if context is not None and context.get('target_xy') is not None:
                delta=np.asarray(context['target_xy'])-pose[:2,3]
                if np.linalg.norm(delta)<.12:effective_goal=progress
                else:
                    bearing=math.atan2(delta[1],delta[0])
                    effective_goal=goal+angle_diff(bearing,start_yaw+turn_sign*goal)*turn_sign
            remaining=effective_goal-progress
            self.last_turn={'requested_yaw_rate_rad_s':turn_sign*TURN_RATE,
                'target_rotation_deg':math.degrees(goal),
                'observed_rotation_deg':math.degrees(progress),'elapsed_s':time.monotonic()-begun,
                'accepted_cmd':getattr(self,'last_turn',None).get('accepted_cmd') if getattr(self,'last_turn',None) else None,
                'start_xy_m':np.asarray(start).tolist(),'current_xy_m':pose[:2,3].tolist(),
                'displacement_m':total_drift,'peak_displacement_m':peak_drift,
                'segment_index':segment_index,'segment_displacement_m':drift,
                'segment_peak_displacement_m':window_peak,'maximum_segment_displacement_m':TURN_MAX_DRIFT,
                'drift_settle_used':any_drift_pause or drift_paused,
                'phase':'braking' if tick<brake_until else 'settling' if settled else 'turning' if command_active else 'starting',
                'commanded_seconds':commanded_seconds,'active_without_progress_s':active_without_progress,
                'brake_count':brake_count,'remaining_angle_deg':math.degrees(remaining),
                'effective_target_rotation_deg':math.degrees(effective_goal),'support_window_only':True}
            if drift>TURN_MAX_DRIFT:
                raise RuntimeError(f'单段转向位移异常（{drift:.2f} m），已停止')
            if np.linalg.norm(pose[:2,3]-previous_position)>max(.12,.2*dt):
                raise RuntimeError('转身定位位置跳变，已停止')
            planner=getattr(self.w,'planner',None)
            if planner is not None and hasattr(planner,'segment_free') and not planner.segment_free(previous_position,pose[:2,3]):
                raise RuntimeError('转身位移经过静态障碍区域，已停止')
            previous_position=pose[:2,3].copy()
            previous_pose=pose.copy();previous_state=getattr(self,'last_guard_state',None)
            if drift>TURN_SOFT_DRIFT and not drift_paused:
                self.client.zero();command_active=False;drift_paused=True;any_drift_pause=True;drift_pause_until=tick+.6
            if tick<drift_pause_until:
                self.client.zero();command_active=False;self.w.cancel.wait(.1);continue
            if progress<-math.radians(3):
                raise RuntimeError('实际转向与预览不符：观测到反方向转动，已停止')
            if remaining<-math.radians(8):
                raise RuntimeError('实际转向与预览不符：超过最终目标角度，已停止')
            if progress-window_progress>=TURN_SEGMENT_ANGLE:
                self.turn_segments.append(dict(segment=segment_index,requested_deg=math.degrees(TURN_SEGMENT_ANGLE),
                    observed_total_deg=math.degrees(progress),start_xy_m=window_start.tolist(),
                    end_xy_m=pose[:2,3].tolist(),continuous=True))
                window_start=pose[:2,3].copy();window_progress=progress;window_peak=0.
                segment_index+=1;drift_paused=False
            if time.monotonic()>deadline:raise RuntimeError('转身超时，已停止')
            # A fixed 800 ms wait alone can finish while delayed feedback still
            # shows the robot coasting. Keep zero until the recent yaw settles;
            # full-target overrun and all other guards continue on every sample.
            if tick<brake_until or (settled and measured_rate>math.radians(2)):
                self.client.zero();command_active=False;self.w.cancel.wait(.1);continue
            if settled and remaining<=tolerance:
                if progress-window_progress>math.radians(.1):
                    self.turn_segments.append(dict(segment=segment_index,
                        requested_deg=math.degrees(min(TURN_SEGMENT_ANGLE,max(0.,effective_goal-window_progress))),
                        observed_total_deg=math.degrees(progress),start_xy_m=window_start.tolist(),
                        end_xy_m=pose[:2,3].tolist(),continuous=True))
                break
            if active_without_progress>4:
                raise RuntimeError(f'转身响应不足：指令 {TURN_RATE:.2f} rad/s，观测转角 {math.degrees(progress):.1f}°，已停止')
            if settled:
                # A G1 step needs time to start. A 150 ms yaw pulse followed by
                # 600 ms stopped can reset that startup forever. Resume one
                # sustained request, using fresh feedback for the next brake.
                settled=False;measured_rate=0.;rate_samples=[(tick,progress)]
            braking_angle=max(tolerance,min(math.radians(25),measured_rate*.7))
            self.last_turn['braking_angle_deg']=math.degrees(braking_angle)
            if remaining<=tolerance or (command_active and remaining<=braking_angle):
                self.client.zero();command_active=False;brake_until=tick+.8
                settled=True;brake_count+=1;continue
            reply=self.client.velocity(0.,0.,turn_sign*TURN_RATE,self.w.cancel)
            command_active=True
            if isinstance(reply,dict) and reply.get('cmd'):
                self.last_turn['accepted_cmd']={k:reply['cmd'].get(k) for k in ('vx','vy','wz')}
            self.commands_sent+=1
            self.w.cancel.wait(max(0,min(.1-(time.monotonic()-tick),deadline-time.monotonic())))
    def face_travel(self,session,desired,*,route=False,continuous=False,target_xy=None):
        self.client.zero()
        # Let the stopped pose reach the feedback pipeline before establishing
        # a new rotation origin. Otherwise delayed walking samples look like
        # translation during an in-place turn.
        settle_until=time.monotonic()+.8
        while time.monotonic()<settle_until:
            self.guard(session,np.zeros(2),turning='route' if route else True)
            self.client.zero()
            self.w.cancel.wait(min(.1,max(0,settle_until-time.monotonic())))
        _,pose=self.w.snapshot();start_yaw=pose_yaw(pose);error=angle_diff(desired,start_yaw)
        tolerance=NAVIGATION_TURN_TOLERANCE if continuous else TURN_TOLERANCE
        if abs(error)<=(tolerance if continuous else TRAVEL_ALIGN):return
        self.rotate(session,start_yaw,abs(error),1 if error>0 else -1,pose[:2,3].copy(),route=route,tolerance=tolerance,target_xy=target_xy)
        self.client.zero()
    def run(self,steps,session,*,probe=False):
        speed=task_speed(steps,probe)
        records=[];self.last_health=0.;self.commands_sent=0;self.gestures_sent=0;self.gestures_started=0;self.last_turn=None
        self.forward_commands=0;self.last_navigation=None;self.replans=[];self.dynamic_obstacles=None;self.stop_error=None;self.turn_segments=[]
        self.localization_waits=0;self.last_guard_pose=None
        health=self.client.health();require_ready(health)
        if any(s['skill'] in ('turn_by','face') for s in steps) and float((health.get('limits') or {}).get('wz',TURN_RATE))<TURN_RATE-1e-6:
            raise RuntimeError('机器人转向限速低于任务角速度，未执行')
        try:
            for step in steps:
                name=step['skill']
                self.dynamic_obstacles=None;self.navigation_remaining_m=None
                if name in ('wave','handshake'):
                    started=time.time()
                    result=self.client.gesture(name,self.w.cancel,lambda:self.guard(session,np.zeros(2),turning=True),self.gesture_started)
                    self.gestures_sent+=1
                    records.append({'skill':name,'started_unix_s':started,'finished_unix_s':time.time(),
                        'command_completed':True,'official_acknowledged':True,'gesture_completed':True,
                        'motion_observed':False,'physical_motion_verified':False,'result':result})
                    continue
                started=time.time()
                try:state,pose=self.w.snapshot()
                except LocalizationUnavailable as exc:
                    if name in ('move_for','turn_by','face'):raise
                    reference=getattr(self,'last_guard_pose',None)
                    if reference is None:
                        # No current trusted heading is available at startup;
                        # don't automatically launch an invalidated preview.
                        raise
                    pose=self.wait_for_localization(session,reference,time.monotonic()+30.,exc)
                    state,_=self.fresh_snapshot(session)
                start=pose[:2,3].copy();start_yaw=pose_yaw(pose);turn_target=None
                if name=='move_for':
                    xy=np.asarray(step['route_xy_m']);unit=xy[-1]-xy[0];unit/=np.linalg.norm(unit)
                    facing=step.get('face_yaw_rad')
                    if facing is not None:self.face_travel(session,float(facing))
                    deadline=time.monotonic()+step['arguments']['duration_s']
                    while time.monotonic()<deadline:
                        tick=time.monotonic();velocity=unit*speed
                        if facing is not None:
                            _,before=self.w.snapshot();direction=pose_yaw(before)
                            velocity=np.array([math.cos(direction),math.sin(direction)])*speed
                        pose=self.guard(session,velocity)
                        if np.linalg.norm(pose[:2,3]-start)>speed*step['arguments']['duration_s']+.15:raise RuntimeError('实际位移超出本次验证范围')
                        if facing is not None:
                            if abs(angle_diff(float(facing),pose_yaw(pose)))>TRAVEL_ALIGN:raise RuntimeError('行走朝向偏离目标，已停止')
                            cmd=(speed,0.)
                        else:cmd=world_to_body(velocity,pose)
                        self.client.velocity(*cmd,0.,self.w.cancel,probe=probe)
                        if cmd[0]>0:self.forward_commands+=1
                        self.commands_sent+=1
                        self.w.cancel.wait(max(0,min(.1-(time.monotonic()-tick),deadline-time.monotonic())))
                elif name in ('turn_by','face'):
                    planned_start=float(step['start_yaw_rad'])
                    if abs(angle_diff(start_yaw,planned_start))>math.radians(5):
                        raise RuntimeError('当前朝向与预览不同，请重新生成预览')
                    turn_sign=1 if float(step['turn_delta_rad'])>=0 else -1
                    if name=='turn_by':
                        goal=math.radians(step['arguments']['angle_deg'])
                        turn_target=math.atan2(math.sin(start_yaw+turn_sign*goal),math.cos(start_yaw+turn_sign*goal))
                    else:
                        turn_target=float(step['target_yaw_rad'])
                        goal=(turn_target-start_yaw)%(2*math.pi) if turn_sign>0 else (start_yaw-turn_target)%(2*math.pi)
                        if abs(angle_diff(turn_target,start_yaw))<=TURN_TOLERANCE:goal=0.
                        elif goal>math.pi+math.radians(5):raise RuntimeError('朝向变化使预览转向失效，请重新预览')
                    self.rotate(session,start_yaw,goal,turn_sign,start)
                else:
                    path=np.asarray(step['route_xy_m']);index=1;deadline=time.monotonic()+min(180.,30+np.linalg.norm(np.diff(path,axis=0),axis=1).sum()/.04)
                    self.dynamic_obstacles=DynamicObstacleLayer(self.w.planner,session) if hasattr(getattr(self.w,'planner',None),'occupied') else None
                    last_progress=time.monotonic();anchor=start.copy()
                    aligned=False;walk_started_at=None
                    while True:
                        try:
                            if time.monotonic()>deadline:raise RuntimeError('导航超时，已停止')
                            tick=time.monotonic();_,pose=self.w.snapshot();current=pose[:2,3]
                            if np.linalg.norm(current-path[-1])<.12:break
                            a=path[:-1];d=path[1:]-a
                            u=np.clip(np.sum((current-a)*d,axis=1)/(np.sum(d*d,axis=1)+1e-12),0,1)
                            deviation=float(np.min(np.linalg.norm(a+u[:,None]*d-current,axis=1)))
                            if deviation>.18:
                                if deviation>ROUTE_CORRECTION_DISTANCE:
                                    raise RuntimeError('当前位置明显偏离已预览路线，已停止，请重新预览')
                                path=self.replan(session,path[-1].copy(),deadline,RoutePoseCorrection('当前位置小幅偏离路线，更新导航起点',pose))
                                index=1;aligned=False;walk_started_at=None
                                last_progress=time.monotonic();_,stopped=self.w.snapshot();anchor=stopped[:2,3].copy()
                                continue
                            while index<len(path)-1 and np.linalg.norm(current-path[index])<.12:
                                index+=1
                            delta=path[index]-current;velocity=delta/max(np.linalg.norm(delta),1e-9)*speed
                            self.navigation_remaining_m=float(np.linalg.norm(delta))
                            desired=math.atan2(delta[1],delta[0])
                            error=angle_diff(desired,pose_yaw(pose))
                            if abs(error)>(TRAVEL_REALIGN if aligned else TRAVEL_ALIGN):
                                self.client.zero()
                                self.last_navigation={'phase':'aligning','heading_error_deg':math.degrees(error),
                                                      'remaining_m':float(np.linalg.norm(current-path[-1]))}
                                try:self.face_travel(session,desired,route=np.linalg.norm(current-start)>.12,continuous=True,target_xy=path[index].copy())
                                except (ObstacleBlocked,RoutePoseCorrection) as exc:
                                    path=self.replan(session,path[-1].copy(),deadline,exc);index=1
                                    aligned=False;walk_started_at=None
                                    last_progress=time.monotonic();_,stopped=self.w.snapshot();anchor=stopped[:2,3].copy()
                                    continue
                                aligned=True
                                last_progress=time.monotonic();_,after_turn=self.w.snapshot();anchor=after_turn[:2,3].copy()
                                a=path[:-1];d=path[1:]-a
                                u=np.clip(np.sum((anchor-a)*d,axis=1)/(np.sum(d*d,axis=1)+1e-12),0,1)
                                if np.min(np.linalg.norm(a+u[:,None]*d-anchor,axis=1))>.14:
                                    path=self.replan(session,path[-1].copy(),deadline,ObstacleBlocked('转身后从实际停车点更新路线'))
                                    index=1;aligned=False
                                walk_started_at=None
                                continue
                            aligned=True
                            direction=pose_yaw(pose)
                            actual_velocity=np.array([math.cos(direction),math.sin(direction)])*speed
                            # Establish forward walking before adding steering. Keep
                            # refreshing vx through this bounded start interval.
                            starting=walk_started_at is None or tick-walk_started_at<WALK_START_SECONDS
                            yaw_rate=0. if starting or abs(error)<math.radians(1.5) else float(np.clip(1.2*error,-TRAVEL_YAW_RATE,TRAVEL_YAW_RATE))
                            try:pose=self.guard(session,actual_velocity,yaw_rate=yaw_rate)
                            except ObstacleBlocked as exc:
                                path=self.replan(session,path[-1].copy(),deadline,exc);index=1
                                aligned=False;walk_started_at=None
                                last_progress=time.monotonic();_,stopped=self.w.snapshot();anchor=stopped[:2,3].copy()
                                continue
                            if abs(angle_diff(desired,pose_yaw(pose)))>TRAVEL_REALIGN:
                                self.client.zero();aligned=False;continue
                            self.client.velocity(speed,0.,yaw_rate,self.w.cancel)
                            if walk_started_at is None:walk_started_at=time.monotonic()
                            self.forward_commands+=1
                            self.last_navigation={'phase':'starting_forward' if starting else 'walking',
                                                  'heading_error_deg':math.degrees(error),'yaw_rate_rad_s':yaw_rate,
                                                  'remaining_m':float(np.linalg.norm(current-path[-1]))}
                            self.commands_sent+=1
                            # Continue through minor corrections and waypoint changes.
                            # Stop once on reaching the goal, then verify the settled pose.
                            along_route=(path[index]-path[index-1])/max(1e-9,np.linalg.norm(path[index]-path[index-1]))
                            if (current-anchor)@along_route>.03:anchor=current.copy();last_progress=time.monotonic()
                            if time.monotonic()-last_progress>5:raise RuntimeError('前进指令已持续发送，但五秒内未观测到沿路线的有效前进，已停止')
                            self.w.cancel.wait(max(0,.1-(time.monotonic()-tick)))
                        except LocalizationUnavailable as exc:
                            reference=getattr(self,'last_guard_pose',None)
                            if reference is None:reference=pose
                            path=self.recover_navigation_localization(session,path[-1].copy(),reference,deadline,exc)
                            index=1;aligned=False;walk_started_at=None
                            last_progress=time.monotonic();anchor=path[0].copy()
                            continue
                self.client.zero();command_finished=time.time()
                pose=self.observe_after_stop(session)
                observation_valid=pose is not None
                distance=float(np.linalg.norm(pose[:2,3]-start)) if observation_valid else None
                rotation=float(math.degrees(angle_diff(pose_yaw(pose),start_yaw))) if observation_valid else None
                turning=name in ('turn_by','face')
                rotation_observed=bool(turning and observation_valid and abs(rotation)>2)
                navigation=name not in ('move_for','turn_by','face')
                goal_error=float(np.linalg.norm(pose[:2,3]-np.asarray(step['route_xy_m'])[-1])) if navigation and observation_valid else None
                departure_clearance=self.w.planner.clearance_at(pose[:2,3]) if navigation and observation_valid and hasattr(self.w,'planner') else None
                departure_ready=departure_clearance is None or departure_clearance>TURN_CLEARANCE
                target_reached=bool(observation_valid and (abs(angle_diff(turn_target,pose_yaw(pose)))<=TURN_TOLERANCE if turning else goal_error<=.12 and departure_ready if navigation else False))
                records.append({'skill':name,'started_unix_s':started,'finished_unix_s':time.time(),
                    'command_finished_unix_s':command_finished,'post_stop_observation_s':1.5,
                    'start_xy_m':start.tolist(),'end_xy_m':pose[:2,3].tolist() if observation_valid else None,'command_completed':True,
                    'observed_displacement_m':distance if observation_valid else None,
                    'observed_rotation_deg':rotation,'rotation_observed':rotation_observed,'target_reached':target_reached if turning or navigation else None,
                    'goal_error_m':goal_error,
                    'departure_clearance_m':departure_clearance,'departure_ready':departure_ready if navigation else None,
                    'obstacle_replans':list(self.replans) if navigation else [],
                    'motion_observed':observation_valid and (rotation_observed if turning else distance>.02),
                    'observation_valid':observation_valid,'speed_m_s':speed,'turn_rate_rad_s':TURN_RATE if turning else None,'speed_probe':probe})
            return records
        finally:
            active_error=sys.exc_info()[0] is not None
            try:self.client.zero()
            except Exception as exc:
                self.stop_error=str(exc)
                if not active_error:raise

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('health','stop','enable'));args=parser.parse_args()
    client=Teleop()
    if args.action=='stop':STOPFILE.touch()
    print(json.dumps(client.health() if args.action=='health' else client.zero() if args.action=='stop' else client.enable(),ensure_ascii=False))
