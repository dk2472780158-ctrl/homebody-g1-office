"""Real-office task preparation. This module has no robot command transport."""
import html,json,math,re,threading,time,uuid
from pathlib import Path
import numpy as np
from g1_readonly_geometry import StaticRoute
from g1_pointcloud_slice import obstacle_slice,scan_projection
from office_live_geometry import HistoricalMatcher,voxel
from office_live_panel import display_state
from office_landmarks import LANDMARK_PLACES,load_landmarks,visible_marks,approach_points
from g1_real_control import Executor,SPEED,STOPFILE,require_ready,task_speed,PROBE_SPEED,PROBE_DURATION,TURN_RATE,TURN_CLEARANCE,ROUTE_TURN_CLEARANCE,TRAVEL_ALIGN,pose_yaw,angle_diff,LocalizationUnavailable

PLACES={'go_north':'北','go_northeast':'东北','go_east':'东','go_southeast':'东南',
    'go_south':'南','go_southwest':'西南','go_west':'西','go_northwest':'西北','go_door':'门口','go_center':'房间中央',**LANDMARK_PLACES}
COMPASS={'east':(1,0),'northeast':(1,1),'north':(0,1),'northwest':(-1,1),
    'west':(-1,0),'southwest':(-1,-1),'south':(0,-1),'southeast':(1,-1)}
ALIASES={'东':'east','东北':'northeast','北':'north','西北':'northwest','西':'west',
    '西南':'southwest','南':'south','东南':'southeast','前':'forward','后':'backward','左':'left','右':'right'}
SKILLS=(*PLACES,'navigate','move_for','turn_by','face','wave','handshake')
PROMPT='''Translate the operator instruction into an ordered plan, never claim execution.
Return skill="plan", arguments_json encodes {"steps":[{"skill":...,"arguments":{...}}]}.
At most 12 steps. Allowed skills:
go_north 北, go_northeast 东北, go_east 东, go_southeast 东南, go_south 南,
go_southwest 西南, go_west 西, go_northwest 西北, go_door 门口: arguments {}.
go_center 房间中央/办公室中心/正中间: arguments {}. The centre of the main rectangular office room, not the doorway.
go_table1 桌子1, go_table2 桌子2, go_shelf 架子, go_window 窗口: arguments {}. Navigate to free approach points beside objects.
move_for: {"direction":"north|northeast|east|southeast|south|southwest|west|northwest|forward|backward|left|right", "duration_s":0.1..5}.
turn_by: {"direction":"left|right", "angle_deg":5..180}; rotate in place by this angle.
face: {"direction":"north|northeast|east|southeast|south|southwest|west|northwest"}; rotate in place to this office-map heading.
wave 挥手/招手 and handshake 握手: arguments {}. Official gestures performed standing still, then release arms.
Compass is office map north +y, east +x; forward/backward/left/right are robot relative.
Left/right in move_for mean sideways translation, not rotation. Use turn_by for 转/旋转/转身.
Compass and backward move_for first face the travel direction, then walk forward; named-place navigation turns at route bends.
Do not invent coordinates or substitute a named place for timed movement. Do not add unrequested actions.
Unsupported or ambiguous instruction: zero steps and explain in text. Output Chinese text.
Operator instruction:
'''

def open_office_server(host,port,title,mode):
    import viser
    from homebody.ui import theme
    server=viser.ViserServer(host=host,port=port)
    server.gui.configure_theme(titlebar_content=None,control_width='large',dark_mode=False,
        brand_color=theme.BRAND,show_logo=False,show_share_button=False)
    server.gui.main_panel.dock_right();server.gui.main_panel.set_width(theme.PANEL_WIDTH)
    server.gui.add_html(theme.STYLE)
    server.office_header=server.gui.add_html(theme.header(title,mode='真机任务预览'))
    return server

def clean_task(text):return html.unescape(re.sub(r'<[^>]*>',' ',text)).strip()

def validate_steps(steps):
    if not isinstance(steps,list) or not 1<=len(steps)<=12:raise ValueError('未得到可支持的任务步骤')
    for step in steps:
        if not isinstance(step,dict) or set(step)!={'skill','arguments'}:raise ValueError('技能计划格式无效')
        name,args=step['skill'],step['arguments']
        if name not in SKILLS or not isinstance(args,dict):raise ValueError('不支持的技能')
        if name=='move_for':
            if set(args)!={'direction','duration_s'} or args['direction'] not in (*COMPASS,'forward','backward','left','right'):
                raise ValueError('方向或参数无效')
            n=args['duration_s']
            if type(n) not in (int,float) or not math.isfinite(n) or not .1<=n<=5:raise ValueError('时长必须为 0.1–5 秒')
        elif name=='turn_by':
            if set(args)!={'direction','angle_deg'} or args['direction'] not in ('left','right'):
                raise ValueError('转身方向或参数无效')
            n=args['angle_deg']
            if type(n) not in (int,float) or not math.isfinite(n) or not 5<=n<=180:
                raise ValueError('转身角度必须为 5–180 度')
        elif name=='face':
            if set(args)!={'direction'} or args['direction'] not in COMPASS:
                raise ValueError('目标朝向无效')
        elif name=='navigate':
            xy=args.get('goal_xy_m')
            if set(args)!={'goal_xy_m'} or not isinstance(xy,list) or len(xy)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in xy):
                raise ValueError('目标坐标无效')
        elif args:raise ValueError('命名地点和手势不接受额外参数')
    return steps

def direct_steps(task):
    text=clean_task(task)
    called=re.fullmatch(r'请调用 (\w+) 一次，完成后调用 done。不要调用其他技能。',text)
    if called and called[1] in SKILLS:return validate_steps([{'skill':called[1],'arguments':{}}])
    text=text.replace('桌子一','桌子1').replace('桌子二','桌子2')
    if re.fullmatch(r'(?:请)?(?:去|到|走到|前往|移动到)\s*(?:(?:房间|办公室|屋子|屋内)(?:的)?)?(?:正中央|中央|中心|正中间|中间|正中)(?:位置|处|那里)?[。！! ]*',text):
        return [{'skill':'go_center','arguments':{}}]
    m=re.fullmatch(r'(?:请)?(?:去|到|走到|前往)(东北|东南|西北|西南|北|东|南|西|门口|桌子1|桌子2|架子|窗口)(?:点|旁边|边|那里)?[。！! ]*',text)
    if m:return [{'skill':next(k for k,v in PLACES.items() if v==m[1]),'arguments':{}}]
    if re.fullmatch(r'(?:请)?(?:挥手|挥挥手|挥个手|招手|招招手|招个手)[。！! ]*',text):return [{'skill':'wave','arguments':{}}]
    if re.fullmatch(r'(?:请)?(?:握手|握个手)[。！! ]*',text):return [{'skill':'handshake','arguments':{}}]
    if re.fullmatch(r'(?:请)?(?:原地)?转身[。！! ]*',text):raise ValueError('请说明左转或右转，以及转动角度')
    m=re.fullmatch(r'(?:请)?(?:原地)?(?:向)?(左|右)(?:转|旋转)(\d{1,3}(?:\.\d+)?|四十五|九十|一百八十)度[。！! ]*',text)
    if m:
        angle={'四十五':45,'九十':90,'一百八十':180}.get(m[2])
        return validate_steps([{'skill':'turn_by','arguments':{'direction':'left' if m[1]=='左' else 'right','angle_deg':float(m[2]) if angle is None else angle}}])
    m=re.fullmatch(r'(?:请)?(?:原地)?(?:向)?(左|右)转身[。！! ]*',text)
    if m:return validate_steps([{'skill':'turn_by','arguments':{'direction':'left' if m[1]=='左' else 'right','angle_deg':180}}])
    m=re.fullmatch(r'(?:请)?(?:原地)?(?:转向|朝向|面向)(东北|东南|西北|西南|北|东|南|西)(?:方)?[。！! ]*',text)
    if m:return validate_steps([{'skill':'face','arguments':{'direction':ALIASES[m[1]]}}])
    m=re.fullmatch(r'(?:请)?(?:原地)?(?:朝|向)(东北|东南|西北|西南|北|东|南|西)(?:方)?转[。！! ]*',text)
    if m:return validate_steps([{'skill':'face','arguments':{'direction':ALIASES[m[1]]}}])
    m=re.fullmatch(r'(?:请)?(?:按照现在的速度)?(?:向|往|朝)(东北|东南|西北|西南|北|东|南|西|前|后|左|右)(?:方)?(?:走|移动|横移)(\d+(?:\.\d+)?|一|二|两|三|四|五)秒[。！! ]*',text)
    if m:
        n={'一':1,'二':2,'两':2,'三':3,'四':4,'五':5}.get(m[2])
        return validate_steps([{'skill':'move_for','arguments':{'direction':ALIASES[m[1]],'duration_s':float(m[2]) if n is None else n}}])
    return None

class RealWorkflow:
    def __init__(self,ui,artifacts,provider,speed):
        self.ui=ui;self.root=artifacts;self.live=artifacts/'sensors/runtime/live-readonly'
        self.provider=provider;self.speed=SPEED;self.stop=threading.Event();self.cancel=threading.Event()
        self.lock=threading.Lock();self.worker=None;self.path=None;self.click_at=None
        self.pending=None;self.executing=False;self.executor=Executor(self)
        self.control_reason='等待控制状态';self.control_health_at=0.
        saved=json.loads((artifacts/'office-waypoints.json').read_text())['points']
        self.points=approach_points(artifacts,saved)
        self.planner=StaticRoute(artifacts/'real-office-navigation.npz' if (artifacts/'real-office-navigation.npz').exists() else artifacts/'office-map-proxy-scene/map.npz')
        history=HistoricalMatcher(artifacts/'g1_fastlio.historical-office.pcd')
        registration=np.array(json.loads((artifacts/'historical-office-registration.json').read_text())['T_office_map_from_provided_pcd'])
        self.floor=float((registration@[0,0,history.floor['height_m'],1])[2])
        target=history.target@registration[:3,:3].T+registration[:3,3]
        slice_file=artifacts/'office-cloud-slice/cloud-slice.npz'
        if slice_file.exists():
            with np.load(slice_file,allow_pickle=False) as data:target=data['xyz'].copy()
        server=ui.server
        self.sim_root=server.scene.add_frame('/world',show_axes=False)
        self.guides_root=server.scene.add_frame('/guides',show_axes=False)
        self.real_root=server.scene.add_frame('/real-office',show_axes=False)
        server.scene.add_point_cloud('/real-office/history',points=target.astype(np.float32),colors=(139,150,159),point_size=.025)
        self.cloud=server.scene.add_point_cloud('/real-office/current',points=np.zeros((1,3),np.float32),colors=(248,145,44),point_size=.04,visible=False)
        self.base=server.scene.add_frame('/real-office/base',axes_length=.45,axes_radius=.015,visible=False)
        self.route=server.scene.add_line_segments('/real-office/route',points=np.zeros((1,2,3),np.float32),colors=(55,145,240),line_width=5,visible=False)
        self.goal=server.scene.add_icosphere('/real-office/goal',radius=.09,color=(242,175,35),visible=False)
        self.turn_preview=server.scene.add_line_segments('/real-office/turn-preview',points=np.zeros((1,2,3),np.float32),colors=(244,124,45),line_width=6,visible=False)
        self.turn_goal=server.scene.add_icosphere('/real-office/turn-heading',radius=.08,color=(244,124,45),visible=False)
        for label,xy in visible_marks(artifacts,saved).items():
            server.scene.add_icosphere('/real-office/points/'+label,radius=.07,color=(49,154,115),position=(*xy,self.floor+.05))
            server.scene.add_label('/real-office/labels/'+label,label,position=(*xy,self.floor+.2))
        with server.gui.add_folder('运行模式',expand_by_default=True):
            self.mode=server.gui.add_dropdown('任务目标',options=('真机任务预览','MuJoCo 仿真'),initial_value='真机任务预览')
            self.status=server.gui.add_markdown('真机模式使用现场定位；提交任务后生成技能计划和路线，保持机器人现有姿态。')
            self.cloud_view=server.gui.add_dropdown('实时点云显示',options=('障碍高度切片','二维障碍投影','原始点云'),initial_value='障碍高度切片')
            server.gui.add_markdown('障碍切片：离地 0.15–1.80 m，过滤机身近点；用于实时避障和遇障重规划。')
            self.result=server.gui.add_markdown(f'真机行走速度 {SPEED:.2f} m/s；原地转身角速度 {TURN_RATE:.2f} rad/s。先生成预览，再执行；使能腿控制可能触发自动站立。')
            self.enable_button=server.gui.add_button('使能腿控制（可能触发站立）')
            self.execute_button=server.gui.add_button('执行已预览任务 · 真机',disabled=True)
            self.execute_reason=server.gui.add_markdown('执行不可用：请先生成任务预览。')
            self.probe_button=server.gui.add_button(f'向前短时验证 · {PROBE_SPEED:.2f} m/s × {PROBE_DURATION:g} 秒（只预览）')
            self.probe_button.on_click(lambda _:self.submit(f'向前短时验证：向机身前方移动 {PROBE_DURATION:g} 秒',
                steps=[{'skill':'move_for','arguments':{'direction':'forward','duration_s':PROBE_DURATION}}],probe=True))
            halt=server.gui.add_button('停止真机移动 · 零速度',color='red')
            self.enable_button.on_click(lambda _:self.enable_control())
            self.execute_button.on_click(lambda _:self.execute_pending())
            halt.on_click(lambda _:self.request_stop())
            @ui.task.on_update
            def instruction_changed(_):
                if self.executing:return
                self.pending=None;self.execute_button.disabled=True
                self.route.visible=self.goal.visible=self.turn_preview.visible=self.turn_goal.visible=False;self.path=None
                self.ui.thought.content='';self.result.content='指令已改变，请重新生成预览。'
                self.execute_reason.content='指令已改变，请重新生成预览。'
            pick=server.gui.add_button('真机地图选点')
            top=server.gui.add_button('真机地图俯视')
            self.mode.on_update(lambda _:self.change_mode())
            top.on_click(lambda _:self.top_view())
            @pick.on_click
            def pick_once(_):
                if not self.real_mode:return
                self.top_view();self.click_at=time.monotonic();self.result.content='点击右侧办公室空地，预览从当前真机位置出发的路线。'
        @server.scene.on_click()
        def map_click(event):
            if not self.real_mode or self.click_at is None:return
            elapsed=time.monotonic()-self.click_at
            if elapsed<.75:return
            self.click_at=None
            if elapsed>30:return
            origin=np.asarray(event.ray_origin);direction=np.asarray(event.ray_direction)
            if abs(direction[2])<1e-6:return
            length=(self.floor-origin[2])/direction[2]
            if length<=0:return
            xy=(origin+length*direction)[:2]
            self.submit('地图选点',steps=[{'skill':'navigate','arguments':{'goal_xy_m':xy.tolist()}}])
        @server.on_client_connect
        def connected(client):
            if self.real_mode:self.top_view()
        @server.on_client_disconnect
        def disconnected(client):
            if self.executing and not server.get_clients():self.request_stop()
        self.thread=threading.Thread(target=self.refresh,daemon=True,name='real-office-scene');self.thread.start()
        self.change_mode()
    @property
    def real_mode(self):return self.mode.value=='真机任务预览'
    def top_view(self):
        for client in self.ui.server.get_clients().values():
            client.camera.up_direction=(0.,1.,0.);client.camera.position=(-.7,-1.4,self.floor+16);client.camera.look_at=(-.7,-1.4,self.floor)
    def change_mode(self):
        self.cancel.set();self.click_at=None;real=self.real_mode
        self.pending=None;self.execute_button.disabled=True
        self.sim_root.visible=self.guides_root.visible=not real;self.real_root.visible=real
        self.probe_button.visible=real
        self.ui.ego.visible=not real
        self.ui.submit_button.label='生成真机任务预览' if real else '执行仿真任务'
        self.ui.task.label='自然语言指令（例：去桌子1再招手；去窗口；原地左转90度；往北走两秒）'
        self.ui.stop_button.label='停止真机移动' if real else '停止仿真任务'
        from homebody.ui.theme import header
        self.ui.server.office_header.content=header('办公室九点导航',mode='真机任务预览' if real else 'MuJoCo 仿真')
        if real:self.top_view()
        else:self.ui.scene_view.reset_camera()
    def snapshot(self):
        state=json.loads((self.live/'live-state.json').read_text());view=json.loads((self.live/'real-view-state.json').read_text())
        session=(self.live/'session-id').read_text().strip();now=time.time()
        if not display_state(state,now,session):
            message='地图定位暂不可用：'+str(state.get('message','等待数据'))
            if state.get('status')=='matching' and state.get('localization_backend')=='nav2_amcl':
                raise LocalizationUnavailable(message,state)
            raise ValueError(message)
        if view.get('session_id')!=session:raise ValueError('底座定位会话正在同步')
        if not 0<=now-view['updated_unix_s']<2:raise ValueError('底座位置更新已中断')
        if not view.get('base'):raise ValueError('底座数据暂不可用：'+str(view.get('base_unavailable_reason') or '等待关节与 IMU 数据'))
        pose=np.array(view['base']['T_office_from_pelvis'],float)
        if pose.shape!=(4,4) or not np.isfinite(pose).all():raise ValueError('底座坐标无效')
        return state,pose
    def refresh(self):
        from scipy.spatial.transform import Rotation
        while not self.stop.wait(.7):
            if not self.real_mode:continue
            try:
                state,pose=self.snapshot()
                self.base.position=tuple(pose[:3,3]);q=Rotation.from_matrix(pose[:3,:3]).as_quat();self.base.wxyz=tuple(q[[3,0,1,2]]);self.base.visible=True
                with np.load(self.live/'clouds.npz',allow_pickle=False) as data:
                    if str(data['session_id'])!=state['session_id'] or time.time()-float(data['received_unix_s'])>3:raise ValueError('点云等待刷新')
                    xyz=data['latest_xyz'].copy()
                t=np.array(state['T_office_from_current']);xyz=xyz@t[:3,:3].T+t[:3,3]
                if self.cloud_view.value!='原始点云':
                    xyz=obstacle_slice(xyz,self.floor,pose[:2,3])
                    if self.cloud_view.value=='二维障碍投影':xyz,_=scan_projection(xyz,pose[:2,3],self.floor)
                self.cloud.points=voxel(xyz,.05).astype(np.float32);self.cloud.visible=True
                clearance=self.planner.clearance_at(pose[:2,3])
                if time.monotonic()-self.control_health_at>2:
                    try:
                        require_ready(self.executor.client.health());self.control_reason=''
                    except RuntimeError as exc:self.control_reason=str(exc)
                    self.control_health_at=time.monotonic()
                self.status.content=f'**真机位置在线** · x={pose[0,3]:.2f}，y={pose[1,3]:.2f} m\n\n真机行走 {SPEED:.2f} m/s；转身 {TURN_RATE:.2f} rad/s；静态净空 {clearance:.2f} m。'+('控制不可用：'+self.control_reason if self.control_reason else '正在执行。' if self.executing else '已接通现有遥操作控制，等待操作。')
                if not self.executing and self.path is not None and (self.path['session']!=state['session_id'] or np.linalg.norm(pose[:2,3]-self.path['start'])>.1 or abs(angle_diff(pose_yaw(pose),self.path['yaw']))>math.radians(5) or clearance<=self.planner.radius):
                    self.route.visible=self.goal.visible=self.turn_preview.visible=self.turn_goal.visible=False;self.path=None
                    self.pending=None;self.execute_button.disabled=True
                self.refresh_execute_state(state,pose)
            except (OSError,ValueError,KeyError,TypeError) as exc:
                self.base.visible=self.cloud.visible=self.route.visible=self.goal.visible=self.turn_preview.visible=self.turn_goal.visible=False;self.path=None
                self.status.content='**等待真机定位** · '+str(exc)
                self.execute_button.disabled=True
                self.execute_reason.content='执行暂不可用：'+str(exc)+'；数据恢复后自动恢复有效预览。'
    def refresh_execute_state(self,state,pose):
        if self.executing:reason='任务正在执行。'
        elif self.control_reason:reason=self.control_reason
        elif self.pending is None:reason='请先生成新的任务预览；每份预览只执行一次。'
        elif time.monotonic()-self.pending['at']>30:reason='预览超过 30 秒，请重新生成。'
        elif state['session_id']!=self.pending['session']:reason='定位会话已变化，请重新生成预览。'
        elif np.linalg.norm(pose[:2,3]-self.pending['pose'])>.1:reason='位置已变化，请重新生成预览。'
        elif self.pending.get('yaw') is not None and abs(angle_diff(pose_yaw(pose),self.pending['yaw']))>math.radians(5):reason='朝向已变化，请重新生成预览。'
        elif not self.real_mode:reason='当前为仿真模式。'
        else:reason=''
        self.execute_button.disabled=bool(reason)
        self.execute_button.label=f'执行短时验证 · {PROBE_SPEED:.2f} m/s × {PROBE_DURATION:g} 秒' if self.pending and self.pending.get('probe') else '执行已预览任务 · 真机'
        self.execute_reason.content='执行不可用：'+reason if reason else '预览有效，可以点击执行；点击前会重新检查定位和障碍。'
    def submit(self,task,steps=None,*,probe=False):
        if not self.lock.acquire(blocking=False):return
        self.pending=None;self.execute_button.disabled=True
        self.ui.thought.content=''
        self.execute_reason.content='正在生成新任务预览。'
        self.cancel.clear();self.ui.submit_button.disabled=True;self.mode.disabled=True;self.probe_button.disabled=True
        def work():
            attempt=self.root/'real-task-plans'/uuid.uuid4().hex
            try:
                self.route.visible=self.goal.visible=self.turn_preview.visible=self.turn_goal.visible=False;self.path=None
                self.result.content='正在生成真机技能计划…'
                self.ui.status.content='正在生成真机任务预览…'
                chosen=steps or direct_steps(task)
                if chosen is None:
                    decision=self.provider.generate(PROMPT+clean_task(task),images=(),names=('plan',),attempt=attempt/'model',tick=lambda:time.sleep(.02),cancelled=self.cancel.is_set)
                    if set(decision.arguments)!={'steps'}:raise ValueError('模型未返回有效步骤列表')
                    chosen=decision.arguments['steps']
                chosen=validate_steps(chosen)
                if self.cancel.is_set():raise ValueError('任务预览已取消')
                speed=task_speed(chosen,probe);prepared=self.prepare(chosen,probe=probe)
                if self.cancel.is_set():raise ValueError('任务预览已取消')
                attempt.mkdir(parents=True,exist_ok=True)
                (attempt/'plan.json').write_text(json.dumps({'task':clean_task(task),'steps':prepared,'executed':False,'speed_m_s':speed,'turn_rate_rad_s':TURN_RATE,'speed_probe':probe},ensure_ascii=False,indent=2))
                self.result.content=(f'**向前短时验证 · 尚未执行**\n\n本次使用 {PROBE_SPEED:.2f} m/s、{PROBE_DURATION:g} 秒，指令距离 {PROBE_SPEED*PROBE_DURATION:.2f} m。\n\n' if probe else '**真机任务已编排 · 尚未执行**\n\n')+'\n\n'.join(f'{i+1}. {s["description"]}' for i,s in enumerate(prepared))
                blocked=any(s['status'] in ('blocked','unavailable') for s in prepared)
                if blocked:self.result.content=self.result.content.replace('**真机任务已编排 · 尚未执行**','**真机任务受阻 · 尚未执行**',1)
                if not blocked:
                    state,pose=self.snapshot()
                    self.pending={'steps':chosen,'session':state['session_id'],'pose':pose[:2,3].copy(),'yaw':pose_yaw(pose),
                        'at':time.monotonic(),'attempt':attempt,'probe':probe}
                    self.execute_button.disabled=False
                    self.refresh_execute_state(state,pose)
                self.ui.status.content='真机任务受阻 · 尚未执行' if blocked else '真机任务已编排 · 尚未执行'
                description='<br>'.join(html.escape(f'{i+1}. {s["description"]}') for i,s in enumerate(prepared))
                self.ui.thought.content=f'<div class="hb-bubble"><div class="hb-title">真机技能计划</div><div class="hb-message">{description}</div></div>'
            except Exception as exc:
                self.pending=None;self.execute_button.disabled=True;self.path=None
                self.route.visible=self.goal.visible=self.turn_preview.visible=self.turn_goal.visible=False
                self.ui.thought.content=''
                self.execute_reason.content='执行不可用：本次预览未完成，请修改指令或重新生成。'
                self.result.content='任务预览未完成：'+str(exc);self.ui.status.content='任务预览未完成'
            finally:
                self.ui.submit_button.disabled=False;self.mode.disabled=False;self.probe_button.disabled=False;self.lock.release()
        self.worker=threading.Thread(target=work,daemon=True,name='real-office-plan');self.worker.start()
    def prepare(self,steps,*,probe=False):
        speed=task_speed(steps,probe)
        state,pose=self.snapshot();start=pose[:2,3].copy();cursor=start.copy();segments=[];turn_segments=[];prepared=[]
        yaw=pose_yaw(pose);initial_yaw=yaw;blocked=False
        for step in steps:
            if self.cancel.is_set():raise ValueError('任务预览已取消')
            name,args=step['skill'],step['arguments'];item=dict(step,status='planned')
            if name in PLACES or name=='navigate':
                goal=np.array(self.points[PLACES[name]] if name in PLACES else args['goal_xy_m'])
                try:
                    xy,detail=self.planner.navigation_route(cursor,goal,yaw=yaw);goal=xy[-1].copy();route_yaw=yaw;markers=[]
                    for i,d in enumerate(np.diff(xy,axis=0)):
                        if np.linalg.norm(d)<.01:continue
                        desired=math.atan2(d[1],d[0])
                        if abs(angle_diff(desired,route_yaw))>TRAVEL_ALIGN:
                            limit=TURN_CLEARANCE if np.linalg.norm(xy[i]-start)<.12 else ROUTE_TURN_CLEARANCE
                            if self.planner.clearance_at(xy[i])<=max(self.planner.radius+.1,limit):
                                raise ValueError(f'路线转身点净空 {self.planner.clearance_at(xy[i]):.2f} m，不足 {max(self.planner.radius+.1,limit):.2f} m')
                            tip=xy[i]+.65*np.array([math.cos(desired),math.sin(desired)])
                            markers.append(np.stack([xy[i],tip]))
                        route_yaw=desired
                    segments.append(xy);turn_segments.extend(markers);cursor=goal;yaw=route_yaw
                    item['description']=f'前往{PLACES.get(name,"点击位置")}：先朝路线方向转身，沿路线逐段正面行走 {detail["length_m"]:.2f} m，计划速度 {speed:.2f} m/s'
                    if detail.get('forward_exit_m',0):
                        item['description']=f'前往{PLACES.get(name,"点击位置")}：先沿当前朝向直行 {detail["forward_exit_m"]:.2f} m 离开狭窄处，再转向并正面行走；全程 {detail["length_m"]:.2f} m，计划速度 {speed:.2f} m/s'
                    if detail.get('goal_adjustment_m',0)>.01:
                        item['description']+=f'；目标邻近障碍，停车点调整 {detail["goal_adjustment_m"]:.2f} m，为下次转向留空间'
                    item['requested_goal_xy_m']=detail.get('requested_goal_xy_m',goal.tolist())
                    item['stop_goal_xy_m']=goal.tolist()
                    item['route_xy_m']=xy.tolist()
                except ValueError as exc:
                    item.update(status='blocked',description=f'前往{PLACES.get(name,"点击位置")}：{exc}');blocked=True
            elif name=='move_for':
                direction=args['direction'];v=np.array(COMPASS.get(direction,(0,0)),float)
                forward_travel=direction in COMPASS or direction=='backward'
                if direction in ('forward','backward','left','right'):
                    angle=yaw+{'forward':0,'backward':math.pi,'left':math.pi/2,'right':-math.pi/2}[direction];v=np.array([math.cos(angle),math.sin(angle)])
                if forward_travel:
                    target_yaw=math.atan2(v[1],v[0]);turn_delta=angle_diff(target_yaw,yaw)
                v/=np.linalg.norm(v);goal=cursor+v*speed*args['duration_s']
                xy=np.linspace(cursor,goal,max(2,int(np.linalg.norm(goal-cursor)/.025)+2))
                try:
                    if forward_travel and abs(turn_delta)>TRAVEL_ALIGN and self.planner.clearance_at(cursor)<=max(self.planner.radius+.1,TURN_CLEARANCE):
                        raise ValueError(f'起点转身净空 {self.planner.clearance_at(cursor):.2f} m，不足 {max(self.planner.radius+.1,TURN_CLEARANCE):.2f} m')
                    if any(self.planner.clearance_at(p)<=self.planner.radius for p in xy):raise ValueError('直线路径净空不足')
                    segments.append(xy);cursor=goal
                    if forward_travel:
                        item['face_yaw_rad']=target_yaw
                        label='后方' if direction=='backward' else next(k for k,v in ALIASES.items() if v==direction)
                        if abs(turn_delta)>TRAVEL_ALIGN:
                            tip=xy[0]+.65*np.array([math.cos(target_yaw),math.sin(target_yaw)])
                            turn_segments.append(np.stack([xy[0].copy(),tip]));yaw=target_yaw
                        item['description']=f'先转向{label}再正面移动 {args["duration_s"]:g} 秒，计划速度 {speed:.2f} m/s'
                    else:item['description']=f'{direction} 移动 {args["duration_s"]:g} 秒，计划速度 {speed:.2f} m/s'
                    item['route_xy_m']=xy.tolist()
                except ValueError as exc:
                    label=next((k for k,v in ALIASES.items() if v==direction),direction)
                    item.update(status='blocked',description=f'向{label}移动 {args["duration_s"]:g} 秒：{exc}');blocked=True
            elif name in ('turn_by','face'):
                if self.planner.clearance_at(cursor)<=max(self.planner.radius+.1,TURN_CLEARANCE):
                    item.update(status='blocked',description=f'原地转身净空 {self.planner.clearance_at(cursor):.2f} m，不足 {max(self.planner.radius+.1,TURN_CLEARANCE):.2f} m');blocked=True
                else:
                    before=yaw
                    if name=='turn_by':
                        delta=math.radians(args['angle_deg'])*(1 if args['direction']=='left' else -1)
                        yaw=math.atan2(math.sin(before+delta),math.cos(before+delta))
                        label='左转' if delta>0 else '右转'
                        item['description']=f'原地{label} {args["angle_deg"]:g}°，角速度上限 {TURN_RATE:.2f} rad/s'
                    else:
                        direction=args['direction'];v=COMPASS[direction]
                        yaw=math.atan2(v[1],v[0]);delta=angle_diff(yaw,before)
                        label=next(k for k,v in ALIASES.items() if v==direction)
                        item['description']=f'原地转向{label}，预计转 {abs(math.degrees(delta)):.1f}°，角速度上限 {TURN_RATE:.2f} rad/s'
                    item.update(start_yaw_rad=before,target_yaw_rad=yaw,turn_delta_rad=delta)
                    tip=cursor+.65*np.array([math.cos(yaw),math.sin(yaw)])
                    turn_segments.append(np.stack([cursor.copy(),tip]))
            elif name in ('wave','handshake'):
                try:
                    capabilities=self.executor.client.gesture_capabilities()
                    if name not in capabilities.get('available',[]):raise ValueError('机器人尚未提供此手势')
                    if self.planner.clearance_at(cursor)<=TURN_CLEARANCE:raise ValueError('手臂动作周围净空不足')
                    item['description']=('招手' if name=='wave' else '握手')+'：停止底座后执行官方动作，结束释放手臂；可点击停止中断'
                except (ValueError,RuntimeError) as exc:
                    item.update(status='unavailable',description=('招手' if name=='wave' else '握手')+'：'+str(exc));blocked=True
            prepared.append(item)
            if blocked:
                for pending in steps[len(prepared):]:prepared.append(dict(pending,status='pending',description='前序步骤未就绪，后续待处理'))
                break
        latest,current=self.snapshot()
        if latest['session_id']!=state['session_id'] or np.linalg.norm(current[:2,3]-start)>.1 or abs(angle_diff(pose_yaw(current),initial_yaw))>math.radians(5):
            raise ValueError('规划期间定位已改变，请重新生成预览')
        if segments:
            lines=np.concatenate([np.stack([xy[:-1],xy[1:]],axis=1) for xy in segments])
            self.route.points=np.concatenate([lines,np.full((*lines.shape[:2],1),self.floor+.06)],axis=2).astype(np.float32)
            self.route.visible=not self.cancel.is_set() and self.real_mode
            self.goal.position=(*cursor,self.floor+.07);self.goal.visible=self.route.visible
        if turn_segments:
            lines=np.asarray(turn_segments)
            self.turn_preview.points=np.concatenate([lines,np.full((*lines.shape[:2],1),self.floor+.10)],axis=2).astype(np.float32)
            self.turn_preview.visible=not self.cancel.is_set() and self.real_mode
            self.turn_goal.position=(*turn_segments[-1][1],self.floor+.11);self.turn_goal.visible=self.turn_preview.visible
        if segments or turn_segments:self.path={'start':start,'yaw':initial_yaw,'session':state['session_id']}
        return prepared
    def request_stop(self):
        self.cancel.set();STOPFILE.touch();self.pending=None;self.execute_button.disabled=True
        def stop_motion():
            try:
                h=self.executor.client.health()
                if h.get('armed'):self.executor.client.zero()
                self.result.content='已停止移动；只发送零速度，没有下蹲或阻尼命令。'
            except Exception as exc:self.result.content='停止请求异常：'+str(exc)+'。请使用现场遥控器停止。'
        threading.Thread(target=stop_motion,daemon=True).start()
    def enable_control(self):
        if not self.lock.acquire(blocking=False):return
        self.enable_button.disabled=self.mode.disabled=True
        def enable():
            try:
                reply=self.executor.client.enable()
                self.result.content='腿控制已使能，机器人可能已自动站立。请生成短时任务预览，再点击执行。' if reply.get('armed') else '腿控制未使能。'
            except Exception as exc:self.result.content='腿控制使能失败：'+str(exc)
            finally:self.enable_button.disabled=self.mode.disabled=False;self.lock.release()
        threading.Thread(target=enable,daemon=True).start()
    def navigation_update(self,phase,message,path=None):
        if self.cancel.is_set() or not self.executing:return
        labels={'replanning':'导航暂停 · 正在重新规划','waiting':'导航暂停 · 等待通路',
                'resuming':'真机导航 · 按新路线继续','blocked':'导航暂停 · 无可行路线',
                'localizing':'导航暂停 · 等待定位稳定'}
        self.result.content=message;self.ui.status.content=labels[phase]
        if path is not None:
            xy=np.asarray(path,float);lines=np.stack([xy[:-1],xy[1:]],axis=1)
            self.route.points=np.concatenate([lines,np.full((*lines.shape[:2],1),self.floor+.06)],axis=2).astype(np.float32)
            self.route.visible=True
            self.goal.position=(*xy[-1],self.floor+.07);self.goal.visible=True
            self.turn_preview.visible=self.turn_goal.visible=False
    def execute_pending(self):
        if not self.lock.acquire(blocking=False):return
        pending=self.pending;self.pending=None;self.execute_button.disabled=True
        self.mode.disabled=self.enable_button.disabled=self.ui.submit_button.disabled=self.probe_button.disabled=True
        self.cancel.clear();self.executing=True
        def run():
            self.executor.commands_sent=0
            self.executor.last_turn=None
            self.executor.forward_commands=0;self.executor.last_navigation=None;self.executor.replans=[]
            self.executor.gestures_sent=0
            self.executor.gestures_started=0
            outcome={'executed':False,'started_unix_s':time.time(),'records':[]}
            try:
                if not self.real_mode:raise RuntimeError('当前为仿真模式，未执行真机任务')
                if pending is None or time.monotonic()-pending['at']>30:raise RuntimeError('预览已过期，请重新生成')
                probe=pending.get('probe',False);speed=task_speed(pending['steps'],probe)
                outcome.update(speed_m_s=speed,turn_rate_rad_s=TURN_RATE,speed_probe=probe)
                state,pose=self.snapshot()
                if state['session_id']!=pending['session'] or np.linalg.norm(pose[:2,3]-pending['pose'])>.1 or abs(angle_diff(pose_yaw(pose),pending['yaw']))>math.radians(5):
                    raise RuntimeError('当前位置或朝向已改变，请重新预览')
                prepared=self.prepare(pending['steps'],probe=probe)
                if any(s['status']!='planned' for s in prepared):raise RuntimeError('重新检查路线未通过，未执行')
                STOPFILE.unlink(missing_ok=True)
                self.result.content=f'**真机正在执行** · 行走 {speed:.2f} m/s；转身 {TURN_RATE:.2f} rad/s；“停止真机移动”可中断。'
                self.ui.status.content='真机正在执行'
                outcome['records']=self.executor.run(prepared,pending['session'],probe=probe)
                observed=any(r.get('motion_observed',False) for r in outcome['records'])
                completed=all(r.get('gesture_completed',False) if r['skill'] in ('wave','handshake') else
                    r['target_reached'] if r['skill']!='move_for' else r['motion_observed'] for r in outcome['records'])
                outcome.update(executed=True,command_sequence_completed=True,motion_observed=observed,completed=completed)
                if completed and any(r['skill'] in ('wave','handshake') for r in outcome['records']):
                    self.result.content='任务指令已结束；官方手势接口返回成功并释放手臂，实际手势请核对现场。'
                    self.ui.status.content='任务结束 · 手势接口已确认'
                elif completed and observed:
                    self.result.content='速度指令已结束并发送零速度，定位观测到预期动作；请核对现场结果。'
                    self.ui.status.content='速度指令已结束 · 定位观测到动作'
                elif completed:
                    self.result.content='当前位置已满足目标朝向，无需下发转身速度。'
                    self.ui.status.content='目标朝向已满足'
                elif any(r.get('departure_ready') is False for r in outcome['records']):
                    self.result.content='已停止；停车后的净空不足以再次转向，未判定导航完成。请重新预览附近更开阔的停车点。'
                    self.ui.status.content='停车后转向空间不足'
                else:
                    self.result.content='速度指令已结束并发送零速度，但未确认完成全部移动或转身；请核对现场结果。'
                    self.ui.status.content='未确认真机任务完成'
            except Exception as exc:
                outcome.update(completed=False,error=str(exc))
                self.result.content='真机任务已中止：'+str(exc);self.ui.status.content='真机任务已中止'
            finally:
                outcome['finished_unix_s']=time.time()
                outcome['nonzero_commands_accepted']=self.executor.commands_sent
                outcome['last_turn']=self.executor.last_turn
                outcome['turn_segments']=getattr(self.executor,'turn_segments',[])
                outcome['forward_commands_accepted']=self.executor.forward_commands
                outcome['last_navigation']=self.executor.last_navigation
                outcome['obstacle_replans']=self.executor.replans
                outcome['stop_error']=getattr(self.executor,'stop_error',None)
                transport=getattr(self.executor.client,'transport',None)
                if transport is not None:outcome['control_http']=transport.diagnostics()
                outcome['gesture_commands_completed']=self.executor.gestures_sent
                outcome['gesture_commands_started']=self.executor.gestures_started
                outcome['executed']=self.executor.commands_sent>0 or self.executor.gestures_started>0
                if pending is not None:(pending['attempt']/'execution.json').write_text(json.dumps(outcome,ensure_ascii=False,indent=2))
                self.executing=False;self.mode.disabled=self.enable_button.disabled=self.ui.submit_button.disabled=self.probe_button.disabled=False
                self.lock.release()
        threading.Thread(target=run,daemon=True,name='g1-real-execution').start()
    def close(self):
        if self.executing:self.request_stop()
        self.cancel.set();self.stop.set();self.thread.join(timeout=2)
