"""Map clicks drive HomeBody's checked navigation in the office MuJoCo scene."""
import hb_config
from dataclasses import replace
import argparse
import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
import time
import tkinter as tk

import mujoco
import mujoco.viewer
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageTk
from homebody.backends.mujoco import MujocoBackend
from homebody.session.settings import Settings
from homebody.skills.contract import Context, Runner
from homebody.skills.navigate_real import task as navigate_real
from homebody.skills.registry import task_registry

ROOT = Path(hb_config.expand('${HB_REPO_ROOT}'))
ARTIFACTS = Path(hb_config.expand('${HB_OFFICE_ROOT}'))
SCENE = ARTIFACTS / 'office-map-proxy-scene'


class MapPanel:
    def __init__(self, robot):
        self.robot = robot
        self.home = np.array(json.loads((SCENE/'task_scene.json').read_text())['robot_start'][:2])
        self.waypoint_file = ARTIFACTS/'office-waypoints.json'
        self.waypoint_names = ('北','东北','东','东南','南','西南','西','西北','门口')
        self.waypoints = {}
        if self.waypoint_file.is_file():
            saved = json.loads(self.waypoint_file.read_text())
            if (saved.get('map_shape') == list(robot.navigation.occupied.shape) and
                    np.array_equal(saved.get('T_map_px'),robot.navigation.T_map_px)):
                self.waypoints = {name: np.asarray(point,dtype=float)
                                  for name,point in saved.get('points',{}).items()
                                  if name in self.waypoint_names and np.asarray(point).shape == (2,)}
        self.assigning = None
        self.waypoint_buttons = {}
        self.pending = self.goal = self.result = None
        self.busy = self.cancelled = self.closed = False
        self.path, self.trace = [], []
        self.last_sample = -math.inf
        self.offset, self.scale = np.zeros(2), 1.
        self.inverse = np.linalg.inv(robot.navigation.T_map_px)
        self.ui = tk.Tk()
        self.ui.title('HomeBody | Click to navigate')
        self.ui.geometry('1160x900+1320+60')
        self.ui.configure(bg='#101c2d')
        self.ui.protocol('WM_DELETE_WINDOW', self.close)
        font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
        def rendered(text, color, bg, size=12, padding=(0,0)):
            face = ImageFont.truetype(font_path, round(size*1.35))
            box = face.getbbox(text)
            image = Image.new('RGB',(max(1,box[2]-box[0]+2*padding[0]),
                                     max(1,box[3]-box[1]+2*padding[1])),bg)
            ImageDraw.Draw(image).text((padding[0]-box[0],padding[1]-box[1]),
                                       text,font=face,fill=color)
            return ImageTk.PhotoImage(image)
        def label(text=None, variable=None, color='#b8c9dc', size=12):
            widget = tk.Label(self.ui, bg='#101c2d', anchor='w', bd=0)
            def update(*_):
                widget.photo = rendered(variable.get() if variable is not None else text,
                                        color,'#101c2d',size)
                widget.configure(image=widget.photo)
            if variable is not None:
                variable.trace_add('write',update)
            update()
            widget.pack(fill='x', padx=24, pady=4)
        label('HomeBody  /  办公室导航仿真', color='#f3f8ff', size=21)
        label('点击地图空白处设置目标 · 蓝色为 G1 · 橙色为目标 · 青色为实际轨迹')
        places = tk.Frame(self.ui,bg='#101c2d')
        places.pack(fill='x',padx=24,pady=(8,0))
        for row,names in enumerate((('西北','北','东北'),('西','门口','东'),('西南','南','东南'))):
            for column,name in enumerate(names):
                button = tk.Button(places,command=lambda n=name:self.use_waypoint(n),
                                   bg='#263b53',activebackground='#263b53',relief='flat',bd=0)
                button.bind('<Button-3>',lambda event,n=name:self.begin_assign(n))
                button.grid(row=row,column=column,padx=3,pady=2,sticky='w')
                self.waypoint_buttons[name]=button
        self.rendered = rendered
        self.refresh_waypoint_buttons()
        self.canvas = tk.Canvas(self.ui, bg='#101c2d', highlightthickness=0, height=490)
        self.canvas.pack(fill='both', expand=True, padx=20, pady=12)
        pixels = np.where(robot.navigation.occupied[..., None], [48,65,84], [226,234,239]).astype('uint8')
        self.map_image = Image.fromarray(pixels)
        self.canvas.bind('<Configure>', self.resize)
        self.canvas.bind('<Button-1>', self.click)
        self.status = tk.StringVar(value='就绪：点击地图开始行走；右键地点按钮可设置或重设位置。')
        self.coordinates = tk.StringVar()
        label(variable=self.status, color='#67e8c6')
        label(variable=self.coordinates)
        buttons = tk.Frame(self.ui, bg='#101c2d')
        buttons.pack(fill='x', padx=24, pady=12)
        for title, command in [('向北 0.8m  [N]','N'), ('向西 0.8m  [W]','W'), ('返回起点  [H]','H')]:
            button = tk.Button(buttons, command=lambda c=command: self.shortcut(c),
                               bg='#263b53', activebackground='#263b53', relief='flat',bd=0)
            button.photo = rendered(title,'white','#263b53',padding=(12,7))
            button.configure(image=button.photo)
            button.pack(side='left',padx=(0,10))
        stop_button = tk.Button(buttons,command=self.stop,bg='#a33d51',
                                activebackground='#a33d51',relief='flat',bd=0)
        stop_button.photo = rendered('停止  [Esc]','white','#a33d51',padding=(22,7))
        stop_button.configure(image=stop_button.photo)
        stop_button.pack(side='right')
        label('仅仿真 · 办公室栅格代理场景 · 墙体及不可达目标会被拒绝', size=10)
        self.ui.bind('<Escape>', lambda _: self.stop())
        for letter in 'nNwWhH':
            self.ui.bind(letter, lambda e: self.shortcut(e.char.upper()))

    def close(self):
        self.stop()
        self.closed = True

    def stop(self):
        self.pending = None
        self.cancelled = True
        self.assigning = None
        self.refresh_waypoint_buttons()
        self.robot.stop(self.robot.epoch)
        self.status.set('正在停止…' if self.busy else '已停止，点击地图可选择新目标。')

    def shortcut(self, key):
        self.submit(self.home + {'N':[0.,.8], 'W':[-.8,0.], 'H':[0.,0.]}[key])

    def refresh_waypoint_buttons(self):
        for name,button in self.waypoint_buttons.items():
            defined = name in self.waypoints
            color = '#365742' if defined else '#263b53'
            suffix = ' · 前往' if defined else ' · 未设'
            if self.assigning == name:
                suffix = ' · 点地图'
                color = '#8c5c22'
            button.photo = self.rendered(name+suffix,'white',color,padding=(10,5))
            button.configure(image=button.photo,bg=color,activebackground=color)

    def begin_assign(self,name):
        if self.busy:
            self.status.set('正在行走；请先按“停止”，再设置地点。')
            return
        self.assigning = name
        self.refresh_waypoint_buttons()
        self.status.set(f'正在设置“{name}”：请点击地图上该区域的可通行位置。门口请点右下方开口内侧。')

    def use_waypoint(self,name):
        if self.assigning == name:
            self.status.set(f'请点击地图，保存“{name}”的位置。')
            return
        if name not in self.waypoints:
            self.begin_assign(name)
            return
        self.submit(self.waypoints[name])

    def screen(self, xy):
        p = self.inverse @ np.r_[xy, 1.]
        return self.offset + (p[:2]/p[2] + .5)*self.scale

    def click(self, event):
        pixel = (np.array([event.x,event.y])-self.offset)/self.scale - .5
        h,w = self.robot.navigation.occupied.shape
        if -.5 <= pixel[0] < w-.5 and -.5 <= pixel[1] < h-.5:
            point = self.robot.navigation.T_map_px @ np.r_[pixel,1.]
            point = point[:2]/point[2]
            if self.assigning is not None:
                context = Context(self.robot,self.robot,{},settings=self.robot.settings,
                                  navigation=navigate_real,navigation_evidence=self.robot.navigation_evidence,
                                  cancelled=lambda:False)
                candidate = Runner(context,task_registry()).prepare(
                    'navigate',{'goal_xy_m':point.tolist(),'snap':False},self.robot.snapshot())
                if candidate.finished:
                    self.status.set(f'这个位置不可到达（{candidate.result.code}），请在地图上另选一个可通行点。')
                    return
                name,self.assigning=self.assigning,None
                self.waypoints[name]=point.copy()
                self.waypoint_file.write_text(json.dumps({
                    'map_shape':list(self.robot.navigation.occupied.shape),
                    'T_map_px':self.robot.navigation.T_map_px.tolist(),
                    'points':{key:value.tolist() for key,value in self.waypoints.items()}},
                    ensure_ascii=False,indent=2)+'\n')
                self.refresh_waypoint_buttons()
                self.status.set(f'已保存“{name}”位置；左键对应按钮可导航，右键可重设。')
                self.draw()
            else:
                self.submit(point)

    def submit(self, goal):
        if self.busy:
            self.status.set('正在行走；如需更换目标，先按“停止”。')
            return
        self.pending = np.asarray(goal,dtype=float)
        self.goal, self.result = self.pending.copy(), None
        self.cancelled = False
        self.path, self.trace = [], [self.robot.measure().base_pose[:2].tolist()]
        self.status.set(f'正在规划：({goal[0]:.2f}, {goal[1]:.2f}) m')
        self.draw()

    def resize(self, event):
        h,w = self.robot.navigation.occupied.shape
        self.scale = min(event.width/w,event.height/h)
        size = np.maximum(1,np.rint(np.array([w,h])*self.scale).astype(int))
        self.offset = (np.array([event.width,event.height])-size)/2
        self.photo = ImageTk.PhotoImage(self.map_image.resize(tuple(size),Image.Resampling.NEAREST))
        self.canvas.delete('map')
        self.canvas.create_image(*self.offset,anchor='nw',image=self.photo,tags='map')
        self.canvas.tag_lower('map')
        self.draw()

    def draw(self):
        self.canvas.delete('overlay')
        self.waypoint_photos = {}
        def line(points,color,**kw):
            if len(points)>=2:
                self.canvas.create_line(*np.array([self.screen(p) for p in points]).flatten(),
                                        fill=color,width=3,tags='overlay',**kw)
        line(self.path,'#d88b22',dash=(8,5))
        line(self.trace,'#069c9c')
        home = self.screen(self.home)
        self.canvas.create_oval(*(home-5),*(home+5),outline='#5c6f84',width=2,tags='overlay')
        for name,point in self.waypoints.items():
            marker = self.screen(point)
            self.canvas.create_oval(*(marker-5),*(marker+5),fill='#65be73',
                                    outline='#172e21',width=2,tags='overlay')
            photo = self.rendered(name,'white','#365742',size=10,padding=(5,2))
            self.waypoint_photos[name] = photo
            self.canvas.create_image(marker[0]+8,marker[1]-8,anchor='sw',image=photo,tags='overlay')
        if self.goal is not None:
            p = self.screen(self.goal)
            color = '#db3f59' if self.result is not None and not self.result.ok else '#e98a16'
            self.canvas.create_oval(*(p-9),*(p+9),outline=color,width=3,tags='overlay')
            self.canvas.create_line(p[0]-13,p[1],p[0]+13,p[1],fill=color,width=2,tags='overlay')
            self.canvas.create_line(p[0],p[1]-13,p[0],p[1]+13,fill=color,width=2,tags='overlay')
        pose = self.robot.measure().base_pose
        p = self.screen(pose[:2])
        ahead = self.screen(pose[:2]+.38*np.array([math.cos(pose[2]),math.sin(pose[2])]))
        self.canvas.create_oval(*(p-8),*(p+8),fill='#1a67df',outline='white',width=2,tags='overlay')
        self.canvas.create_line(*p,*ahead,fill='#1a67df',width=3,arrow='last',tags='overlay')
        gap = '' if self.goal is None else f'  |  距目标 {np.linalg.norm(pose[:2]-self.goal):.2f} m'
        self.coordinates.set(f'G1  x={pose[0]:.2f} m   y={pose[1]:.2f} m   朝向={math.degrees(pose[2]):.0f}°'+gap)

    def pump(self):
        if self.closed:
            return
        if self.robot.time-self.last_sample >= .1:
            xy = self.robot.measure().base_pose[:2].tolist()
            if self.busy and (not self.trace or math.dist(self.trace[-1],xy)>.015):
                self.trace.append(xy)
            self.last_sample = self.robot.time
            self.draw()
        self.ui.update_idletasks()
        self.ui.update()

    def navigate(self):
        goal,self.pending = self.pending,None
        self.busy,self.cancelled = True,False
        start = self.robot.measure().base_pose[:2].copy()
        print('SIM_GOAL '+json.dumps(goal.tolist()),flush=True)
        context = Context(self.robot,self.robot,{},settings=self.robot.settings,
                          navigation=navigate_real,navigation_evidence=self.robot.navigation_evidence,
                          cancelled=lambda: self.cancelled or self.closed)
        runner = Runner(context,task_registry())
        try:
            attempt = runner.prepare('navigate',{'goal_xy_m':goal.tolist(),'snap':False},self.robot.snapshot())
            if not attempt.finished:
                self.path = list(attempt.plan.waypoints)
                self.status.set('正在行走：橙色虚线为计划路径，青色实线为实际轨迹。')
                self.draw()
                self.pump()
            self.result = attempt.result if attempt.finished else runner.execute(attempt)
            end = self.robot.measure().base_pose[:2]
            self.trace.append(end.tolist())
            gap = float(np.linalg.norm(end-goal))
            if self.result.ok:
                self.status.set(f'已到达目标 · 终点误差 {gap*100:.1f} cm · 点击地图继续。')
            elif self.result.code=='CANCELLED':
                self.status.set('已停止，点击地图可选择新目标。')
            else:
                self.status.set(f'目标未完成（{self.result.code}）：可能靠近墙体或没有可通行路径，请换一个点。')
            report = dict(code=self.result.code,ok=self.result.ok,goal_xy=goal.tolist(),
                          start_xy=start.tolist(),end_xy=end.tolist(),error_m=gap,
                          motion_m=float(np.linalg.norm(end-start)),
                          message=self.result.message,
                          physics_valid=bool(self.robot.evaluation_state()['physics_valid']))
            print('SIM_RESULT '+json.dumps(report),flush=True)
            return report
        finally:
            self.robot.stop(self.robot.epoch)
            self.busy = False
            self.draw()


def verify(panel,hook):
    """Check canvas coordinates, arrival, occupied-cell refusal and cancellation recovery."""
    reports = []
    def click(goal):
        point = panel.screen(goal)
        panel.canvas.event_generate('<Button-1>',x=round(point[0]),y=round(point[1]))
        panel.pump()
        assert panel.pending is not None,'Canvas click did not produce a goal'
        assert np.linalg.norm(panel.pending-goal)<=panel.robot.navigation.resolution,'Map transform mismatch'
        report = panel.navigate()
        reports.append(report)
        return report
    panel.pump()
    for delta in ([0.,.8],[-.8,0.],[0.,0.]):
        r = click(panel.home+delta)
        assert r['ok'] and r['error_m']<=panel.robot.settings.motion.arrival_distance and r['physics_valid']
    cells = np.argwhere(panel.robot.navigation.occupied)
    points = np.column_stack([cells[:,::-1],np.ones(len(cells))])@panel.robot.navigation.T_map_px.T
    points = points[:,:2]/points[:,2,None]
    wall = points[np.argmin(np.linalg.norm(points-panel.home,axis=1))]
    r = click(wall)
    assert r['code']=='NO_ROUTE' and r['motion_m']<.01 and r['physics_valid']
    hook['cancel_origin'] = panel.robot.measure().base_pose[:2].copy()
    r = click(panel.home+[0.,1.8])
    assert r['code']=='CANCELLED' and r['physics_valid'] and r['motion_m']>=.15
    r = click(panel.home)
    assert r['ok'] and r['physics_valid'],'Navigation failed to recover after cancellation'
    output = dict(passed=True,scope='MuJoCo map clicks only',cases=reports)
    (ARTIFACTS/'office-click-verification.json').write_text(json.dumps(output,indent=2))
    print('VERIFY_PASS '+json.dumps(output),flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--verify',action='store_true')
    args = parser.parse_args()
    base = Settings()
    settings = replace(base,camera=replace(base.camera,width=160,height=120))
    robot = MujocoBackend(ROOT,settings,scene=SCENE)
    panel,keys = None,SimpleQueue()
    try:
        robot.model.vis.map.zfar = 100./robot.model.stat.extent
        robot.model.vis.map.fogstart = 80./robot.model.stat.extent
        robot.model.vis.map.fogend = 100./robot.model.stat.extent
        with mujoco.viewer.launch_passive(robot.model,robot.data,key_callback=keys.put,
                                         show_left_ui=False,show_right_ui=False) as viewer:
            viewer.opt.geomgroup[2:4] = 1
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.fixedcamid = viewer.cam.trackbodyid = -1
            viewer.cam.lookat[:] = [.725,-1.75,.5]
            viewer.cam.distance,viewer.cam.azimuth,viewer.cam.elevation = 19,90,-85
            viewer.sync()
            panel = MapPanel(robot)
            original_tick = robot._tick
            last_wall,last_sync = time.monotonic(),robot.time
            hook = {}
            def tick():
                nonlocal last_wall,last_sync
                if not viewer.is_running():
                    raise KeyboardInterrupt
                before = robot.time
                original_tick()
                if not args.verify:
                    delay = robot.time-before-(time.monotonic()-last_wall)
                    if delay>0:
                        time.sleep(delay)
                last_wall = time.monotonic()
                if ('cancel_origin' in hook and
                        np.linalg.norm(robot.measure().base_pose[:2]-hook['cancel_origin'])>=.2):
                    hook.clear()
                    panel.stop()
                if robot.time-last_sync>=1/30:
                    while True:
                        try:
                            key = keys.get_nowait()
                        except Empty:
                            break
                        if key==256:
                            panel.stop()
                        elif key in (ord('N'),ord('W'),ord('H')):
                            panel.shortcut(chr(key))
                    panel.pump()
                    with viewer.lock():
                        viewer.user_scn.ngeom = 0
                        if panel.goal is not None:
                            mujoco.mjv_initGeom(viewer.user_scn.geoms[0],mujoco.mjtGeom.mjGEOM_SPHERE,
                                np.array([.10,.10,.10]),np.r_[panel.goal,.12],np.eye(3).flatten(),
                                np.array([1.,.45,.05,.85]))
                            viewer.user_scn.ngeom = 1
                    viewer.sync()
                    last_sync = robot.time
            robot._tick = tick
            panel.pump()
            print('VIEWER_READY: Click map to navigate. N/W/H. Esc stops. SIMULATION ONLY.',flush=True)
            if args.verify:
                verify(panel,hook)
            else:
                while viewer.is_running() and not panel.closed:
                    if panel.pending is not None:
                        panel.navigate()
                    else:
                        robot.stop(robot.epoch)
                        robot.advance(robot.epoch,.02)
    except KeyboardInterrupt:
        pass
    finally:
        if panel is not None:
            panel.ui.destroy()
        robot.close()


if __name__=='__main__':
    main()
