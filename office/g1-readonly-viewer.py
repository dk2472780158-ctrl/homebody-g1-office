"""Read-only 3D viewer of file-based localization and public robot health."""
import hb_config
import json,signal,threading,time,urllib.request,sys
from pathlib import Path
import numpy as np
import viser
from scipy.spatial.transform import Rotation
from office_live_geometry import HistoricalMatcher,voxel
from office_live_panel import display_state
from office_landmarks import visible_marks,approach_points
from g1_pointcloud_slice import height_slice,obstacle_slice,scan_projection
sys.path.insert(0,str(Path(hb_config.expand('${HB_OFFICE_ROOT}/sensors/vendor'))))
from g1_readonly_geometry import Telemetry,BaseModel,StaticRoute

ROOT=Path(hb_config.expand('${HB_OFFICE_ROOT}'))
LIVE=ROOT/'sensors/runtime/live-readonly'

def main():
    server=viser.ViserServer(host='127.0.0.1',port=8093)
    server.gui.configure_theme(titlebar_content=None,control_layout='floating')
    stop=threading.Event()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stop.set())
    server.gui.add_markdown('# G1 真机点云与定位\n只读查看；本页面没有运动控制。')
    status=server.gui.add_markdown('等待定位数据…')
    health=server.gui.add_markdown('正在读取机器人状态…')
    base_status=server.gui.add_markdown('正在核对关节与雷达重力方向…')
    cloud_mode=server.gui.add_dropdown('当前点云显示',options=('障碍高度切片','二维障碍投影','原始点云'),initial_value='障碍高度切片')
    slice_status=server.gui.add_markdown('障碍切片：离地 0.15–1.80 m；二维投影按 0.5° 取最近回波。')
    server.gui.add_markdown('灰色：历史办公室图；橙色：当前点云；蓝色：雷达 IMU 位置。\n\n'
        '黄色：按模型推算的骨盆位置，尚未经现场标定。定位未通过时不绘制全局位置。')
    history=HistoricalMatcher(ROOT/'g1_fastlio.historical-office.pcd')
    registration=np.array(json.loads((ROOT/'historical-office-registration.json').read_text())['T_office_map_from_provided_pcd'])
    target=history.target@registration[:3,:3].T+registration[:3,3]
    floor=float((registration@[0,0,history.floor['height_m'],1])[2])
    telemetry=Telemetry()
    base_model=BaseModel(Path(hb_config.expand('${HB_REPO_ROOT}/assets/robot/g1_29dof_with_hand.urdf')))
    planner=StaticRoute(ROOT/'real-office-navigation.npz' if (ROOT/'real-office-navigation.npz').exists() else ROOT/'office-map-proxy-scene/map.npz')
    server.scene.add_point_cloud('/office/static-obstacles',
        points=np.column_stack([planner.obstacle_xy,np.full(len(planner.obstacle_xy),floor+.02)]).astype(np.float32),
        colors=(132,85,153),point_size=.045)
    history_cloud=server.scene.add_point_cloud('/office/history',points=height_slice(target,floor).astype(np.float32),
        colors=(139,150,159),point_size=.025)
    slice_file=ROOT/'office-cloud-slice/cloud-slice.npz'
    if slice_file.exists():
        with np.load(slice_file,allow_pickle=False) as data:history_cloud.points=data['xyz'].astype(np.float32)
    cloud=server.scene.add_point_cloud('/office/current',points=np.zeros((1,3),np.float32),
        colors=np.array([[248,145,44]],np.uint8),point_size=.045,visible=False)
    local=server.scene.add_point_cloud('/local/current',points=np.zeros((1,3),np.float32),
        colors=np.array([[248,145,44]],np.uint8),point_size=.04,visible=False)
    body=server.scene.add_frame('/office/sensor',axes_length=.5,axes_radius=.018,visible=False)
    pelvis=server.scene.add_icosphere('/office/pelvis-candidate',radius=.09,color=(239,195,38),visible=False)
    pelvis_label=server.scene.add_label('/office/pelvis-label','骨盆：模型推算，未标定',visible=False)
    theta=np.linspace(0,2*np.pi,65)
    footprint_xy=planner.radius*np.column_stack([np.cos(theta),np.sin(theta)])
    footprint=server.scene.add_line_segments('/office/preview-radius',points=np.zeros((64,2,3),np.float32),
        colors=(239,195,38),line_width=3,visible=False)
    route_handle=server.scene.add_line_segments('/office/route-preview',points=np.zeros((1,2,3),np.float32),
        colors=(67,143,235),line_width=5,visible=False)
    server.scene.add_frame('/local/origin',axes_length=.6,axes_radius=.014)
    points=json.loads((ROOT/'office-waypoints.json').read_text())['points']
    shown=visible_marks(ROOT,points)
    points=approach_points(ROOT,points)
    for label,xy in shown.items():
        server.scene.add_icosphere('/office/waypoints/'+label,radius=.07,color=(49,154,115),position=(*xy,floor+.05))
        server.scene.add_label('/office/labels/'+label,label,position=(*xy,floor+.2))
    live_view={'pose':None,'session':None,'route_start':None,'updated':0.}
    with server.gui.add_folder('九点路径预览 · 不执行',expand_by_default=True):
        goal=server.gui.add_dropdown('目标',options=tuple(points))
        preview=server.gui.add_button('预览到目标的路径')
        clearance_status=server.gui.add_markdown('等待当前起点净空…')
        route_status=server.gui.add_markdown('只检查旧地图静态净空，不驱动机器人。')
        @preview.on_click
        def preview_route(_):
            start=live_view['pose']
            if start is None or time.monotonic()-live_view['updated']>2:
                route_handle.visible=False;route_status.content='定位或关节数据无效，不能规划。';return
            try:
                xy,detail=planner.navigation_route(start[:2,3],points[goal.value])
                xyz=np.column_stack([xy,np.full(len(xy),floor+.07)])
                route_handle.points=np.stack([xyz[:-1],xyz[1:]],axis=1).astype(np.float32) if len(xyz)>1 else np.stack([xyz,xyz],axis=1).astype(np.float32)
                route_handle.visible=True
                live_view['route_start']=start[:2,3].copy()
                route_status.content=(f'**预览：{goal.value}**\n\n长度 {detail["length_m"]:.2f} m，'
                    f'最小静态净空 {detail["minimum_clearance_m"]:.2f} m；采用半径 {detail["radius_m"]:.2f} m。'
                    f'\n\n停车点调整 {detail["goal_adjustment_m"]:.2f} m；停车点净空 {detail["stop_clearance_m"]:.2f} m，为再次转向留空间。'
                    '\n\n起点来自未标定骨盆模型；尚未检查临时障碍，不执行。')
            except ValueError as exc:route_handle.visible=False;route_status.content=str(exc)+'。未发送命令。'
        @goal.on_update
        def goal_changed(_):
            route_handle.visible=False
            route_status.content='目标已改变，点击预览重新规划。'
    with server.gui.add_folder('视角',expand_by_default=True):
        top=server.gui.add_button('俯视办公室')
        angled=server.gui.add_button('斜视点云')
        @top.on_click
        def top_view(_):
            for client in server.get_clients().values():
                client.camera.position=(-.5,-1.5,13.);client.camera.look_at=(-.5,-1.5,floor);client.camera.up_direction=(0.,1.,0.)
        @angled.on_click
        def angled_view(_):
            for client in server.get_clients().values():
                client.camera.up_direction=(0.,0.,1.);client.camera.position=(6.,-10.,10.);client.camera.look_at=(-1.,-1.,floor)
    @server.on_client_connect
    def connected(client):
        client.camera.position=(6.,-10.,10.)
        client.camera.look_at=(-1.,-1.,0.)
    def update_health():
        while not stop.is_set():
            try:
                with urllib.request.urlopen('http://127.0.0.1:18099/health',timeout=2) as response:d=json.load(response)
                health.content=(f'电池：{d.get("battery",{}).get("soc","未知")}% · '
                    f'腿使能：{d.get("armed")} · 手臂使能：{d.get("arms_armed")}\n\n'
                    f'控制方：{d.get("vel_owner",{}).get("owner","未知")} · '
                    f'ROS：{d.get("ros")}\n\n本查看器不使能机器人；手势由控制页显式执行。')
            except Exception:health.content='机器人遥操作状态暂不可达。'
            stop.wait(3)
    health_thread=threading.Thread(target=update_health,daemon=True,name='viewer-health-readonly')
    health_thread.start()
    last_sequence=None;last_base_error=None
    while not stop.wait(.1):
        base_snapshot=None
        base_error='等待有效全局定位后显示骨盆模型。'
        try:
            live_view['pose']=None
            pelvis.visible=pelvis_label.visible=footprint.visible=False
            raw=json.loads((LIVE/'raw-state.json').read_text())
            state=json.loads((LIVE/'live-state.json').read_text())
            session=(LIVE/'session-id').read_text().strip()
            fresh=display_state(state,time.time(),session)
            with np.load(LIVE/'clouds.npz',allow_pickle=False) as data:
                valid_cloud=str(data['session_id'])==session and time.time()-float(data['received_unix_s'])<3
                seq=(session,int(data['sequence']))
                xyz=data['latest_xyz'].copy() if valid_cloud else None
            if not valid_cloud:
                base_error='当前点云已过期'
                cloud.visible=local.visible=body.visible=False
                route_handle.visible=False
                base_status.content='当前点云已过期，骨盆候选暂不显示。'
            elif fresh and state.get('T_office_from_current'):
                t=np.array(state['T_office_from_current'])
                world=xyz@t[:3,:3].T+t[:3,3]
                origin=np.asarray(state['pose']['xyz_m'])[:2]
                cut=obstacle_slice(world,floor,origin)
                shown=world if cloud_mode.value=='原始点云' else cut
                if cloud_mode.value=='二维障碍投影':shown,_=scan_projection(cut,origin,floor)
                cloud.points=shown.astype(np.float32);cloud.colors=np.tile(np.array([248,145,44],np.uint8),(len(shown),1))
                slice_status.content=f'障碍切片：离地 0.15–1.80 m；原始 {len(world)} 点 → 障碍 {len(cut)} 点。'
                cloud.visible=True;local.visible=False
                odom=raw['odometry'];current=np.eye(4)
                current[:3,:3]=Rotation.from_quat(odom['xyzw']).as_matrix();current[:3,3]=odom['xyz']
                pose=t@current;body.position=tuple(pose[:3,3]);q=Rotation.from_matrix(pose[:3,:3]).as_quat()
                body.wxyz=tuple(q[[3,0,1,2]]);body.visible=True
                try:
                    if raw.get('session_id')!=session:raise ValueError('关节计算所用定位会话不一致')
                    imu=raw['latest_imu'];samples=telemetry.get()
                    samples['virtual_imu']=(imu['received_unix_s'],imu)
                    best,angles=base_model.tracked(samples,session)
                    base_pose=pose@best['T_imu_from_pelvis']
                    pelvis.position=tuple(base_pose[:3,3]);pelvis.visible=True
                    pelvis_label.position=tuple(base_pose[:3,3]+[0,0,.13]);pelvis_label.visible=True
                    if live_view['session']!=session:route_handle.visible=False
                    if live_view['route_start'] is not None and np.linalg.norm(base_pose[:2,3]-live_view['route_start'])>.1:
                        route_handle.visible=False
                    live_view.update(pose=base_pose,session=session,updated=time.monotonic())
                    base_snapshot={'T_office_from_pelvis':base_pose.tolist(),
                        'gravity_error_degrees':best['gravity_error_degrees'],
                        'mount_validated_unix_s':best['mount_validated_unix_s'],
                        'lowstate_received_unix_s':best['lowstate_received_unix_s'],
                        'imu_received_unix_s':best['imu_received_unix_s'],
                        'mount':'operator confirmed factory mounting; nominal URDF, not measured calibration'}
                    base_error=None
                    ring=np.column_stack([footprint_xy+base_pose[:2,3],np.full(len(theta),floor+.06)])
                    footprint.points=np.stack([ring[:-1],ring[1:]],axis=1).astype(np.float32);footprint.visible=True
                    start_clearance=planner.clearance_at(base_pose[:2,3])
                    if route_handle.visible and start_clearance<=planner.radius:
                        route_handle.visible=False;route_status.content='当前起点净空不足，旧预览已隐藏。'
                    clearance_status.content=(f'起点保守静态净空 **{start_clearance:.2f} m**；规划半径 **{planner.radius:.2f} m**。'
                        '\n\n紫色为旧地图障碍栅格，黄色圆环为规划半径；地图和模型误差仍需核对。')
                    base_status.content=('**骨盆位置：模型推算，未标定**\n\n'
                        f'x={base_pose[0,3]:.2f}，y={base_pose[1,3]:.2f} m；'
                        f'高于旧地图地面 {base_pose[2,3]-floor:.2f} m\n\n'
                        f'本会话安装方向确认时重力差 {best["gravity_error_degrees"]:.1f}°；'
                        f'另一假设 {best["alternative_gravity_error_degrees"]:.1f}°。'
                        '\n\n已固定安装方向，实时更新腰部关节外参。'
                        '\n\n重力只能核对倾斜，不能验证安装平移或绕竖直轴的角度。')
                except (ValueError,KeyError,TypeError) as exc:
                    base_snapshot=None;base_error=str(exc)
                    route_handle.visible=False;base_status.content='骨盆候选暂不显示：'+str(exc)
            else:
                base_error='地图定位无效：'+str(state.get('message','等待点云'))
                local.points=xyz;local.colors=np.tile(np.array([248,145,44],np.uint8),(len(xyz),1))
                local.visible=True;cloud.visible=body.visible=False
                route_handle.visible=False
                base_status.content='等待有效全局定位后显示骨盆模型。'
            last_sequence=seq
            if fresh:
                p=state['pose'];q=state.get('quality',{}).get('latest_cloud',{})
                status.content=('**历史地图匹配在线**\n\n'
                    f'x={p["xyz_m"][0]:.2f} m，y={p["xyz_m"][1]:.2f} m，朝向 {p["yaw_degrees"]:.1f}°\n\n'
                    f'近期点云吻合比例 {q.get("within_0_2m_fraction",0):.1%}；'
                    f'窗口 {q.get("window_frames",1)} 帧；底座外参待确认。')
            else:status.content='**位置暂不显示**\n\n'+str(state.get('message','等待点云'))
            if live_view['pose'] is None:clearance_status.content='当前起点无效，暂停路线预览。'
        except (OSError,ValueError,KeyError,TypeError) as exc:
            base_snapshot=None;base_error=str(exc)
            cloud.visible=local.visible=body.visible=False
            route_handle.visible=pelvis.visible=pelvis_label.visible=footprint.visible=False
            clearance_status.content='当前数据无效，暂停路线预览。'
            status.content='等待新的点云和定位输出。'
        export={'updated_unix_s':time.time(),'session_id':live_view['session'],
            'base':base_snapshot,'base_unavailable_reason':base_error,'execution_enabled':False}
        if base_error!=last_base_error:
            print(json.dumps({'event':'base_validity','at':time.time(),'reason':base_error},ensure_ascii=False),flush=True)
            last_base_error=base_error
        try:
            tmp=LIVE/'real-view-state.tmp';tmp.write_text(json.dumps(export));tmp.replace(LIVE/'real-view-state.json')
        except OSError:pass
    health_thread.join(timeout=3);telemetry.close();server.stop()

if __name__=='__main__':main()
