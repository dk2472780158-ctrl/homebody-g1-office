"""Read workstation sensor files; publish only isolated AMCL map/scan/TF.

No robot connection, velocity publisher, navigation action, or mode services.
The compatible live-state export is gated by covariance, scan geometry, particle
mass, repeated convergence, input freshness, and the current sensor session.
"""
import json,math,os,runpy,signal,time
from collections import OrderedDict
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster
from geometry import planar,ros_map,scan_ranges,scan_quality,candidate_ready,historical_consistent,particle_mass,tracking_ready,seed_ready
from office_live_geometry import gravity_alignment,estimate_floor,HistoricalMatcher

STAGE=Path(__file__).resolve().parent
LIVE=Path('/workspace/runtime/live-readonly')
validate_raw=runpy.run_path(str(STAGE/'office-live-map-worker.py'))['validate_raw']

def atomic(path,value):
    pending=path.with_suffix('.amcl-tmp')
    pending.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False));os.replace(pending,path)

def yaw_of(T):return math.atan2(T[1,0],T[0,0])
def delta_angle(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))
def pose_matrix(p):return planar(p.position.x,p.position.y,Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_euler('xyz')[2])
def stamp_key(stamp):return stamp.sec*1000000000+stamp.nanosec

class Adapter(Node):
    def __init__(self):
        super().__init__('sensor_file_adapter',namespace='/homebody_amcl')
        with np.load(STAGE/'real-office-navigation.npz',allow_pickle=False) as data:
            self.occupancy=data['occupancy'].astype(bool);self.map_T=data['T_map_px']
        self.floor=float(json.loads((STAGE/'real-navigation-map-report.json').read_text())['floor_m'])
        self.matcher=HistoricalMatcher(STAGE/'g1_fastlio.historical-office.pcd')
        self.history_from_office=np.linalg.inv(np.asarray(json.loads((STAGE/'historical-office-registration.json').read_text())['T_office_map_from_provided_pcd']))
        data,origin,res=ros_map(self.occupancy,self.map_T)
        self.map=OccupancyGrid();self.map.header.frame_id='office_map'
        self.map.info.resolution=res;self.map.info.width=data.shape[1];self.map.info.height=data.shape[0]
        self.map.info.origin.position.x=origin[0];self.map.info.origin.position.y=origin[1]
        self.map.info.origin.orientation.w=1.;self.map.data=data.ravel().tolist()
        self.map_pub=self.create_publisher(OccupancyGrid,'map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.scan_pub=self.create_publisher(LaserScan,'scan',qos_profile_sensor_data)
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped,'amcl_pose',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(ParticleCloud,'particle_cloud',self.on_particles,qos_profile_sensor_data)
        self.lifecycle=self.create_client(ChangeState,'amcl/change_state')
        self.global_client=self.create_client(Empty,'reinitialize_global_localization')
        self.nomotion=self.create_client(Empty,'request_nomotion_update')
        self.session=None;self.level=None;self.last_seq=None;self.snapshots=OrderedDict()
        self.particles=None;self.particles_at=0.;self.candidate=None;self.accepted=None
        self.particles_odom=None
        self.stable=0;self.pose_at=0.;self.geometry_at=0.;self.quality={};self.raw={}
        self.global_at=0.;self.reset_future=None;self.reset_done=False;self.last_nomotion=0.
        self.last_message=None;self.published_scans=0;self.started=time.time()
        self.last_seed=None
        self.unavailable_since=time.time()
        self.trusted=None;self.trusted_at=0.;self.trusted_covariance=None
        self.seed_reference=None;self.seed_reference_at=0.
        self.publish_map()

    def publish_map(self):
        self.map.header.stamp=self.get_clock().now().to_msg();self.map_pub.publish(self.map)

    def initialize(self):
        if not self.lifecycle.wait_for_service(timeout_sec=20.):raise RuntimeError('AMCL lifecycle service unavailable')
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition
            future=self.lifecycle.call_async(request);rclpy.spin_until_future_complete(self,future,timeout_sec=20.)
            if not future.done() or not future.result().success:raise RuntimeError('AMCL lifecycle transition failed')
        self.create_timer(.1,self.tick)

    def reset(self,session):
        self.session=session;self.level=None;self.last_seq=None;self.snapshots.clear()
        self.candidate=self.accepted=None;self.stable=0;self.pose_at=self.geometry_at=0.
        self.particles=None;self.particles_odom=None;self.particles_at=0.;self.quality={};self.reset_done=False
        self.reset_future=None;self.global_at=0.
        self.unavailable_since=time.time()
        self.trusted=None;self.trusted_at=0.;self.trusted_covariance=None
        self.seed_reference=None;self.seed_reference_at=0.
        self.get_logger().info('New sensor session; starting AMCL global localization: '+session)

    def request_global(self):
        if not self.global_client.service_is_ready():return
        self.reset_done=False;self.accepted=self.candidate=None;self.stable=0
        self.particles=None;self.particles_odom=None
        self.snapshots.clear();self.reset_future=self.global_client.call_async(Empty.Request())
        self.global_at=time.time();self.pose_at=self.geometry_at=0.
        self.unavailable_since=self.global_at
        self.trusted=None;self.trusted_at=0.;self.trusted_covariance=None
        self.seed_reference=None;self.seed_reference_at=0.

    def on_particles(self,msg):
        if not self.reset_done:return
        self.particles=msg;self.particles_at=time.time()
        self.particles_odom=next(reversed(self.snapshots.values()))[0] if self.snapshots else None

    def on_pose(self,msg):
        try:
            key=stamp_key(msg.header.stamp)
            if not self.reset_done or key not in self.snapshots:return
            odom_base,ranges,received,recent=self.snapshots[key]
            if time.time()-received>1.5:return
            map_base=pose_matrix(msg.pose.pose)
            quality=scan_quality(ranges,map_base,self.occupancy,self.map_T)
            mass=0.
            if self.particles is not None and self.particles_odom is not None:
                poses=[[p.pose.position.x,p.pose.position.y,2*math.atan2(p.pose.orientation.z,p.pose.orientation.w)] for p in self.particles.particles]
                mass=particle_mass(poses,[p.weight for p in self.particles.particles],np.linalg.inv(self.particles_odom)@odom_base,map_base)
            covariance=list(msg.pose.covariance)
            self.quality={**quality,'covariance':covariance,'dominant_particle_mass':mass,'particles':len(self.particles.particles) if self.particles else 0,
                'particle_age_s':time.time()-self.particles_at,'candidate_xyyaw':[float(map_base[0,3]),float(map_base[1,3]),math.degrees(yaw_of(map_base))]}
            transform=map_base@np.linalg.inv(odom_base)
            historical=self.history_from_office@transform@self.level
            structure=self.matcher.current_structure(recent,historical)
            q3=self.matcher.metrics(structure,historical) if len(structure)>=100 else {'points':len(structure),'within_0_2m_fraction':0.,'median_distance_m':2.}
            self.quality['historical_3d']=q3
            reference=self.trusted if 0<=time.time()-self.trusted_at<10 else None
            if reference is None and 0<=time.time()-self.seed_reference_at<10:reference=self.seed_reference
            tracking=tracking_ready(covariance,quality,mass,q3,transform,reference)
            self.quality['tracking_hysteresis']=tracking
            if not (candidate_ready(covariance,quality,mass,q3) or tracking) or not historical_consistent(q3):
                self.stable=0;self.accepted=None;self.candidate=None;return
            close=self.candidate is not None and np.linalg.norm(transform[:2,3]-self.candidate[:2,3])<.08 and abs(delta_angle(yaw_of(transform),yaw_of(self.candidate)))<math.radians(5)
            self.stable=self.stable+1 if close else 1
            self.candidate=transform
            self.pose_at=time.time();self.geometry_at=received
            if self.stable>=3:
                self.accepted=transform;self.trusted=transform.copy();self.trusted_at=time.time()
                self.trusted_covariance=covariance
        except (ValueError,KeyError,TypeError) as exc:
            self.accepted=None;self.get_logger().warning(str(exc))

    def export(self,status,message,pose=None,transform=None,quality=None):
        raw=self.raw
        q=self.quality if quality is None else quality
        value=dict(status=status,message=message,updated_unix_s=time.time(),session_id=raw.get('session_id'),
            readonly=True,candidate=True,navigation_ready=False,base_extrinsics_validated=False,
            localization_backend='nav2_amcl',pose=pose,quality={**q,'latest_cloud':q},
            geometry_checked_unix_s=self.geometry_at,cloud_received_unix_s=raw.get('cloud_received_unix_s'),
            odometry_received_unix_s=raw.get('odometry_received_unix_s'),
            cloud_messages=raw.get('cloud_messages',0),odometry_messages=raw.get('odometry_messages',0),
            T_office_from_current=transform.tolist() if transform is not None else None,
            amcl=dict(stable_updates=self.stable,published_scans=self.published_scans,global_started_unix_s=self.global_at))
        atomic(STAGE/'amcl-state.json',value)
        # Shadow mode validates the new source before it becomes the UI source.
        if (STAGE/'publish-live-state').exists():atomic(LIVE/'live-state.json',value)
        if message!=self.last_message:
            print(json.dumps(dict(at=time.time(),status=status,message=message,quality=self.quality),ensure_ascii=False),flush=True)
            self.last_message=message

    def tick(self):
        try:
            now=time.time();raw=json.loads((LIVE/'raw-state.json').read_text());self.raw=raw
            odom=validate_raw(raw,now)
            if raw['session_id']!=self.session:self.reset(raw['session_id'])
            if self.reset_future is not None:
                if not self.reset_future.done():
                    self.export('matching','AMCL 正在初始化全办公室粒子');return
                self.reset_future.result();self.reset_future=None;self.reset_done=True
            if not self.reset_done:
                self.request_global();self.export('matching','AMCL 正在初始化全办公室粒子');return
            current=np.eye(4);current[:3,:3]=Rotation.from_quat(odom['xyzw']).as_matrix();current[:3,3]=odom['xyz']
            seed_path=STAGE/'seed.json'
            if self.accepted is None and seed_path.exists():
                seed=json.loads(seed_path.read_text())
                if seed.get('valid') and seed_ready(seed['quality'],seed.get('alternate_score_m')) and seed.get('session_id')==self.session and 0<=now-seed['updated_unix_s']<60 and seed['updated_unix_s']!=self.last_seed and self.unavailable_since is not None and now-self.unavailable_since>2:
                    seed_T=np.asarray(seed['T_office_from_current'])
                    # Never bridge a previously accepted transform using the
                    # new seed's different roll/pitch/floor decomposition.
                    self.trusted=None;self.trusted_at=0.;self.trusted_covariance=None
                    # Fit roll/pitch and floor from the verified 3-D seed once.
                    self.level=np.linalg.inv(planar(*seed_T[:2,3],yaw_of(seed_T)))@seed_T
                    self.seed_reference=seed_T@np.linalg.inv(self.level);self.seed_reference_at=now
                    self.snapshots.clear();self.last_seq=None
                    self.particles=None;self.particles_odom=None
                    pose=seed_T@current
                    msg=PoseWithCovarianceStamped();msg.header.frame_id='office_map';msg.header.stamp=self.get_clock().now().to_msg()
                    msg.pose.pose.position.x=float(pose[0,3]);msg.pose.pose.position.y=float(pose[1,3])
                    msg.pose.pose.orientation.z=math.sin(yaw_of(pose)/2);msg.pose.pose.orientation.w=math.cos(yaw_of(pose)/2)
                    covariance=np.zeros((6,6));covariance[0,0]=covariance[1,1]=.04;covariance[5,5]=math.radians(10)**2
                    msg.pose.covariance=covariance.ravel().tolist();self.initial_pub.publish(msg)
                    self.last_seed=seed['updated_unix_s'];self.global_at=now;self.unavailable_since=now;self.stable=0;self.candidate=None
                    self.get_logger().info('AMCL automatic seed passed independent 3-D hypothesis comparison')
            seq=raw['cloud_sequence']
            if seq!=self.last_seq:
                with np.load(LIVE/'clouds.npz',allow_pickle=False) as data:
                    if str(data['session_id'])!=self.session or abs(int(data['sequence'])-seq)>1:raise ValueError('正在同步 AMCL 点云快照')
                    received=float(data['received_unix_s'])
                    if not 0<=now-received<.8:raise ValueError('AMCL 激光切片已过期')
                    xyz=data['latest_xyz'].copy();accumulated=data['xyz'].copy()
                    recent=data['recent_xyz'].copy() if 'recent_xyz' in data else xyz
                if self.level is None:
                    self.level=gravity_alignment(raw['initial_up_current_xyz'])
                    floor=estimate_floor(accumulated@self.level[:3,:3].T)
                    self.level[2,3]=self.floor-floor['height_m']
                if not np.isfinite(xyz).all():raise ValueError('AMCL 点云包含无效数值')
                level_body=self.level@current
                odom_base=planar(*level_body[:2,3],yaw_of(level_body))
                points=xyz@self.level[:3,:3].T+self.level[:3,3]
                ranges=scan_ranges(points,odom_base[:2,3],yaw_of(odom_base),self.floor)
                if np.isfinite(ranges).sum()<60:raise ValueError('AMCL 有效激光切片不足')
                stamp=self.get_clock().now().to_msg()
                tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='hb_amcl_odom';tf.child_frame_id='hb_amcl_base'
                tf.transform.translation.x=float(odom_base[0,3]);tf.transform.translation.y=float(odom_base[1,3])
                tf.transform.rotation.z=math.sin(yaw_of(odom_base)/2);tf.transform.rotation.w=math.cos(yaw_of(odom_base)/2)
                self.tf.sendTransform(tf)
                scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='hb_amcl_base'
                scan.angle_min=-math.pi;scan.angle_increment=math.radians(.5);scan.angle_max=scan.angle_min+719*scan.angle_increment
                scan.range_min=.43;scan.range_max=12.;scan.scan_time=.2;scan.ranges=ranges.astype(np.float32).tolist()
                self.snapshots[stamp_key(stamp)]=(odom_base,ranges,received,recent)
                while len(self.snapshots)>80:self.snapshots.popitem(last=False)
                self.scan_pub.publish(scan);self.last_seq=seq;self.published_scans+=1
            if now-self.last_nomotion>.4 and self.nomotion.service_is_ready():
                self.nomotion.call_async(Empty.Request());self.last_nomotion=now
            if self.accepted is None or now-self.pose_at>2 or now-self.geometry_at>2:
                if self.unavailable_since is None:self.unavailable_since=now
                # Hold a verified map transform through a short measurement
                # rejection only when fresh current geometry still supports it.
                # This is live odometry tracking, never replay of an old pose.
                bridged=False
                if self.trusted is not None and 0<=now-self.trusted_at<=3 and self.snapshots:
                    odom_base,ranges,received,recent=next(reversed(self.snapshots.values()))
                    if 0<=now-received<.8:
                        history=self.history_from_office@self.trusted@self.level
                        structure=self.matcher.current_structure(recent,history)
                        q3=self.matcher.metrics(structure,history) if len(structure)>=100 else {}
                        q2=scan_quality(ranges,self.trusted@odom_base,self.occupancy,self.map_T)
                        cov=np.asarray(self.trusted_covariance).reshape(6,6).copy()
                        dt=now-self.trusted_at;cov[0,0]+=(.02*dt)**2;cov[1,1]+=(.02*dt)**2
                        cov[5,5]+=math.radians(2*dt)**2
                        if historical_consistent(q3) and (candidate_ready(cov,q2,1.,q3) or
                            tracking_ready(cov,q2,1.,q3,self.trusted,self.trusted)):
                            transform=self.trusted@self.level;body=transform@current
                            self.geometry_at=received
                            q={**q2,'historical_3d':q3,'tracking_bridge_s':dt,'covariance':cov.ravel().tolist()}
                            self.export('live','AMCL 跟踪 · 本帧候选未采用，实时里程计及当前点云复核通过',
                                dict(frame='office_map',child_frame='FAST-LIO body',xyz_m=body[:3,3].tolist(),
                                    yaw_degrees=math.degrees(yaw_of(body)),sensor_stamp_s=odom['stamp_s']),transform,quality=q)
                            bridged=True
                if bridged:
                    self.unavailable_since=None
                    return
                self.export('matching','AMCL 正在全局定位／验证粒子收敛')
                if now-self.unavailable_since>12:self.request_global()
                return
            self.unavailable_since=None
            transform=self.accepted@self.level;body=transform@current
            self.export('live','AMCL 实时定位 · 激光切片与粒子收敛检查通过',
                dict(frame='office_map',child_frame='FAST-LIO body',xyz_m=body[:3,3].tolist(),
                    yaw_degrees=math.degrees(yaw_of(body)),sensor_stamp_s=odom['stamp_s']),transform)
        except (OSError,ValueError,KeyError,TypeError) as exc:
            self.export('waiting',str(exc))

def main():
    rclpy.init();node=Adapter()
    try:node.initialize();rclpy.spin(node)
    finally:
        node.export('stopped','AMCL 定位进程已停止');node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
