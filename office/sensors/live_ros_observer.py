"""Export isolated ROS observations to files; no publishers or robot services."""
import argparse
import json
import os
import signal
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from capture_registered_cloud import xyz_array


def atomic_json(path,value):
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(value,allow_nan=False),encoding='utf-8')
    os.replace(temp,path)


def bounded_recent(clouds,cloud_times):
    latest_stamp,latest_received=cloud_times[-1]
    selected=[(points,stamp,received) for points,(stamp,received) in zip(clouds,cloud_times)
              if 0<=latest_stamp-stamp<=2. and 0<=latest_received-received<=2.][-7:]
    return np.concatenate([item[0] for item in selected]),len(selected),selected[0][1],selected[0][2]


def odometry_step(previous, current, delta):
    """A missing output is a freshness failure, not evidence of a pose reset."""
    if not np.isfinite(delta) or delta <= 0:
        raise ValueError('Odometry timestamp regressed; reset localization')
    distance = float(np.linalg.norm(np.asarray(current)-np.asarray(previous)))
    # Bound the motion allowance even after a long output gap.
    if not np.isfinite(distance) or distance > .25+2*min(delta, .5):
        raise ValueError(f'Odometry discontinuity; reset localization: dt={delta:.6f}s, distance={distance:.6f}m')
    return distance


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--session',required=True)
    args=parser.parse_args()
    args.directory.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=Node('homebody_readonly_live_observer')
    keep_running=True
    clouds=deque(maxlen=12)
    cloud_times=deque(maxlen=12)
    last_cloud=0.
    seq=0
    state={'session_id':args.session,'started_unix_s':time.time(),'running':True,
           'scope':'local ROS subscriptions only; no robot publishers/services',
           'cloud_messages':0,'odometry_messages':0,'cloud_received_unix_s':None,
           'odometry_received_unix_s':None,'odometry':None,'cloud_frame':None,
           'cloud_sequence':0,'cloud_stamps_nondecreasing':True,
           'odom_stamps_nondecreasing':True}
    last_stamp={'cloud':None,'odom':None}
    last_odom_xyz=None
    initial_up=[]

    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def cloud(msg):
        nonlocal last_cloud,seq
        now=time.monotonic()
        state['cloud_messages']+=1
        state['cloud_received_unix_s']=time.time()
        current=stamp(msg)
        if last_stamp['cloud'] is not None and current<=last_stamp['cloud']:
            state['cloud_stamps_nondecreasing']=False
        last_stamp['cloud']=current
        if now-last_cloud<.28:return
        points=xyz_array(msg)
        if len(points)<100:return
        if np.max(np.abs(points))>100:
            raise ValueError('Registered cloud outside 100m guard; reset localization')
        last_cloud=now
        clouds.append(points)
        cloud_times.append((current,time.time()))
        recent,recent_frames,recent_stamp,recent_received=bounded_recent(clouds,cloud_times)
        seq+=1
        target=args.directory/'clouds.npz'
        temp=args.directory/'clouds.tmp'
        with temp.open('wb') as f:
            # Keep synchronous callback work short; consumers accept normal NPZ.
            np.savez(f,xyz=np.concatenate(list(clouds)),latest_xyz=points,
                                recent_xyz=recent,
                                recent_frames=np.array(recent_frames),
                                recent_oldest_stamp_s=np.array(recent_stamp),
                                recent_oldest_received_unix_s=np.array(recent_received),
                                session_id=np.array(args.session),sequence=np.array(seq),
                                frame_id=np.array(msg.header.frame_id),stamp_s=np.array(current),
                                received_unix_s=np.array(time.time()))
        os.replace(temp,target)
        state.update(cloud_sequence=seq,cloud_frame=msg.header.frame_id,
                     cloud_points_latest=len(points),cloud_window_frames=len(clouds),
                     cloud_stamp_s=current)

    def odom(msg):
        nonlocal last_odom_xyz
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        v=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        if not np.isfinite(v).all() or abs(np.linalg.norm(v[3:])-1.)>.02:
            raise ValueError('Invalid odometry; reset localization')
        if np.linalg.norm(v[:3])>30:
            raise ValueError('Odometry outside 30m local-office guard; reset localization')
        current=stamp(msg)
        if last_stamp['odom'] is not None and current<=last_stamp['odom']:
            raise ValueError('Odometry timestamp regressed; reset localization')
        if last_odom_xyz is not None:
            delta=current-last_stamp['odom']
            odometry_step(last_odom_xyz,v[:3],delta)
            if delta>1:
                state['odometry_output_gaps']=state.get('odometry_output_gaps',0)+1
                state['last_odometry_output_gap_s']=delta
        last_odom_xyz=v[:3].copy()
        last_stamp['odom']=current
        state['odometry_messages']+=1
        state['odometry_received_unix_s']=time.time()
        state['odometry']={'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
                            'stamp_s':current,'xyz':v[:3].tolist(),'xyzw':v[3:].tolist()}

    def imu(msg):
        state['latest_imu']={
            'received_unix_s':time.time(),'stamp_s':stamp(msg),
            'frame_id':msg.header.frame_id,
            'linear_acceleration':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]}
        if len(initial_up)>=200:return
        a=msg.linear_acceleration
        up=np.array([a.x,a.y,a.z],float)
        if not np.isfinite(up).all() or not 7<np.linalg.norm(up)<12:return
        # Express specific force in this session's camera_init using current odometry.
        if state['odometry'] is not None:
            q=np.asarray(state['odometry']['xyzw'])
            up=up+2*np.cross(q[:3],np.cross(q[:3],up)+q[3]*up)
        initial_up.append(up)
        if len(initial_up)>=50:
            mean=np.mean(initial_up,axis=0)
            state['initial_up_current_xyz']=(mean/np.linalg.norm(mean)).tolist()
            state['initial_up_samples']=len(initial_up)

    node.create_subscription(PointCloud2,'/cloud_registered',cloud,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/Odometry',odom,qos_profile_sensor_data)
    node.create_subscription(Imu,'/g1_fastlio/imu_livox_mid360_upright',imu,qos_profile_sensor_data)

    def stop(*_):
        nonlocal keep_running
        keep_running=False
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    last_write=0.
    try:
        while keep_running and rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()-last_write>.05:
                try:
                    relay=json.loads((args.directory/args.session/'source-state.json').read_text())
                    state['sensor_transport']={key:relay.get(key) for key in
                        ('running','ready','heartbeat_unix_s','source_stamp','backlog_growth_s','transport','error')}
                except (OSError,ValueError):state['sensor_transport']=None
                state['heartbeat_unix_s']=time.time()
                atomic_json(args.directory/'raw-state.json',state)
                last_write=time.monotonic()
    except Exception as exc:
        state['error']=str(exc)
        raise
    finally:
        state.update(running=False,heartbeat_unix_s=time.time())
        atomic_json(args.directory/'raw-state.json',state)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
