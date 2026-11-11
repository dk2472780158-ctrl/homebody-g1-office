"""Automatic geometric bootstrap for AMCL; never an operator pose or UI source.

AMCL's uniform particles can collapse onto a symmetric wrong room corner.
Refine all distinct coarse 3-D hypotheses before comparing their scores, then
offer a unique geometrically verified seed. AMCL must still verify new scans.
"""
import json,math,os,signal,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from office_live_geometry import HistoricalMatcher,gravity_alignment,estimate_floor,structural,voxel
from geometry import seed_ready
stage=Path(__file__).resolve().parent;live=Path('/workspace/runtime/live-readonly')
matcher=HistoricalMatcher(stage/'g1_fastlio.historical-office.pcd')
office=np.asarray(json.loads((stage/'historical-office-registration.json').read_text())['T_office_map_from_provided_pcd'])
running=True
def stop(*_):
    global running
    running=False
signal.signal(signal.SIGTERM,stop)
signal.signal(signal.SIGINT,stop)
def atomic(value):
    p=stage/'seed.tmp';p.write_text(json.dumps(value,allow_nan=False));p.replace(stage/'seed.json')
while running:
    try:
        raw=json.loads((live/'raw-state.json').read_text());session=raw['session_id']
        status=json.loads((stage/'amcl-state.json').read_text()) if (stage/'amcl-state.json').exists() else {}
        if status.get('status')=='live' and status.get('session_id')==session and time.time()-status['updated_unix_s']<2:
            time.sleep(1);continue
        with np.load(live/'clouds.npz',allow_pickle=False) as data:
            if str(data['session_id'])!=session or time.time()-float(data['received_unix_s'])>.8:raise ValueError('Stale source')
            recent=data['recent_xyz'].copy();points=data['xyz'].copy()
        align=gravity_alignment(raw['initial_up_current_xyz'])
        level=points@align[:3,:3].T
        align[2,3]=matcher.floor['height_m']-estimate_floor(level)['height_m']
        if abs(align[2,3])>1.5:raise ValueError('Floor mismatch')
        aligned=points@align[:3,:3].T+align[:3,3]
        matcher.last_search={}
        try:matcher._global_match_aligned(aligned)
        except ValueError:pass
        candidates=matcher.last_search.get('planar_candidates',[])
        source=voxel(structural(aligned),.1)
        fine=source[np.random.default_rng(20261110).choice(len(source),min(4000,len(source)),replace=False)]
        refined=[]
        for candidate in candidates:
            if not running:break
            x,y,yaw=candidate['xyyaw'];T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();T[:3,3]=[x,y,0]
            try:T=matcher.refine(fine,T)
            except ValueError:continue
            if math.degrees(math.acos(np.clip(T[2,2],-1,1)))>5:continue
            transform=T@align;world=office@transform
            if not -5.3<world[0,3]<3.6 or not -3.6<world[1,3]<1.9:continue
            structure=matcher.current_structure(recent,transform)
            if len(structure)<250:continue
            recent_q=matcher.metrics(structure,transform)
            q=matcher.metrics(fine,T)
            refined.append((q['mean_clipped_m'],transform,{**q,'recent':recent_q}))
        refined.sort(key=lambda row:row[0])
        if not refined:raise ValueError('No geometric seed')
        score,best,q=refined[0]
        alternate=next((v for v,T,_ in refined if np.linalg.norm(T[:2,3]-best[:2,3])>.8 or abs(np.angle(np.exp(1j*(np.arctan2(T[1,0],T[0,0])-np.arctan2(best[1,0],best[0,0])))))>np.deg2rad(18)),None)
        recent_q=q['recent']
        valid=seed_ready(q,alternate)
        current=json.loads((live/'raw-state.json').read_text())
        if current['session_id']!=session:raise ValueError('Session changed')
        result=dict(session_id=session,updated_unix_s=time.time(),valid=valid,quality=q,
            alternate_score_m=alternate,best_score_m=score,refined_hypotheses=len(refined),
            T_office_from_current=(office@best).tolist() if valid else None,
            candidates=[dict(score=v,T=(office@T).tolist(),quality=q) for v,T,q in refined])
        atomic(result);print(json.dumps(result),flush=True)
    except (ValueError,OSError,KeyError) as exc:print(str(exc),flush=True)
    until=time.monotonic()+12
    while running and time.monotonic()<until:time.sleep(.2)
