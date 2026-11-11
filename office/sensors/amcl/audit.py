import json,math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import scan_quality,scan_ranges,planar
stage=Path(__file__).resolve().parent;live=stage.parent/'runtime/live-readonly'
state=json.loads((stage/'amcl-state.json').read_text());seed=json.loads((stage/'seed.json').read_text());raw=json.loads((live/'raw-state.json').read_text())
seed_T=np.asarray(seed['T_office_from_current']);level=np.linalg.inv(planar(*seed_T[:2,3],math.atan2(seed_T[1,0],seed_T[0,0])))@seed_T
body=np.eye(4);body[:3,:3]=Rotation.from_quat(raw['odometry']['xyzw']).as_matrix();body[:3,3]=raw['odometry']['xyz']
lb=level@body;odom=planar(*lb[:2,3],math.atan2(lb[1,0],lb[0,0]))
mapbase=planar(*state['quality']['candidate_xyyaw'][:2],math.radians(state['quality']['candidate_xyyaw'][2]))
with np.load(stage/'real-office-navigation.npz') as d:occ=d['occupancy'].astype(bool);T=d['T_map_px']
result={}
with np.load(live/'clouds.npz') as d:
    for key in ('latest_xyz','recent_xyz','xyz'):
        points=d[key]@level[:3,:3].T+level[:3,3]
        ranges=scan_ranges(points,odom[:2,3],math.atan2(odom[1,0],odom[0,0]),-1.2601173454149428)
        result[key]=scan_quality(ranges,mapbase,occ,T)
print(json.dumps(dict(candidate=state['quality']['candidate_xyyaw'],quality=result)))
