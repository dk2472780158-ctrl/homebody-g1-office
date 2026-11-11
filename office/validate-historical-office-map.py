"""Validate historical PCD against the already recorded 2026-11-05 sensor slices."""
import base64
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'provided-pcd-match-20261106'
historical=Path('input/g1_fastlio.pcd')
recording=ROOT/'offline-sensors-20261105/raw.jsonl.gz'
old_validation=json.loads((ROOT/'map-match-20261105/temporal-validation.json').read_text())
new_registration=json.loads((OUT/'refined-registration.json').read_text(encoding='utf-8'))
T_office_from_history=np.array(new_registration['T_office_map_from_provided_pcd'])
x,y,yaw=old_validation['fixed_map_candidate_xyyaw']
T_office_from_recording=np.eye(4)
T_office_from_recording[:2,:2]=[[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]
T_office_from_recording[:2,3]=[x,y]
T_history_from_recording=np.linalg.inv(T_office_from_history)@T_office_from_recording
normalizer_R=np.array(old_validation['scan_to_aligned_rotation'])
normalizer_t=np.array(old_validation['scan_to_aligned_shift'])
with historical.open('rb') as f:
    while not f.readline().startswith(b'DATA'):
        pass
    map_points=np.frombuffer(f.read(),dtype='<f4').reshape(-1,8)[:,:3].copy()
target=map_points[(map_points[:,2]>-1.05)&(map_points[:,2]<.75)]
target=target[np.unique(np.floor(target/.07).astype(np.int32),axis=0,return_index=True)[1]]
tree=cKDTree(target)
indices=[0,50,99,150,195,198]
raw_clouds={}
with gzip.open(recording,'rt') as f:
    index=0
    for line in f:
        if 'cloud_livox_mid360' not in line:
            continue
        if index in indices:
            m=json.loads(line)['msg']
            data=base64.b64decode(m['data'])
            raw_clouds[index]=np.ndarray((m['width']*m['height'],3),dtype='<f4',buffer=data,strides=(m['point_step'],4)).copy()
        index+=1
if len(raw_clouds)!=len(indices):
    raise ValueError(f'Expected {len(indices)} frames; found {len(raw_clouds)}')
results=[]
representative=None
for index in indices:
    normalized=raw_clouds[index]@normalizer_R+normalizer_t
    selected=normalized[(normalized[:,2]>-.95)&(normalized[:,2]<.6)]
    selected=selected[np.linalg.norm(selected[:,:2],axis=1)>.6]
    selected=selected[np.unique(np.floor(selected/.12).astype(np.int32),axis=0,return_index=True)[1]]
    moved=selected@T_history_from_recording[:3,:3].T+T_history_from_recording[:3,3]
    distances=tree.query(moved)[0]
    results.append({'scan_index':index,'points_used':len(selected),
                    'median_nearest_neighbor_m':float(np.median(distances)),
                    'mean_clipped_at_0_65m':float(np.minimum(distances,.65).mean()),
                    'within_0_2m_fraction':float(np.mean(distances<.2)),
                    'within_0_4m_fraction':float(np.mean(distances<.4))})
    if index==99:
        representative=moved
fig,ax=plt.subplots(figsize=(9,6),layout='constrained')
ax.scatter(target[::3,0],target[::3,1],s=.5,color='#343434',alpha=.45,label='Historical office PCD')
ax.scatter(representative[:,0],representative[:,1],s=2,color='#e68016',alpha=.7,label='Older sensor slice; composed transform')
ax.set(xlabel='Historical map X (m)',ylabel='Historical map Y (m)',
       title='Offline cross-check from 2026-11-05 recording; not current localization')
ax.set_aspect('equal');ax.legend();ax.grid(alpha=.15)
fig.savefig(OUT/'older-scan-in-historical-map.png',dpi=160)
report={'scope':'old recording checked against user-confirmed historical office PCD; offline only',
        'historical_map_sha256':hashlib.sha256(historical.read_bytes()).hexdigest(),
        'older_recording_sha256':hashlib.sha256(recording.read_bytes()).hexdigest(),
        'coordinate_chain':'T_history_from_recording = inverse(T_office_from_history) @ T_office_from_recording',
        'T_history_from_recording':T_history_from_recording.tolist(),
        'scan_checks':results,'current_pose_validated':False,'navigation_ready':False,
        'limitations':['These six scans come from the same old recording',
                       'The older recording frame differs from any newly started FAST-LIO camera_init',
                       'No current robot pose or base calibration has been inferred']}
(OUT/'older-recording-crosscheck.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({'frames':len(results),'within_0_2m_range':[min(r['within_0_2m_fraction'] for r in results),max(r['within_0_2m_fraction'] for r in results)],
                  'current_pose_validated':False},indent=2))
