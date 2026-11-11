"""Test a map candidate against separate scans from the same 20-second capture."""
import base64
import gzip
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.optimize import minimize
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parent
OUT = ROOT/'map-match-20261105'
with Path('input/test.pcd').open('rb') as f:
    while not f.readline().startswith(b'DATA'):
        pass
    pcd = np.frombuffer(f.read(),dtype='<f4').reshape(-1,8)[:,:3]
target = pcd[(pcd[:,2] > -.95) & (pcd[:,2] < .6)]
target = target[np.unique(np.floor(target/.1).astype(int),axis=0,return_index=True)[1]]
tree = cKDTree(target)
pgm = np.asarray(Image.open('input/map(1).pgm'))
pgm_distance = ndimage.distance_transform_edt(~(pgm < 100))*.05
candidate = np.array(json.loads((OUT/'candidate-registration.json').read_text())['candidates'][0]['xyyaw'])
with gzip.open(ROOT/'offline-sensors-20261105/raw.jsonl.gz','rt') as f:
    clouds = [json.loads(line)['msg'] for line in f if 'cloud_livox_mid360' in line]

def xyz(m):
    data=base64.b64decode(m['data'])
    return np.ndarray((m['width']*m['height'],3),dtype='<f4',buffer=data,
                      strides=(m['point_step'],4)).copy()

# Pair scan 195 with its offline processed output by equal length and matching point order.
paired = xyz(clouds[195])
aligned = np.load(ROOT/'offline-sensors-20261105/registered_scan_aligned.npz')['xyz']
if len(paired) != len(aligned):
    raise ValueError('Expected paired raw/processed scan lengths differ')
ac,bc=paired.mean(axis=0),aligned.mean(axis=0)
u,_,vt=np.linalg.svd((paired-ac).T@(aligned-bc))
rotation=u@vt
if np.linalg.det(rotation)<0:
    u[:,-1]*=-1
    rotation=u@vt
shift=bc-ac@rotation
pair_error=np.linalg.norm(paired@rotation+shift-aligned,axis=1)

def tr(points,p):
    c,s=np.cos(p[2]),np.sin(p[2])
    xy=points[:,:2] @ np.array([[c,s],[-s,c]])+p[:2]
    return np.column_stack([xy,points[:,2]])

def score(points,p):
    d=tree.query(tr(points,p),workers=-1)[0]
    return float(np.minimum(d,.65).mean())

results=[]
for idx in [0,50,99,150,195,198]:
    points=xyz(clouds[idx])@rotation+shift
    points=points[(points[:,2]>-.95)&(points[:,2]<.6)]
    points=points[np.linalg.norm(points[:,:2],axis=1)>.6]
    points=points[np.unique(np.floor(points/.12).astype(int),axis=0,return_index=True)[1]]
    if len(points)>2200:
        points=points[np.random.default_rng(100+idx).choice(len(points),2200,replace=False)]
    fixed=tr(points,candidate)
    d=tree.query(fixed,workers=-1)[0]
    col=(fixed[:,0]+9.45)/.05-.5
    row=pgm.shape[0]-.5-(fixed[:,1]+7.1)/.05
    dp=ndimage.map_coordinates(pgm_distance,[row,col],order=1,mode='constant',cval=1.)
    fit=minimize(lambda p: score(points,p),candidate,method='Nelder-Mead',
                 options={'maxiter':150,'xatol':.003,'fatol':.0005})
    delta=fit.x-candidate
    delta[2]=np.angle(np.exp(1j*delta[2]))
    results.append({'scan_index':idx,'points_used':len(points),
                    'fixed_candidate_3d_mean_clipped_m':float(np.minimum(d,.65).mean()),
                    'fixed_candidate_3d_within_0_2m_fraction':float(np.mean(d<.2)),
                    'fixed_candidate_pgm_within_0_2m_fraction':float(np.mean(dp<.2)),
                    'locally_refit_delta_xyyaw':delta.tolist(),
                    'locally_refit_3d_mean_clipped_m':float(fit.fun)})
report={'scope':'independent time slices of same recording; offline only; no robot pose set',
        'scan_to_aligned_pair_p95_error_m':float(np.percentile(pair_error,95)),
        'scan_to_aligned_rotation':rotation.tolist(),
        'scan_to_aligned_shift':shift.tolist(),
        'fixed_map_candidate_xyyaw':candidate.tolist(),
        'scan_checks':results,
        'confidence':'geometric candidate; still lacks independent physical pose and sensor-to-base calibration'}
(OUT/'temporal-validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
