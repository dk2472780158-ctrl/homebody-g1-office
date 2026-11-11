"""Fuse surveyed PCD obstacles into the existing office frame; offline only."""
import json
from pathlib import Path
import numpy as np
from scipy import ndimage
from office_live_geometry import HistoricalMatcher
from g1_readonly_geometry import StaticRoute

root=Path(__file__).resolve().parent
pcd=Path('input/g1_fastlio(1).pcd')
history=HistoricalMatcher(pcd)
T=np.array(json.loads((root/'provided-pcd-match-20261106/refined-registration.json').read_text())['T_office_map_from_provided_pcd'])
points=history.target@T[:3,:3].T+T[:3,3]
floor=float((T@[0,0,history.floor['height_m'],1])[2])
points=points[(points[:,2]>floor+.15)&(points[:,2]<floor+1.8)]
with np.load(root/'office-map-runtime.npz') as data:
    fields={k:data[k] for k in data.files}
grid=fields['occupancy'].astype(bool);M=fields['T_map_px'];inverse=np.linalg.inv(M)
cells=np.rint((inverse@np.column_stack([points[:,:2],np.ones(len(points))]).T)[:2]).astype(int)
good=(cells[0]>=0)&(cells[0]<grid.shape[1])&(cells[1]>=0)&(cells[1]<grid.shape[0])
counts=np.zeros_like(grid,dtype=int);np.add.at(counts,(cells[1,good],cells[0,good]),1)
# Require multiple returns locally so isolated lidar speckles do not become walls.
support=ndimage.convolve((counts>0).astype(int),np.ones((3,3),int),mode='constant')
added=(counts>0)&(support>=3)
fields['occupancy']=grid|added
np.savez_compressed(root/'real-office-navigation.npz',**fields)
planner=StaticRoute(root/'real-office-navigation.npz')
saved=json.loads((root/'office-waypoints.json').read_text(encoding='utf-8'))
door=np.array([2.75,-2.575])
if planner.clearance_at(door)<=.55:raise ValueError('Requested southeast door point has insufficient clearance')
saved['points']['门口']=door.tolist()
saved['note']='Compass goals retained; door moved southeast at operator request; real routes use fused PCD occupancy.'
landmarks=json.loads((root/'office-landmarks.json').read_text(encoding='utf-8'))
rows,cols=np.nonzero(planner.clearance>.62)
free_xy=(planner.T@np.stack([cols,rows,np.ones(len(rows))]))[:2].T
for label,point in saved['points'].items():
    if label=='门口' or planner.clearance_at(point)>.55:continue
    distance=np.linalg.norm(free_xy-np.array(point),axis=1)
    # Compass aliases remain in the same rectangular office, with nearby free goals.
    valid=(free_xy[:,0]>-4.7)&(free_xy[:,0]<3.1)&(free_xy[:,1]>-3.1)&(free_xy[:,1]<1.3)
    distance[~valid]=np.inf
    index=int(np.argmin(distance))
    if distance[index]>.8:raise ValueError('No nearby free compass goal: '+label)
    saved['points'][label]=free_xy[index].tolist()
(root/'office-waypoints.json').write_text(json.dumps(saved,ensure_ascii=False,indent=2),encoding='utf-8')
for label,item in landmarks['landmarks'].items():
    original=np.array(item['approach_xy_m'])
    if planner.clearance_at(original)<.6:
        index=int(np.argmin(np.linalg.norm(free_xy-original,axis=1)))
        item['approach_xy_m']=free_xy[index].tolist()
        item['approach_cell_col_row']=[int(cols[index]),int(rows[index])]
    item['approach_static_clearance_m']=planner.clearance_at(item['approach_xy_m'])
(root/'office-landmarks.json').write_text(json.dumps(landmarks,ensure_ascii=False,indent=2),encoding='utf-8')
report={'pcd_obstacle_cells_added':int(np.count_nonzero(added&~grid)),
        'door_xy_m':door.tolist(),'door_clearance_m':planner.clearance_at(door),
        'floor_m':floor,'scope':'offline map fusion; no robot commands'}
(root/'real-navigation-map-report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
