"""Build a bounded, reversible correction for the operator-removed west object."""
import hb_config
import ast,hashlib,io,json,re,shutil,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image,ImageDraw
from g1_readonly_geometry import StaticRoute

ROOT=Path(hb_config.expand('${HB_OFFICE_ROOT}'))
OUT=ROOT/'west-obstacle-removal-20261110'
OUT.mkdir(exist_ok=True)
BOUNDS=(-4.75,-3.10,-1.575,.125)


def inside(xy):
    x,y=np.asarray(xy).T
    a,b,c,d=BOUNDS
    return (x>=a)&(x<=b)&(y>=c)&(y<=d)


def save_npz(relative,fields):
    dest=OUT/relative;dest.parent.mkdir(parents=True,exist_ok=True)
    with dest.open('wb') as stream:np.savez_compressed(stream,**fields)


def read_npz(relative):
    with np.load(ROOT/relative,allow_pickle=False) as data:return {k:data[k].copy() for k in data.files}


report=dict(created_unix_s=time.time(),operator_request='Remove the west inward protrusion; operator confirms the object is no longer present',
    office_xy_bounds_m=BOUNDS,live_obstacle_filter_changed=False,files={},motion_sent=False)
for relative in ('real-office-navigation.npz','office-map-proxy-scene/map.npz'):
    fields=read_npz(relative);old=fields['occupancy'].astype(bool);t=fields['T_map_px']
    rows,cols=np.indices(old.shape)
    world=(t@np.stack([cols.ravel(),rows.ravel(),np.ones(old.size)]))[:2].T
    mask=inside(world).reshape(old.shape)
    new=old.copy();new[mask]=False
    assert np.array_equal(new[~mask],old[~mask])
    assert not np.any(new&~old)
    assert np.array_equal(old[:,0],new[:,0]) and np.array_equal(old[:,-1],new[:,-1])
    fields['occupancy']=new;save_npz(relative,fields)
    report['files'][relative]=dict(removed_cells=int((old&~new).sum()),outside_region_unchanged=True)
    if relative.startswith('real-'):
        before,after=old,new
        transform=t

# Historical obstacle display and its exported grid use independent raster geometry.
relative='office-cloud-slice/cloud-slice.npz'
fields=read_npz(relative);xyz=fields['xyz'];floor=float(fields['floor_m'])
fields['xyz']=xyz[~inside(xyz[:,:2])]
old=fields['occupancy'].astype(bool);rows,cols=np.indices(old.shape)
xy=fields['origin_xy_m']+(np.stack([cols.ravel(),rows.ravel()],axis=1)+.5)*float(fields['resolution_m'])
mask=inside(xy).reshape(old.shape);new=old.copy();new[mask]=False
assert np.array_equal(old[~mask],new[~mask])
fields['occupancy']=new;save_npz(relative,fields)
image=np.where(new,0,254).astype(np.uint8)[::-1]
(OUT/'office-cloud-slice/map.pgm').write_bytes(f'P5\n{new.shape[1]} {new.shape[0]}\n255\n'.encode()+image.tobytes())
report['files'][relative]=dict(removed_points=int(len(xyz)-len(fields['xyz'])),removed_cells=int((old&~new).sum()))

# Keep the original user-supplied PCD untouched; the deployed historical copy
# receives the same bounded above-floor correction and is backed up on deploy.
relative='g1_fastlio.historical-office.pcd'
with (ROOT/relative).open('rb') as stream:
    header=[]
    while True:
        line=stream.readline()
        if not line:raise ValueError('Incomplete PCD header')
        header.append(line.decode('ascii'))
        if line.startswith(b'DATA'):break
    assert header[-1].strip()=='DATA binary'
    assert any(line.strip()=='FIELDS x y z intensity normal_x normal_y normal_z curvature' for line in header)
    data=np.frombuffer(stream.read(),dtype='<f4').reshape(-1,8)
t=np.asarray(json.loads((ROOT/'historical-office-registration.json').read_text())['T_office_map_from_provided_pcd'])
world=data[:,:3]@t[:3,:3].T+t[:3,3]
remove=inside(world[:,:2])&(world[:,2]>floor+.10)
kept=data[~remove]
header=[re.sub(r'^(WIDTH|POINTS)\s+\d+',lambda m:f'{m[1]} {len(kept)}',line) for line in header]
assert any(line.strip()=='HEIGHT 1' for line in header)
(OUT/relative).write_bytes(''.join(header).encode('ascii')+kept.tobytes())
report['files'][relative]=dict(removed_points=int(remove.sum()),remaining_points=len(kept),floor_retained=True)

# Rebuild only occupancy-derived collision boxes, keeping the rest of the scene.
relative='office-map-proxy-scene/scene.xml'
scene=ET.parse(ROOT/relative);worldbody=scene.getroot().find('worldbody')
geoms=[g for g in worldbody.findall('geom') if g.get('name','').startswith('structure__map_')]
assert geoms
template=dict(geoms[0].attrib)
for g in geoms:worldbody.remove(g)
source=ast.parse((ROOT/'office-sim-build.py').read_text())
fn=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='rectangles')
ns={'np':np};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<rectangles>','exec'),ns)
sim=read_npz('office-map-proxy-scene/map.npz')
with np.load(OUT/'office-map-proxy-scene/map.npz') as d:sim['occupancy']=d['occupancy'].copy()
t=sim['T_map_px'];res=float(t[0,0]);rects=ns['rectangles'](sim['occupancy'])
for i,(left,right,top,bottom) in enumerate(rects):
    a=t@np.array([left-.5,bottom-.5,1]);b=t@np.array([right-.5,top-.5,1])
    attrs={**template,'name':f'structure__map_{i:04d}',
        'pos':f'{(a[0]+b[0])/2:.5f} {(a[1]+b[1])/2:.5f} 1.1',
        'size':f'{(b[0]-a[0])/2:.5f} {(b[1]-a[1])/2:.5f} 1.1'}
    ET.SubElement(worldbody,'geom',attrs)
scene.write(OUT/relative,encoding='unicode')
report['files'][relative]=dict(before_boxes=len(geoms),after_boxes=len(rects))

view=json.loads((ROOT/'sensors/runtime/live-readonly/real-view-state.json').read_text())
pose=np.asarray(view['base']['T_office_from_pelvis']) if view.get('base') else None
planner=StaticRoute(OUT/'real-office-navigation.npz')
if pose is not None:
    report['current_xy_m']=pose[:2,3].tolist()
    report['clearance_before_m']=StaticRoute(ROOT/'real-office-navigation.npz').clearance_at(pose[:2,3])
    report['clearance_after_m']=planner.clearance_at(pose[:2,3])

# Render the exact grid change in office coordinates without changing live data.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,2,figsize=(14,6),sharex=True,sharey=True)
for ax,grid,title in zip(axes,(before,after),('Before','After: removed west protrusion')):
    rows,cols=np.nonzero(grid);xy=(transform@np.stack([cols,rows,np.ones(len(rows))]))[:2].T
    ax.scatter(xy[:,0],xy[:,1],s=3,c='#556a7b')
    a,b,c,d=BOUNDS
    ax.plot([a,b,b,a,a],[c,c,d,d,c],c='#dd8132')
    if pose is not None:ax.scatter(*pose[:2,3],s=45,c='#dab72f',label='Current base')
    ax.set(xlim=(-5.6,3.9),ylim=(-3.8,2.1),title=title,xlabel='East / x (m)');ax.set_aspect('equal');ax.grid(alpha=.3)
axes[0].set_ylabel('North / y (m)')
fig.tight_layout();fig.savefig(OUT/'before-after.png',dpi=150);plt.close(fig)
report['source_hashes']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in report['files']}
(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False))
