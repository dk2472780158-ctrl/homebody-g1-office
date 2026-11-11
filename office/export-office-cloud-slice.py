"""Offline height slicing and PGM/YAML export following g1_nav2's converter."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import ndimage
from office_live_geometry import estimate_floor,voxel
from g1_pointcloud_slice import height_slice,MIN_HEIGHT,MAX_HEIGHT

def export(pcd,registration,output):
    with pcd.open('rb') as stream:
        fields={}
        while True:
            line=stream.readline()
            if not line:raise ValueError('PCD header missing DATA')
            parts=line.decode('ascii').split()
            if not parts or parts[0].startswith('#'):continue
            fields[parts[0]]=parts[1:]
            if parts[0]=='DATA':break
        names=fields['FIELDS']
        if fields['DATA']!=['binary'] or any(v!='4' for v in fields['SIZE']) or any(v!='F' for v in fields['TYPE']):
            raise ValueError('Expected binary float32 PCD')
        if any(v!='1' for v in fields.get('COUNT',['1']*len(names))):raise ValueError('Unsupported PCD field counts')
        data=np.frombuffer(stream.read(),dtype='<f4').reshape(-1,len(names))
        points=data[:,[names.index(k) for k in ('x','y','z')]].copy()
    points=points[np.isfinite(points).all(axis=1)]
    floor=estimate_floor(points)['height_m']
    t=np.asarray(json.loads(registration.read_text())['T_office_map_from_provided_pcd'],float)
    world=points@t[:3,:3].T+t[:3,3]
    floor=float((t@[0,0,floor,1])[2]);cut=height_slice(world,floor)
    resolution=.05;origin=np.floor(cut[:,:2].min(axis=0)/resolution)*resolution
    size=np.ceil((cut[:,:2].max(axis=0)-origin)/resolution).astype(int)+1
    if np.prod(size)>100000000:raise ValueError('Map too large')
    cells=np.floor((cut[:,:2]-origin)/resolution).astype(int)
    occupied=np.zeros((size[1],size[0]),bool);occupied[cells[:,1],cells[:,0]]=True
    labels,n=ndimage.label(occupied,structure=np.ones((3,3),int))
    counts=np.bincount(labels.ravel());keep=counts>=4;keep[0]=False
    cleaned=keep[labels]
    output.mkdir(parents=True,exist_ok=True)
    image=np.where(cleaned,0,254).astype(np.uint8)[::-1]
    (output/'map.pgm').write_bytes(f'P5\n{size[0]} {size[1]}\n255\n'.encode()+image.tobytes())
    (output/'map.yaml').write_text(f'image: map.pgm\nmode: trinary\nresolution: {resolution:.6f}\norigin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    np.savez_compressed(output/'cloud-slice.npz',xyz=voxel(cut,.05),floor_m=floor,occupancy=cleaned,
                        origin_xy_m=origin,resolution_m=resolution)
    report=dict(input_points=len(points),sliced_points=len(cut),floor_m=floor,
        min_height_m=MIN_HEIGHT,max_height_m=MAX_HEIGHT,resolution_m=resolution,
        minimum_component_cells=4,occupied_cells=int(cleaned.sum()),raw_occupied_cells=int(occupied.sum()),
        scope='offline export; live obstacle guards keep all sliced returns without component deletion',
        reference='g1_nav2 99e977a src/fast_lio/src/pcd_to_nav2_map.cpp')
    (output/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--pcd',type=Path,required=True)
    parser.add_argument('--registration',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();export(args.pcd,args.registration,args.output)
