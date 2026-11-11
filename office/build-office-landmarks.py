"""Create reviewed semantic anchors and nearby free approach points from map cells."""
import json,os
from pathlib import Path
import numpy as np
from scipy import ndimage
from PIL import Image,ImageDraw,ImageFont

root=Path(__file__).parent
with np.load(root/'office-map-runtime.npz',allow_pickle=False) as data:
    occupied=data['occupancy'].astype(bool);T=data['T_map_px']
clearance=np.maximum(0,ndimage.distance_transform_edt(np.pad(~occupied,1,constant_values=False))[1:-1,1:-1]*.05-.05/np.sqrt(2))
# The tables are separate north/northeast PCD structures, not two parts of one.
# Table 2 is visible in the PCD near office (0.575, 1.175), although absent
# from this older occupancy grid; keep its approach south of its cloud extent.
# The west-side obstacle is deliberately unnamed.
items=[('桌子1',(148,48),(144,72)),('桌子2',(200,48),(200,78)),
       ('架子',(249,57),(231,60)),('窗口',(91,130),(109,114))]
result={}
rows,cols=np.nonzero(clearance>.58)
for label,anchor,preferred in items:
    distance=(cols-preferred[0])**2+(rows-preferred[1])**2
    i=int(np.argmin(distance));goal=(int(cols[i]),int(rows[i]))
    xy=lambda cell:(T@[*cell,1])[:2].tolist()
    result[label]={'anchor_xy_m':xy(anchor),'approach_xy_m':xy(goal),
        'anchor_cell_col_row':list(anchor),'approach_cell_col_row':list(goal),
        'approach_static_clearance_m':float(clearance[goal[1],goal[0]])}
(root/'office-landmarks.json').write_text(json.dumps({'landmarks':result,
    'source':'operator corrected: table1 north left; table2 separate northeast PCD structure left of shelf; west obstacle unnamed; shelf/window/door unchanged; free-space approach points'},
    ensure_ascii=False,indent=2),encoding='utf-8')
im=Image.fromarray(np.where(occupied[...,None],[47,62,77],[230,236,241]).astype('uint8'))
im=im.crop((75,30,270,149)).resize((975,595))
draw=ImageDraw.Draw(im)
font_candidates=[os.environ.get('HB_MAP_FONT',''),
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    str(Path(os.environ.get('WINDIR','C:/Windows'))/'Fonts/msyh.ttc')]
font_path=next((p for p in font_candidates if p and Path(p).is_file()),None)
if font_path is None:raise RuntimeError('Set HB_MAP_FONT to a TrueType/OpenType font; CJK coverage is recommended')
font=ImageFont.truetype(font_path,24)
for label,item in result.items():
    x,y=item['anchor_cell_col_row'];x=(x-75)*5;y=(y-30)*5
    gx,gy=item['approach_cell_col_row'];gx=(gx-75)*5;gy=(gy-30)*5
    draw.line((x,y,gx,gy),fill='#da8b28',width=2)
    draw.ellipse((gx-5,gy-5,gx+5,gy+5),fill='#20a27f')
    offset=(-65,-30) if label=='桌子1' else (8,-30) if label=='桌子2' else (8,-20)
    draw.text((x+offset[0],y+offset[1]),label,font=font,fill='#ef7d19',stroke_width=2,stroke_fill='white')
im.save(root/'office-named-landmarks.png')
print(json.dumps(result,ensure_ascii=False))
