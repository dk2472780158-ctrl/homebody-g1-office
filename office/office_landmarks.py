"""Named office objects and their approach points, separate from compass aliases."""
import json
from pathlib import Path

LANDMARK_PLACES={'go_table1':'桌子1','go_table2':'桌子2','go_shelf':'架子','go_window':'窗口'}
COMPASS_LABELS=frozenset(('北','东北','东','东南','南','西南','西','西北'))
# Midpoint of the main rectangular room's walls in office-map metres.
ROOM_CENTER_XY_M=(-.65,-.85)

def load_landmarks(root):
    path=Path(root)/'office-landmarks.json'
    return json.loads(path.read_text(encoding='utf-8'))['landmarks'] if path.exists() else {}

def visible_marks(root,points):
    marks={label:xy for label,xy in points.items() if label not in COMPASS_LABELS}
    marks.update({label:item['anchor_xy_m'] for label,item in load_landmarks(root).items()})
    return marks

def approach_points(root,points):
    return {'房间中央':list(ROOM_CENTER_XY_M),**points,
        **{label:item['approach_xy_m'] for label,item in load_landmarks(root).items()}}
