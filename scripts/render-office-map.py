"""Render the shipped office map and an offline route example."""
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'office'))
from g1_readonly_geometry import StaticRoute
from office_landmarks import approach_points,load_landmarks
import json

office=ROOT/'office';planner=StaticRoute(office/'real-office-navigation.npz')
points=approach_points(office,json.loads((office/'office-waypoints.json').read_text(encoding='utf-8'))['points'])
T=planner.T;h,w=planner.occupied.shape;res=planner.resolution
extent=[T[0,2]-res/2,T[0,2]+res*(w-.5),T[1,2]-res*(h-.5),T[1,2]+res/2]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11})
fig,ax=plt.subplots(figsize=(11.2,7.3),facecolor='#f7f9fc')
ax.set_facecolor('#f7f9fc')
ax.imshow(planner.occupied,extent=extent,origin='upper',cmap=matplotlib.colors.ListedColormap(['#edf2f7','#31445a']),interpolation='nearest')
labels={'桌子1':'Table 1','桌子2':'Table 2','架子':'Shelf','窗口':'Window','门口':'Entrance','房间中央':'Room center'}
landmarks=load_landmarks(office)
for label,text in labels.items():
    xy=np.asarray(points[label])
    if label in landmarks:
        anchor=np.asarray(landmarks[label]['anchor_xy_m'])
        ax.plot(*anchor,'s',color='#7890a2',markersize=5)
        ax.plot([anchor[0],xy[0]],[anchor[1],xy[1]],color='#94a3b8',ls=':',lw=1)
    ax.scatter(*xy,s=95 if label=='房间中央' else 55,color='#f2ab3c' if label=='房间中央' else '#16836b',edgecolor='white',linewidth=1.5,zorder=5)
    offset=(-8,15) if label=='房间中央' else (9,10)
    ax.annotate(text,xy,xytext=offset,textcoords='offset points',fontweight='bold',color='#20354c',zorder=6)
start=np.array([2.62,-2.60]);route,detail=planner.navigation_route(start,points['房间中央'],yaw=0.)
ax.plot(route[:,0],route[:,1],lw=2.8,color='#3478c5',label='Offline route example',zorder=4)
ax.scatter(*start,s=55,color='#3478c5',edgecolor='white',zorder=5)
ax.annotate('Example start',start,xytext=(-105,-22),textcoords='offset points',color='#3478c5')
ax.annotate('N',xy=(3.85,1.3),xytext=(3.85,.5),arrowprops={'arrowstyle':'->','color':'#20354c','lw':2},ha='center',fontweight='bold',color='#20354c')
ax.set_xlim(-5.25,4.15);ax.set_ylim(-3.95,2.2);ax.set_aspect('equal')
ax.set_xlabel('East / x (m)');ax.set_ylabel('North / y (m)')
ax.set_title('Office map  /  named places and room-center navigation',loc='left',fontweight='bold',pad=18,color='#20354c')
ax.grid(alpha=.1);ax.legend(loc='lower left',frameon=True,facecolor='white',edgecolor='none')
for spine in ax.spines.values():spine.set_color('#d4dde7')
fig.text(.13,.025,'Historical occupancy map · 5 cm grid · not a recorded robot trajectory',color='#62748b',fontsize=10)
fig.subplots_adjust(bottom=.12,top=.90,left=.09,right=.97)
output=ROOT/'docs/images/office-map.png';output.parent.mkdir(exist_ok=True,parents=True)
fig.savefig(output,dpi=180,facecolor=fig.get_facecolor());plt.close(fig)
print(output.relative_to(ROOT))
