"""Read AMCL/sensor snapshots only; no ROS or robot interfaces."""
import json,time
from pathlib import Path
stage=Path(__file__).resolve().parent;live=stage.parent/'runtime/live-readonly'
samples=[]
for i in range(30):
    now=time.time()
    state=json.loads((stage/'amcl-state.json').read_text());raw=json.loads((live/'raw-state.json').read_text())
    seed=json.loads((stage/'seed.json').read_text()) if (stage/'seed.json').exists() else {}
    q=state.get('quality') or {}
    value=dict(at=now,status=state.get('status'),message=state.get('message'),pose=state.get('pose'),
        session=state.get('session_id'),cloud_age=now-float(raw.get('cloud_received_unix_s') or 0),
        odom_age=now-float(raw.get('odometry_received_unix_s') or 0),
        scan_fraction=q.get('within_0_2m_fraction'),q3=q.get('historical_3d'),
        mass=q.get('dominant_particle_mass'),seed_valid=seed.get('valid'),
        seed_best=seed.get('best_score_m'),seed_alternate=seed.get('alternate_score_m'),seed_quality=seed.get('quality'))
    samples.append(value)
    if i%10==0:print(json.dumps(value,ensure_ascii=False),flush=True)
    time.sleep(1)
(stage/'observation.json').write_text(json.dumps(samples,ensure_ascii=False,indent=2))
print(json.dumps(dict(samples=len(samples),live=sum(v['status']=='live' for v in samples),
    sessions=len({v['session'] for v in samples}),last=samples[-1]),ensure_ascii=False),flush=True)
