"""Host service entry point; synchronize maps and start only AMCL processes."""
import hb_config
import hashlib,shutil,subprocess
from pathlib import Path
root=Path(hb_config.expand('${HB_OFFICE_ROOT}'))
stage=root/'sensors/amcl'
names=('real-office-navigation.npz','historical-office-registration.json',
       'g1_fastlio.historical-office.pcd','office_live_geometry.py','office-live-map-worker.py')
# The offline map's floor report is shipped with this adapter; older deployments
# do not have a root copy. Keep the shipped floor rather than failing startup.
if (root/'real-navigation-map-report.json').exists():names=(*names,'real-navigation-map-report.json')
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None
changed=any(digest(root/name)!=digest(stage/name) for name in names)
if changed:
    (root/'real-motion-stop').touch()
    subprocess.run(['docker','exec','homebody-office-sensors','/usr/bin/python3','/workspace/amcl/stop.py'],check=True)
    for name in names:shutil.copy2(root/name,stage/name)
    for name in ('seed.json','amcl-state.json'):
        if (stage/name).exists():(stage/name).unlink()
subprocess.run(['docker','exec','homebody-office-sensors','/usr/bin/python3','/workspace/amcl/manage.py'],check=True)
