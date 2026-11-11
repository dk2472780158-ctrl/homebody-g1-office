"""Prepare AMCL inputs locally; no robot I/O, services or subprocesses."""
import argparse,json,shutil,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'office'))
import hb_config

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--publish-localization',action='store_true',
        help='Enable AMCL live-state export on the next sensor service start; does not start anything.')
    args=parser.parse_args()
    office=Path(hb_config.expand('${HB_OFFICE_ROOT}'));stage=office/'sensors/amcl'
    stage.mkdir(parents=True,exist_ok=True)
    names=('real-office-navigation.npz','historical-office-registration.json',
           'g1_fastlio.historical-office.pcd','office_live_geometry.py',
           'office-live-map-worker.py','real-navigation-map-report.json')
    # Avoid editing the input files of an active AMCL process.
    pidfile=stage/'launcher.pid'
    if pidfile.exists():
        try:
            pid=int(pidfile.read_text().strip())
            if Path(f'/proc/{pid}/cmdline').exists():raise RuntimeError('Stop the isolated localization service before preparing inputs')
        except ValueError:raise RuntimeError('Invalid localization PID file') from None
    for name in names:
        if not (office/name).is_file():raise FileNotFoundError(name)
    for name in names:shutil.copy2(office/name,stage/name)
    (office/'sensors/runtime/live-readonly').mkdir(parents=True,exist_ok=True)
    if args.publish_localization:(stage/'publish-live-state').touch()
    print(json.dumps(dict(copied=list(names),publish_localization=(stage/'publish-live-state').exists(),
                         motion_started=False,services_started=False),ensure_ascii=False))

if __name__=='__main__':main()
