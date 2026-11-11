"""Portable defaults for the office extension; secrets stay outside the checkout."""
import os
from pathlib import Path

def _repo():
    for root in Path(__file__).resolve().parents:
        if (root/'pyproject.toml').is_file():return root
    return Path(os.environ.get('HB_REPO_ROOT','/repo'))

ROOT=_repo()
_defaults={
    'HB_REPO_ROOT':str(ROOT),'HB_OFFICE_ROOT':str(ROOT/'office'),
    'HB_VENV':str(ROOT/'.venv'),'HB_USER_HOME':str(Path.home()),
    'HB_SECRET_DIR':str(Path.home()/'.config/homebody'),
    'HB_TELEOP_TOKEN_FILE':str(Path.home()/'.config/homebody/g1-teleop.token'),
    'HB_ROBOT_HOST':'robot.local','HB_WORKSTATION_HOST':'workstation.local',
    'HB_WORKSTATION_HOST_KEY_ALIAS':'workstation.local',
    'HB_ROSBRIDGE_SOURCE':'ws://127.0.0.1:19090',
}
for name,value in _defaults.items():os.environ.setdefault(name,value)

def expand(value):return os.path.expandvars(value)
