"""Restart only the isolated observer's exact sensor-relay child."""
import os
import signal
from pathlib import Path

matches=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    try:args=(entry/'cmdline').read_bytes().split(b'\0')
    except (FileNotFoundError,PermissionError):continue
    if b'/workspace/rosbridge_to_ros2.py' in args:matches.append(int(entry.name))
if len(matches)>1:raise RuntimeError(f'Multiple relay children: {matches}')
for pid in matches:os.kill(pid,signal.SIGTERM)
print({'readonly_sensor_children_stopped':matches})
