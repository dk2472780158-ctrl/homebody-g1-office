"""Stop only this AMCL shadow group, never the sensor/localization container."""
import os,signal,time
from pathlib import Path
stage='/workspace/amcl'
targets=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=(proc/'cmdline').read_bytes().split(b'\0')
    except OSError:continue
    if (stage+'/run.sh').encode() in args:
        targets.append((int(proc.name),proc));os.kill(int(proc.name),signal.SIGTERM)
for pid,proc in targets:
    until=time.monotonic()+10
    while proc.exists() and time.monotonic()<until:
        try:
            if (proc/'stat').read_text().split(') ')[1].startswith('Z'):break
        except OSError:break
        time.sleep(.1)
    else:
        if proc.exists():raise RuntimeError('AMCL shadow did not exit')
print('Stopped AMCL shadow group only')
