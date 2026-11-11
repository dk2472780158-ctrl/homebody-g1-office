"""Ensure this isolated localization process group exists, without robot I/O."""
import json,subprocess
from pathlib import Path
stage=Path(__file__).resolve().parent
found=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:args=(proc/'cmdline').read_bytes().split(b'\0')
    except OSError:continue
    if str(stage/'run.sh').encode() in args:found.append(int(proc.name))
if not found:
    with (stage/'launcher.log').open('a') as log:
        process=subprocess.Popen(['bash',str(stage/'run.sh')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    (stage/'launcher.pid').write_text(str(process.pid));found=[process.pid]
print(json.dumps(dict(running_pids=found,motion_started=False)))
