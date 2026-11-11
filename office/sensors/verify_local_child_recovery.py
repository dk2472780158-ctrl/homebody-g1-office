"""Fault injection limited to our container's own sensor relay process."""
import json
import os
import signal
import time
from pathlib import Path

live=Path('/workspace/runtime/live-readonly')
old_session=(live/'session-id').read_text().strip()
targets=[]
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:
        args=(proc/'cmdline').read_bytes().split(b'\0')
        if b'/workspace/rosbridge_to_ros2.py' in args and b'--exit-on-discontinuity' in args:
            targets.append(int(proc.name))
    except OSError:pass
assert len(targets)==1,targets
os.kill(targets[0],signal.SIGTERM)
start=time.monotonic()
observed_invalid=False
while time.monotonic()-start<35:
    raw=json.loads((live/'raw-state.json').read_text())
    try:
        state=json.loads((live/'live-state.json').read_text())
        if state.get('status')!='live' and state.get('pose') is None:
            observed_invalid=True
    except OSError:pass
    if (raw.get('session_id')!=old_session and raw.get('odometry_messages',0)>70
            and time.time()-float(raw.get('odometry_received_unix_s') or 0)<2):
        assert observed_invalid,'UI state did not invalidate pose'
        result=dict(passed=True,scope='terminated only workstation relay child; no G1 commands',
            old_session=old_session,new_session=raw['session_id'],
            recovered_after_s=time.monotonic()-start,odometry_messages=raw['odometry_messages'],
            pose_hidden_during_restart=observed_invalid)
        (live.parent/'diagnostic-20261109/recovery-test.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
        break
    time.sleep(.4)
else:
    raise AssertionError('Local localization did not recover within 35 seconds')
