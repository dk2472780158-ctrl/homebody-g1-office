"""Run a diagnostic with the isolated supervisor's ROS/Python environment."""
import os
import sys
from pathlib import Path

env=dict(part.split(b'=',1) for part in Path('/proc/1/environ').read_bytes().split(b'\0') if b'=' in part)
os.execve(b'/usr/bin/python3',[b'/usr/bin/python3',*[os.fsencode(arg) for arg in sys.argv[1:]]],env)
