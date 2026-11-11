"""Supervise only isolated workstation sensor and localization subprocesses."""
import json
import os
import signal
import subprocess
import time
import uuid
from pathlib import Path

ROOT = Path('/workspace')
LIVE = ROOT / 'runtime/live-readonly'
running = True


def stop(*_):
    global running
    running = False


def atomic(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
    temp.replace(path)


def shutdown(processes):
    for p in reversed(processes):
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+5
    for p in processes:
        try:
            p.wait(timeout=max(.01, deadline-time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.wait()


def cleanup_isolated_dds():
    # This supervisor runs in its own container with private /dev/shm.
    # Called only before starting children or after every child has exited.
    # SIGKILL during disconnect recovery can leave Fast DDS segments behind.
    directory = Path('/dev/shm')
    for item in (*directory.glob('fastrtps*'), *directory.glob('sem.fastrtps*')):
        try:
            if item.is_file() and not item.is_symlink() and item.stat().st_uid == os.geteuid():
                item.unlink()
        except FileNotFoundError:
            pass


def main():
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    LIVE.mkdir(parents=True, exist_ok=True)
    cleanup_isolated_dds()
    restart = 0
    while running:
        session = str(uuid.uuid4())
        run = LIVE / session
        run.mkdir()
        (LIVE / 'session-id').write_text(session+'\n')
        atomic(LIVE / 'raw-state.json', dict(session_id=session, running=False,
               heartbeat_unix_s=time.time(), reason='new localization session'))
        atomic(LIVE / 'supervisor-state.json', dict(session_id=session, running=True,
               restart_count=restart, started_unix_s=time.time()))
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = str(run / 'ros-logs')
        # All children share bounded local SHM with UDP discovery/fallback.
        # Clean inactive segments before recovery to avoid filling /dev/shm.
        profile = ROOT / 'readonly-dds-udp.xml'
        if not profile.is_file():
            raise RuntimeError('Missing isolated DDS transport profile')
        env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
        env['FASTDDS_DEFAULT_PROFILES_FILE'] = str(profile)
        py = '/usr/bin/python3'
        bridge = Path(os.environ.get('HB_SENSOR_INSTALL','/opt/homebody-sensor-install')) / 'g1_fastlio_bridge/lib/g1_fastlio_bridge'
        commands = [
            ('observer', [py, '-u', str(ROOT/'live_ros_observer.py'), '--directory', str(LIVE), '--session', session]),
            ('cloud-converter', [str(bridge/'cloud_roll_pi_converter'), '--ros-args',
                '-p', 'input_topic:=/utlidar/cloud_livox_mid360', '-p', 'output_topic:=/g1_fastlio/cloud_livox_mid360_upright',
                '-p', 'target_frame_id:=body', '-p', 'rotate_roll_pi:=true']),
            ('imu-converter', [str(bridge/'imu_si_converter'), '--ros-args',
                '-p', 'input_topic:=/utlidar/imu_livox_mid360', '-p', 'output_topic:=/g1_fastlio/imu_livox_mid360_upright',
                '-p', 'target_frame_id:=body', '-p', 'auto_scale_accel:=false',
                '-p', 'accel_scale:=9.80665', '-p', 'rotate_roll_pi:=true']),
            ('fastlio', [str(Path(os.environ.get('HB_SENSOR_INSTALL','/opt/homebody-sensor-install'))/'fast_lio/lib/fast_lio/fastlio_mapping'), '--ros-args',
                '--params-file', str(ROOT/'readonly-live-config.yaml')]),
            ('relay', [py, '-u', str(ROOT/'rosbridge_to_ros2.py'), '--source', os.environ.get('HB_ROSBRIDGE_SOURCE','ws://127.0.0.1:19090'),
                '--exit-on-discontinuity', '--state', str(run/'source-state.json')])]
        processes, files = [], []
        reason = 'operator stopped read-only localization'
        begun = time.monotonic()
        source_first_ready = None
        try:
            for name, command in commands:
                log = (run/(name+'.log')).open('w')
                files.append(log)
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL, env=env, start_new_session=True))
            print(f'Started read-only session {session}', flush=True)
            while running:
                exited = [(name, p.returncode) for (name, _), p in zip(commands, processes) if p.poll() is not None]
                if exited:
                    reason = f'child exited: {exited}'
                    break
                rss_kib = 0
                for p in processes:
                    try:
                        for line in Path(f'/proc/{p.pid}/status').read_text().splitlines():
                            if line.startswith('VmRSS:'):
                                rss_kib += int(line.split()[1])
                    except FileNotFoundError:
                        pass
                if rss_kib > 3*1024*1024:
                    reason = 'localization memory exceeded 3 GiB guard'
                    break
                try:
                    source = json.loads((run/'source-state.json').read_text())
                    raw = json.loads((LIVE/'raw-state.json').read_text())
                    if source.get('ready') and source_first_ready is None:
                        source_first_ready = time.monotonic()
                    if source_first_ready is not None and time.monotonic()-source_first_ready > 15:
                        if raw.get('session_id') != session or time.time()-float(raw.get('odometry_received_unix_s') or 0) > 5:
                            reason = 'fresh sensor input but no fresh odometry'
                            break
                    if time.monotonic()-begun > 10 and time.time()-float(raw.get('heartbeat_unix_s') or 0) > 5:
                        reason = 'observer heartbeat timeout'
                        break
                except (OSError, ValueError):
                    if time.monotonic()-begun > 15:
                        reason = 'localization observation state unavailable'
                        break
                time.sleep(.5)
        finally:
            shutdown(processes)
            cleanup_isolated_dds()
            for log in files:
                log.close()
            atomic(LIVE/'raw-state.json', dict(session_id=session, running=False,
                   heartbeat_unix_s=time.time(), reason=reason))
            atomic(LIVE/'supervisor-state.json', dict(session_id=session, running=running,
                   restart_count=restart, stopped_unix_s=time.time(), reason=reason))
            print(f'Stopped read-only session {session}: {reason}', flush=True)
        restart += 1
        until = time.monotonic()+3
        while running and time.monotonic() < until:
            time.sleep(.2)


if __name__ == '__main__':
    main()
