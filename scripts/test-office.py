"""Run hardware-free office regression tests from any working directory."""
import os,subprocess,sys
from pathlib import Path

root=Path(__file__).resolve().parents[1]
env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1',
     'PYTHONPATH':os.pathsep.join([*(str(root/p) for p in ('office','office/sensors','src','vendor')),
                                 os.environ.get('PYTHONPATH','')])}
tests=(
    'test_room_center','test_amcl_task_recovery.AMCLTaskRecoveryChecks',
    'test_pose_correction_recovery.PoseCorrectionChecks','test_turning.TurningChecks',
    'test_turn_drift','test_dynamic_navigation.DynamicNavigationChecks','test_navigation_feedback.NavigationFeedbackChecks',
    'test_g1_real_control','test_control_transport','test_clearance_recovery',
    'test_base_pose_tracking','test_real_execute_ui','test_readonly_windows',
    'test_transport_runtime','test_localization_output_gap','test_map_scan_recovery',
    'test_office_landmarks_gestures','test_official_gesture_service',
    'test_leg_trace_readonly','test_post_stop_observation','test_speed_probe','test_turn_startup',
)
result=subprocess.run([sys.executable,'-B','-m','unittest',*tests],cwd=root/'office',env=env)
if result.returncode:raise SystemExit(result.returncode)
raise SystemExit(subprocess.run([sys.executable,'-B','-m','unittest','discover','-s','office/sensors/amcl',
                                '-p','test_geometry.py'],cwd=root,env=env).returncode)
