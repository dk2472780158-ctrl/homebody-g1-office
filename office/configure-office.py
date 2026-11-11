import hb_config
from pathlib import Path
root = Path(hb_config.expand('${HB_REPO_ROOT}'))
def change(name, old, new):
    p = root / name
    text = p.read_text()
    assert text.count(old) == 1, (name, text.count(old))
    p.write_text(text.replace(old, new))
change('src/homebody/real2sim/exploration/config.py',
    '    """Dex3 finger servo gains (N m/rad, N m s/rad) and limits."""\n',
    '    """Optional Dex3 hardware and its finger servo gains and limits."""\n    enabled: bool = True\n')
change('hardware/g1/explore.sh',
    '    SLAM_IMAGE=slam.image RECORDER_IMAGE=recording.image ZONE_SIZE_M=exploration.zone_size_m)',
    '    HANDS_ENABLED=hands.enabled \\\n    SLAM_IMAGE=slam.image RECORDER_IMAGE=recording.image ZONE_SIZE_M=exploration.zone_size_m)')
change('hardware/g1/explore.sh',
    '# Dex3 does not answer: open them, or refuse now\nif [ "$OBSERVE" != "1" ]; then',
    '# Dex3 does not answer: open them, or refuse now when configured.\nif [ "$OBSERVE" != "1" ] && [ "$HANDS_ENABLED" = "1" ]; then')
change('src/homebody/real2sim/exploration/explore_backend.py',
    '        if self._hands_note is None:\n            try:\n                return super().gripper_state()',
    '        from .settings import current\n\n        if not current().hands.enabled:\n            self._hands_note = "Dex3 disabled in navigation settings; served as UNKNOWN"\n        if self._hands_note is None:\n            try:\n                return super().gripper_state()')
(root / 'configs/office-navigation.toml').write_text('''# Navigation only. Network and live camera must be set from measured hardware.\n[hands]\nenabled = false\n\n[exploration]\ntask = "Navigate the accessible parts of this office according to the operator goal."\n''')
print('Applied explicit optional-hand configuration; no hardware processes started.')
