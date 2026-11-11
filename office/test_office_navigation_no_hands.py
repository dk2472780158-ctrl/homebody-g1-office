"""Office navigation without Dex3: settings, observations and actual launcher branch."""
import os
import subprocess
from pathlib import Path
from unittest.mock import patch

import pytest

from homebody.backends.g1.real import RealBackend
from homebody.primitives.navigation.observations.gripper_state import UNKNOWN
from homebody.real2sim.exploration.explore_backend import ExploreRealBackend
from homebody.real2sim.exploration.settings import Settings, from_json, install, load, to_json

ROOT = Path(__file__).resolve().parents[1]


def test_office_settings_round_trip():
    settings = load(ROOT, ROOT / 'configs/office-navigation.toml')
    assert settings.hands.enabled is False
    assert from_json(to_json(settings)).hands.enabled is False
    assert Settings().hands.enabled is True


def test_disabled_hands_never_acquire_transport():
    backend = object.__new__(ExploreRealBackend)
    backend._hands_note = None
    backend._term_stamp = {}
    previous = install(load(ROOT, ROOT / 'configs/office-navigation.toml'))
    try:
        with patch.object(RealBackend, 'gripper_state', side_effect=AssertionError('hardware read')):
            state = backend.gripper_state()
            assert state['held_l'] == UNKNOWN and state['held_r'] == UNKNOWN
            assert backend._term_stamp['gripper_state'] > 0
            assert 'disabled' in backend.hands_note()
    finally:
        install(Settings())


@pytest.mark.parametrize('observe,enabled,expected', [('0','0',False), ('0','1',True), ('1','1',False)])
def test_launcher_only_opens_enabled_hands(tmp_path, observe, enabled, expected):
    source = (ROOT / 'hardware/g1/explore.sh').read_text()
    start = source.index('if [ "$OBSERVE" != "1" ] && [ "$HANDS_ENABLED" = "1" ]; then')
    end = source.index('\nif [ "$RECORD" = 1 ]; then', start)
    marker = tmp_path / 'opened'
    fake_python = tmp_path / 'python'
    fake_python.write_text('#!/bin/sh\ntouch "$MARKER"\n')
    fake_python.chmod(0o755)
    subprocess.run(['bash', '-eu', '-c', source[start:end]], check=True,
                   env={**os.environ, 'OBSERVE': observe, 'HANDS_ENABLED': enabled,
                        'HOMEBODY_PYTHON': str(fake_python), 'SETTINGS': 'unused',
                        'MARKER': str(marker)})
    assert marker.exists() is expected
