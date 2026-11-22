# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

import json
from pathlib import Path

import numpy as np
import pytest
import robodm

from examples.teleop_tablet import prompt_recording_note
from rio.data.recorder import RecorderClient

pytestmark = pytest.mark.unit


@pytest.mark.parametrize("response", ["Operator: Jihyun; drawing: 꽃", "", EOFError, KeyboardInterrupt])
def test_closing_note_preserves_saved_recording(tmp_path, monkeypatch, response):
    with RecorderClient("Thread", path=str(tmp_path), note="Initial note", video_codec="rawvideo") as recorder:
        recorder.record_dict({"value": np.array([1.0, 2.0])}, timestamp=0.0)
        recorder.save(wait=True, timeout=5)
        recording_path = Path(recorder.path)
    original_recording = recording_path.read_bytes()
    metadata_path = recording_path.with_suffix(".json")
    original_metadata = json.loads(metadata_path.read_text())

    def respond(_):
        if isinstance(response, str):
            return response
        raise response()

    monkeypatch.setattr("builtins.input", respond)
    prompt_recording_note(str(recording_path))
    metadata = json.loads(metadata_path.read_text())
    assert metadata == {**original_metadata, "note": response if isinstance(response, str) and response else "Initial note"}
    assert recording_path.read_bytes() == original_recording
    assert not metadata_path.with_suffix(".json.tmp").exists()
    loaded = robodm.Trajectory(str(recording_path), mode="r").load()
    np.testing.assert_array_equal(loaded["value"][0], [1.0, 2.0])
