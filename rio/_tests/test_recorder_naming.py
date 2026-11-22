# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

import json
import re
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import robodm

import rio.data.recorder as recording
from rio.data.recorder import RecorderClient

pytestmark = pytest.mark.unit


def record_values(recorder):
    recorder.record_dict({"value": np.array([1.0, 2.0])}, timestamp=0.0)
    recorder.record_dict({"value": np.array([3.0, 4.0])}, timestamp=0.02)
    recorder.save(wait=True, timeout=5)
    return Path(recorder.path)


@pytest.mark.parametrize("explicit_file", [False, True])
def test_repeated_recordings_have_unique_names_and_preserve_existing_files(tmp_path, explicit_file):
    requested = tmp_path / "session.vla" if explicit_file else tmp_path / "new_dataset"
    if explicit_file:
        requested.write_bytes(b"previous recording")
    note = "Operator: Jihyun; drawing: flower"
    with RecorderClient("Thread", path=str(requested), note=note, video_codec="rawvideo") as recorder:
        first = record_values(recorder)
        recorder.new_trajectory(note="Operator: Sam; drawing: circle")
        second = record_values(recorder)
    assert first != second
    assert first.exists() and second.exists()
    assert re.search(r"_\d{8}T\d{6}_\d{6}Z_[a-f0-9]{8}\.vla$", first.name)
    assert json.loads(first.with_suffix(".json").read_text())["note"] == note
    assert json.loads(second.with_suffix(".json").read_text())["note"] == "Operator: Sam; drawing: circle"
    loaded = robodm.Trajectory(str(first), mode="r").load()
    assert list(loaded["meta/recording/note"]) == [note, note]
    if explicit_file:
        assert requested.read_bytes() == b"previous recording"
        assert first.parent == tmp_path


def test_same_timestamp_and_identifier_collision_retries_without_overwriting(tmp_path, monkeypatch):
    fixed = datetime(2026, 11, 22, 12, 0, tzinfo=timezone.utc)
    monkeypatch.setattr(recording, "datetime", SimpleNamespace(now=lambda _: fixed))
    identifiers = iter(["a" * 32, "a" * 32, "b" * 32])
    monkeypatch.setattr(recording, "uuid4", lambda: SimpleNamespace(hex=next(identifiers)))
    first = RecorderClient("Thread", path=str(tmp_path), video_codec="rawvideo")
    # An empty first recording has reserved its JSON name but may not have a .vla yet.
    second = RecorderClient("Thread", path=str(tmp_path), video_codec="rawvideo")
    assert first.path != second.path
    first._record({"value": 1}, 0)
    first._save()
    second._record({"value": 2}, 0)
    second._save()
    assert json.loads(Path(first.path).with_suffix(".json").read_text())["trajectory"] == Path(first.path).name


def test_new_trajectory_can_change_directory_and_note(tmp_path):
    with RecorderClient("Thread", path=str(tmp_path / "one"), video_codec="rawvideo") as recorder:
        first = record_values(recorder)
        recorder.new_trajectory(path=str(tmp_path / "two" / "drawing.vla"), note="New destination")
        second = record_values(recorder)
    assert first.parent == tmp_path / "one"
    assert second.parent == tmp_path / "two"
    assert second.name.startswith("drawing_")
    assert json.loads(second.with_suffix(".json").read_text())["note"] == "New destination"
