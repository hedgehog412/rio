# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

"""Exercise compressed recording through the real recorder and container codecs."""

import av
import numpy as np
import pytest
import robodm
from rio_hw.middleware import ServerManager

from rio.envs.factory import make_node


@pytest.mark.integration
@pytest.mark.parametrize("direct_encoding", [False, True])
@pytest.mark.parametrize("video_codec", ["rawvideo", "libx264"])
def test_compressed_recording_roundtrip(tmp_path, direct_encoding, video_codec):
    """Keep every frame and exact non-image values, including across episodes."""
    config = {
        "path": str(tmp_path),
        "start_recording": False,
        "direct_encoding": direct_encoding,
        "video_codec": video_codec,
        "codec_options": {"preset": "ultrafast", "crf": "18"} if video_codec == "libx264" else {},
    }
    server, client = make_node("Thread", "data", "Recorder", config, package="rio")
    frames = 24
    with ServerManager("Thread", [server]), client() as recorder:
        for episode in range(2):
            recorder.new_trajectory(wait=True)
            for i in range(frames):
                recorder.record_dict(
                    {
                        "timestep": i / 30,
                        "cameras": {
                            "wrist": {
                                "rgb": np.full((48, 64, 3), i * 8, dtype=np.uint8),
                                "depth": np.full((48, 64), i + episode * 100, dtype=np.uint16),
                            },
                            "external": {"rgb": np.full((48, 64, 3), 200 - i * 4, dtype=np.uint8)},
                        },
                        "action": np.full(7, i + episode, dtype=np.float32),
                        "instruction": "move the arm",
                    },
                    timestamp=i / 30,
                )
            recorder.save(wait=True)

    for episode, path in enumerate(sorted(tmp_path.glob("*.vla"))):
        trajectory = robodm.Trajectory(str(path), mode="r")
        data = trajectory.load()
        trajectory.close()
        assert len(data) == 6
        assert all(len(values) == frames for values in data.values())
        np.testing.assert_allclose(data["timestep"], np.arange(frames) / 30, atol=1e-6)
        for i in range(frames):
            np.testing.assert_array_equal(data["cameras/wrist/depth"][i], np.full((48, 64), i + episode * 100, np.uint16))
            np.testing.assert_array_equal(data["action"][i], np.full(7, i + episode, np.float32))
            assert data["instruction"][i] == "move the arm"
            for key, expected in [("cameras/wrist/rgb", i * 8), ("cameras/external/rgb", 200 - i * 4)]:
                assert data[key][i].shape == (48, 64, 3)
                assert data[key].dtype == np.uint8
                np.testing.assert_allclose(data[key][i].astype(float), expected, atol=3 if video_codec == "libx264" else 0)
        with av.open(str(path)) as container:
            codecs = {stream.metadata["FEATURE_NAME"]: stream.codec_context.name for stream in container.streams}
        assert codecs["cameras/wrist/rgb"] == ("h264" if video_codec == "libx264" else "rawvideo")
    assert len(list(tmp_path.glob("*.vla"))) == 2
    assert not list(tmp_path.glob("*.temp"))
