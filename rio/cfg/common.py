# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass, field


@dataclass
class VisualizerCfg:
    app_id: str = "rio"
    spawn: bool = True
    freq: int = 30


@dataclass
class RecorderCfg:
    path: str = "data/"
    video_codec: str = "libx264"
    codec_options: dict = field(default_factory=lambda: {"crf": "23", "preset": "fast"})
    verbose: bool = True
    start_recording: bool = True
    direct_encoding: bool = False


@dataclass
class DatasetCfg:
    image_height: int = 180
    image_width: int = 320
    num_joints: int | None = None
    action_dim: int | None = None
    fps: int = 50
    robot_type: str = "panda"
    repo_id: str | None = "rio"
    # Maps robodm camera keys → LeRobot camera names. When None, inferred from data
    # by sorting cameras by resolution (smallest = wrist, larger = exterior views).
    camera_mapping: dict[str, str] | None = None
