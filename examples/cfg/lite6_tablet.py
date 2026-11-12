# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass, field

from rio.cfg import Camera, NodeCfg, RecorderCfg


@dataclass
class Lite6TabletStation:
    # EDIT HERE: measured TCP-to-pen-tip (X, Y, Z), in mm in the reported TCP frame.
    # None = unknown offset (legacy fixed-orientation teaching).
    # Use (0.0, 0.0, 0.0) only if Studio already reports the physical pen-tip TCP.
    tip_offset_mm: tuple[0.0, 0.0, 184.0] | None = None

    arm: str = "XarmArm"
    arm_cfg: NodeCfg = field(
        default_factory=lambda: NodeCfg(
            robot_ip="192.168.1.111",
            robot_model="lite6",
            robot_controller="task_pos",
            max_pos_speed=0.03,
            max_rot_speed=0.1,
            freq=250,
            # No joints_init: connecting must not home an arm holding a pen.
        )
    )
    # The pen stays clamped. No gripper node is created or commanded.
    gripper: str | None = None
    teleop: str | None = None
    cameras: dict[str, Camera] = field(default_factory=dict)
    recorder: str | None = "Recorder"
    recorder_cfg: RecorderCfg = field(default_factory=lambda: RecorderCfg(path="data/drawing/"))
    mw: str = "Thread"
    mp_method: str = "spawn"
    freq: int = 50
    arm_latency: float = 0.0
    action_space: str = "task_pos"
    embodiment_type: str = "SINGLE_ARM"
    instruction: str = "Draw on the paper following the stylus."
