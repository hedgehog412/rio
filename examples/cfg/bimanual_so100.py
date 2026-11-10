from dataclasses import dataclass, field

from rio.cfg import NodeCfg, VisualizerCfg
from rio.cfg.common import RecorderCfg


@dataclass
class BimanualSO100Station:
    arm1: str = "SoArm" #Right arm
    arm1_cfg: NodeCfg | None = field(
        default_factory=lambda: NodeCfg(
            addr="127.0.0.1:5555",
            port="/dev/ttyACM1",
            calibration_file="/home/jihyun/.cache/huggingface/lerobot/calibration/robots/so_follower/follower_right.json",
            robot_controller="joint_pos",
        )
    )

    arm2: str | None = "SoArm"
    arm2_cfg: NodeCfg | None = field(
        default_factory=lambda: NodeCfg(
            addr="127.0.0.1:5556",
            port="/dev/ttyACM2",
            calibration_file="/home/jihyun/.cache/huggingface/lerobot/calibration/robots/so_follower/follower_left.json",
            robot_controller="joint_pos",
        )
    )

    gripper: str | None = None

    teleop: str = "SoArm" #Right arm
    teleop_cfg: NodeCfg = field(
        default_factory=lambda: NodeCfg(
            port="/dev/ttyACM3",
            motors_enabled=False,
            robot_model="so100",
            robot_controller="joint_pos",
            calibration_file="/home/jihyun/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/leader_right.json",
        )
    )

    teleop2: str | None = "SoArm"
    teleop2_cfg: NodeCfg = field(
        default_factory=lambda: NodeCfg(
            port="/dev/ttyACM0",
            motors_enabled=False,
            robot_model="so100",
            robot_controller="joint_pos",
            calibration_file="/home/jihyun/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/leader_left.json",
        )
    )

    teleop_keyboard: str | None = "Keyboard"

    teleop_module: str = "robots"
    teleop2_module: str = "robots"
    startup_delay: float = 0.5

    arm_latency: float = 0.01
    gripper_latency: float = 0.01
    mw: str = "Thread"
    mp_method: str = "spawn"
    freq: int = 50

    action_space: str = "JOINT_POS"
    embodiment_type: str = "BIMANUAL"

    instruction: str = "Fold the cloth in half."
    visualizer: str | None = None
    visualizer_cfg: VisualizerCfg = field(default_factory=VisualizerCfg)

    recorder: str | None = None
    recorder_cfg: RecorderCfg = field(
        default_factory=lambda: RecorderCfg(
            path="/home/abucker/data/xlerobot/",
            video_codec="libx264",
            codec_options={"crf": "23", "preset": "fast"},
            verbose=True,
        )
    )
