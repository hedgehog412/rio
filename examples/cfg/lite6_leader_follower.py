from dataclasses import dataclass, field

from rio.cfg import Camera, NodeCfg, VisualizerCfg
from rio.cfg.common import RecorderCfg


@dataclass
class Lite6LeaderFollowerStation:
    # Follower Lite6
    arm: str = "XarmArm"
    arm_cfg: NodeCfg = field(
        default_factory=lambda: NodeCfg(
            addr="127.0.0.1:5555",
            robot_ip="192.168.2.176",
            robot_model="lite6",
            robot_controller="joint_pos",
            max_motor_speed=0.20,
            joints_lowpass_alpha=0.2,
            freq=250,
        )
    )

    gripper: str | None = None

    # Leader Lite6
    teleop: str = "XarmArm"
    teleop_cfg: NodeCfg = field(
        default_factory=lambda: NodeCfg(
            addr="127.0.0.1:5556",
            robot_ip="192.168.1.168",
            robot_model="lite6",
            robot_controller="guide",
            freq=250,
        )
    )

    cameras: dict[str, Camera] = field(
        default_factory=lambda: {
            "realsense": Camera(
                addr="127.0.0.1:5130",
                cam_type="Realsense",
                serial="238222074772",
                model="D400",
                enable_color=True,
                enable_depth=False,
                resolution=(480, 640),
                resolution_depth=(480, 640),
                freq=30,
            ),
            "left": Camera(
                addr="127.0.0.1:5131",
                cam_type="Gphoto",
                port="usb:001,048",
                model="dslr",
                resolution=(480, 640),
                enable_color=True,
                enable_depth=False,
                freq=30,
            ),
            "right": Camera(
                addr="127.0.0.1:5132",
                cam_type="Gphoto",
                port="usb:001,033",
                model="dslr",
                resolution=(480, 640),
                enable_color=True,
                enable_depth=False,
                freq=30,
            ),
        }
    )

    # Normally teleop devices are searched under rio_hw.interfaces.
    # Our teleop is another robot, so tell RIO to use rio_hw.robots.
    teleop_module: str = "robots"

    check_alignment: bool = True
    invert_gripper: bool = False
    # Real XArm leaders use the follower's joint filter, not the Gello helper.
    use_leader_smoothing: bool = False
    startup_delay: float = 1.0

    arm_latency: float = 0.0
    gripper_latency: float = 0.0

    mw: str = "Thread"
    mp_method: str = "spawn"

    # Outer leader-follower loop
    freq: int = 15

    action_space: str = "JOINT_POS"
    embodiment_type: str = "SINGLE_ARM"

    instruction: str = "Lite6 leader-follower test"

    visualizer: str | None = None
    visualizer_cfg: VisualizerCfg = field(default_factory=VisualizerCfg)

    # Recording can be disabled for initial testing with --recorder None.
    recorder: str | None = "Recorder"

    recorder_cfg: RecorderCfg = field(
        default_factory=lambda: RecorderCfg(
            path="data/lite6_leader_follower/",
            video_codec="libx264",
            codec_options={
                "crf": "23",
                "preset": "fast",
            },
            verbose=True,
        )
    )
