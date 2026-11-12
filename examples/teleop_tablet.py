# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

"""Huion/evdev drawing with RIO: devices, check-input, calibrate, preview, and run.

All drawing geometry uses meters and rotation vectors. Calibration enables
gravity-compensated hand guiding. Only the explicit `run` command starts RIO.
"""

import argparse
import json
import multiprocessing as mp
import time as wall_time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from rio_hw import time
from scipy.spatial.transform import Rotation

from rio.control.tablet import DrawingController, PaperCalibration, TabletSample
from rio.control.tablet_input import LinuxTablet


def select_device(path: str) -> str:
    if path != "auto":
        return path
    devices, inaccessible = LinuxTablet.discover()
    if len(devices) != 1:
        raise ValueError(
            f"Select a pen node with --device /dev/input/eventN. Found: {devices}. "
            f"Unreadable nodes: {inaccessible}. Run the devices command first."
        )
    return devices[0][0]


def check_tablet(args):
    """Exercise the production input reader without a robot or paper calibration."""
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        raise ValueError("--seconds must be finite and positive.")
    minimum = np.full(2, np.inf)
    maximum = np.full(2, -np.inf)
    touched = lifted = False
    count = 0
    last_keys = None
    buttons_pressed = [False, False]
    buttons_released = [False, False]
    last_print = -np.inf
    with LinuxTablet(select_device(args.device)) as tablet:
        print(f"Reading {tablet.name}. No robot connection.")
        supported = getattr(tablet, "buttons_supported", None)
        if supported is not None:
            print(f"Kernel button capabilities: BTN_STYLUS={supported[0]}, BTN_STYLUS2={supported[1]}")
            if not all(supported):
                print(
                    "Missing button capabilities: also inspect the tablet's mouse/keyboard nodes in evtest and driver mappings."
                )
        print(f"For {args.seconds:g} seconds: move left/right and up/down, touch the tip, then lift it into hover.")
        print("While hovering close to the tablet, press and release EACH stylus side button.")
        deadline = wall_time.monotonic() + args.seconds
        while wall_time.monotonic() < deadline:
            for sample in tablet.poll():
                count += 1
                if sample.proximity:
                    minimum = np.minimum(minimum, [sample.u, sample.v])
                    maximum = np.maximum(maximum, [sample.u, sample.v])
                touched |= sample.touching
                lifted |= touched and sample.proximity and not sample.touching
                for index, pressed in enumerate((sample.button1, sample.button2)):
                    buttons_pressed[index] |= pressed
                    buttons_released[index] |= buttons_pressed[index] and not pressed
                now = wall_time.monotonic()
                keys = (sample.touching, sample.proximity, sample.button1, sample.button2)
                if now - last_print >= 0.1 or keys != last_keys:
                    print(
                        f"u={sample.u:.3f} v={sample.v:.3f} pressure={sample.pressure:.3f} "
                        f"contact={int(sample.touching)} proximity={int(sample.proximity)} "
                        f"button1={int(sample.button1)} button2={int(sample.button2)}",
                        flush=True,
                    )
                    last_print = now
                last_keys = keys
            wall_time.sleep(0.005)
    moved = maximum - minimum > 0.01
    print(f"Reports={count}; X moved={bool(moved[0])}; Y moved={bool(moved[1])}; tip down={touched}; tip lifted={lifted}")
    print(f"Button1 pressed/released={buttons_released[0]}; Button2 pressed/released={buttons_released[1]}")
    if not (moved.all() and touched and lifted):
        raise RuntimeError("Input check incomplete: repeat with movement on both axes and a tip-down -> hover transition.")
    if getattr(args, "require_buttons", False) and not all(buttons_released):
        raise RuntimeError("Button check incomplete: press and release each side button while the pen is in proximity.")
    print("PASS: the drawing code received absolute X/Y, pen contact, and pen lift.")
    if all(buttons_released):
        print("PASS: both stylus side buttons were pressed and released.")


def _sdk_ok(code, operation: str):
    if code != 0:
        raise RuntimeError(f"{operation} failed (SDK code {code}).")


def calibrate(args):
    from xarm.wrapper import XArmAPI

    arm = XArmAPI(args.robot_ip, is_radian=True, do_not_open=True)
    poses = []
    guiding_changed = False
    hold_error = None
    try:
        arm.connect()
        code, errors = arm.get_err_warn_code()
        _sdk_ok(code, "Reading controller errors")
        if any(errors):
            raise RuntimeError(f"Resolve controller error/warning {errors} in Studio before hand guiding.")
        if arm.get_is_moving():
            raise RuntimeError("The arm is moving. Stop other robot-control scripts before calibration.")
        input("Stop other robot-control scripts. Support the arm with the pen clear of paper, then press Enter: ")
        guiding_changed = True  # Also clean up if enabling teaching mode only partially succeeds.
        _sdk_ok(arm.motion_enable(True), "Enabling arm motors")
        _sdk_ok(arm.set_mode(2), "Enabling gravity-compensated hand guiding")
        _sdk_ok(arm.set_state(0), "Starting hand guiding")
        print(
            "Hand guiding is enabled (mode 2). Move the arm by hand to teach three TCP contact poses.\n"
            "Motors remain enabled for gravity compensation; no pose trajectories are sent.\n"
            "Keep the pen orientation and TCP configuration identical at all three points.\n"
            "The corners define the actual drawing rectangle, not necessarily the full sheet."
        )
        for label in ("origin (tablet top-left)", "+U corner (tablet top-right)", "+V corner (tablet bottom-left)"):
            input(f"Place pen in light contact at {label}, then press Enter: ")
            code, values = arm.get_position_aa(is_radian=True)
            if code != 0:
                raise RuntimeError(f"Reading robot TCP pose failed: {code}")
            pose = np.asarray(values, dtype=float)
            pose[:3] *= 0.001  # SDK millimeters -> RIO meters; rotation is already axis-angle/radians.
            if pose.shape != (6,) or not np.isfinite(pose).all():
                raise ValueError("Robot returned an invalid TCP pose.")
            if poses:
                angle = (Rotation.from_rotvec(poses[0][3:]).inv() * Rotation.from_rotvec(pose[3:])).magnitude()
                if angle > np.deg2rad(3):
                    raise ValueError("Pen orientation changed by more than 3 degrees. Repeat calibration.")
            poses.append(pose)
            print(f"TCP: {pose.tolist()}")
    finally:
        try:
            if guiding_changed:
                try:
                    _sdk_ok(arm.set_state(4), "Stopping hand guiding")
                    _sdk_ok(arm.set_mode(0), "Returning to position-hold mode")
                    _sdk_ok(arm.set_state(0), "Holding the current position")
                    print("Hand guiding disabled; arm holding its current position.")
                except Exception as exc:
                    hold_error = str(exc)
                    print(f"Could not restore position hold: {exc} Check the arm in Studio before continuing.")
        finally:
            arm.disconnect()
    if hold_error is not None:
        raise RuntimeError(f"Calibration not saved because restoring position hold failed: {hold_error}")
    calibration = PaperCalibration(poses[0][:3].tolist(), poses[1][:3].tolist(), poses[2][:3].tolist(), poses[0][3:].tolist())
    # For a desktop drawing surface, clearance points toward base +Z.
    calibration.normal_sign = 1 if calibration.normal[2] >= 0 else -1
    if args.flip_normal:
        calibration.normal_sign *= -1
    calibration.save(args.output)
    print(f"Saved {args.output}; retract normal = {calibration.normal.tolist()}")


class SvgPreview:
    def __init__(self, calibration: PaperCalibration):
        self.calibration = calibration
        lengths = np.linalg.norm(calibration.axes, axis=0)
        self.width, self.height = 800 * lengths / lengths.max()
        self.paths: list[list[tuple[float, float]]] = []
        self.drawing = False

    def add_pose(self, pose: np.ndarray, phase: str):
        u, v, height = self.calibration.coordinates(pose[:3])
        contact = abs(height) < 1e-8 and phase in ("lower", "draw")
        if contact:
            if not self.drawing:
                self.paths.append([])
            self.paths[-1].append((float(u * self.width + 10), float(v * self.height + 10)))
        self.drawing = contact

    def save(self, path: str):
        paths = []
        for points in self.paths:
            if points:
                # A zero-length final segment also makes single-point dots visible.
                coords = " L ".join(f"{x:.3f},{y:.3f}" for x, y in [*points, points[-1]])
                paths.append(f'<path d="M {coords}"/>')
        Path(path).write_text(
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.width + 20:.3f} {self.height + 20:.3f}">'
            f'<rect x="10" y="10" width="{self.width:.3f}" height="{self.height:.3f}" fill="white" stroke="#aaa"/>'
            '<g fill="none" stroke="#155eef" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">'
            + "".join(paths)
            + "</g></svg>\n"
        )


def demo_samples() -> list[TabletSample]:
    return [
        TabletSample(0.1, 0.1, False),
        TabletSample(0.1, 0.1, True, pressure=0.5),
        TabletSample(0.4, 0.1, True, pressure=0.5),
        TabletSample(0.4, 0.4, True, pressure=0.5),
        TabletSample(0.1, 0.4, True, pressure=0.5),
        TabletSample(0.1, 0.1, True, pressure=0.5),
        TabletSample(0.1, 0.1, False),
        TabletSample(0.6, 0.6, True, pressure=0.5),
        TabletSample(0.9, 0.9, True, pressure=0.5),
        TabletSample(0.9, 0.9, False),
    ]


def make_controller(args, calibration: PaperCalibration, pose: np.ndarray) -> DrawingController:
    return DrawingController(
        calibration,
        pose,
        clearance=args.clearance_mm * 0.001,
        speed=args.speed_mm_s * 0.001,
        lift_speed=args.lift_speed_mm_s * 0.001,
        max_queue=args.max_queue,
    )


def preview(args):
    calibration = PaperCalibration.load(args.calibration)
    pose = np.concatenate((np.asarray(calibration.origin) + args.clearance_mm * 0.001 * calibration.normal, calibration.rotvec))
    controller = make_controller(args, calibration, pose)
    svg = SvgPreview(calibration)
    dt = 1 / args.freq
    if args.demo:
        for sample in demo_samples():
            controller.add_sample(sample)
    else:
        print("Preview only; no robot connection. Draw on the tablet, then Ctrl+C to save the SVG.")
        try:
            with LinuxTablet(select_device(args.device)) as tablet:
                while True:
                    for sample in tablet.poll():
                        controller.add_sample(sample)
                    svg.add_pose(controller.step(dt), controller.phase)
                    wall_time.sleep(dt)
        except KeyboardInterrupt:
            pass
    # Include strokes that were buffered when preview ended.
    while controller.queue:
        svg.add_pose(controller.step(dt), controller.phase)
    svg.save(args.output)
    print(f"Saved {args.output} ({len(svg.paths)} strokes). No robot was connected.")


def read_pose(env) -> np.ndarray:
    state = env.robot.arm.get_state()
    timestamp = float(state.get("timestamp", float("nan")))
    if not np.isfinite(timestamp) or not -0.1 <= time.now() - timestamp <= 0.5:
        raise RuntimeError("Robot state is missing or stale; no drawing command sent.")
    measured = np.asarray(state["eef_pose"], dtype=float)
    if measured.shape != (6,) or not np.isfinite(measured).all():
        raise RuntimeError("Robot reported an invalid TCP pose.")
    return measured


def drawing_step(env, controller, dt, target_time, sample):
    """Send and record the exact RIO action, including input provenance."""
    measured = read_pose(env)
    if np.linalg.norm(measured[:3] - controller.position) > 0.01:
        raise RuntimeError("Robot is more than 10 mm behind the drawing target; stopping.")
    pose = controller.step(dt, measured_position=measured[:3])
    # No gripper client is created by Lite6TabletStation; its placeholder action stays fixed.
    action = env.robot.build_action(pose, gripper_cmd=0.0)
    env.move(action, t_cmd_target=target_time)
    step = env.get_state(action=action)
    step.meta = {
        "tablet": asdict(sample),
        "drawing_phase": controller.phase,
        "drawing_queue": len(controller.queue),
        "drawing_target_time": target_time,
    }
    if env.recorder:
        env.recorder.record_step(step)
    return step


def run(args):
    from loguru import logger
    from rio_hw.middleware import ServerManager

    from examples.cfg.lite6_tablet import Lite6TabletStation
    from rio.envs.env import make_env

    calibration = PaperCalibration.load(args.calibration)
    # Validate drawing settings and input permissions before constructing any robot nodes.
    preview_pose = np.concatenate((np.asarray(calibration.origin), calibration.rotvec))
    make_controller(args, calibration, preview_pose)
    station = Lite6TabletStation()
    if args.station:
        from examples import cfg

        if args.station not in cfg.__all__:
            raise ValueError(f"Unknown station: {args.station}")
        station = getattr(cfg, args.station)()
    if station.action_space.lower() != "task_pos" or station.embodiment_type.upper() != "SINGLE_ARM":
        raise ValueError("Tablet drawing requires a SINGLE_ARM station with task_pos actions.")
    if station.arm != "XarmArm" or station.arm_cfg.cfg.get("robot_model", "").lower() != "lite6":
        raise ValueError("This entry point expects an XarmArm configured for lite6.")
    if station.arm_cfg.cfg.get("robot_controller", "").lower() != "task_pos":
        raise ValueError("Tablet drawing requires robot_controller=task_pos.")
    if station.arm_cfg.cfg.get("joints_init") is not None:
        raise ValueError("Remove joints_init from the drawing station to prevent startup homing.")
    if getattr(station, "gripper", None) or getattr(station, "hand", None):
        raise ValueError("Disable gripper and hand nodes in the drawing station; the pen must remain clamped.")
    if getattr(station, "teleop", None):
        raise ValueError("Disable the station's teleop node; this entry point reads the tablet directly.")
    if args.robot_ip:
        station.arm_cfg.cfg["robot_ip"] = args.robot_ip
    station.arm_cfg.cfg["max_pos_speed"] = max(args.speed_mm_s, args.lift_speed_mm_s) * 0.001
    if args.no_record:
        station.recorder = None
    if args.record_path:
        station.recorder_cfg.path = args.record_path
    with LinuxTablet(select_device(args.device)) as tablet:
        mp.set_start_method(station.mp_method, force=True)
        servers, _, env = make_env(station)
        with ServerManager(station.mw, [server for server in servers.values() if server is not None]), env:
            measured = read_pose(env)
            controller = make_controller(args, calibration, measured)
            input("Lift the stylus off the tablet. Press Enter to begin drawing (Ctrl+C retracts and saves): ")
            tablet.poll()  # Discard all pre-session input; never replay old contact events.
            if tablet.events.touching:
                raise ValueError("Stylus is touching the tablet. Lift it and restart.")
            if np.linalg.norm(read_pose(env) - measured) > 0.001:
                raise ValueError("Robot pose changed while waiting to start; restart.")
            controller = make_controller(args, calibration, measured)
            t_start = time.now()
            env.set_start_time(t_start)
            env.set_instruction(station.instruction)
            sample = TabletSample(0.0, 0.0, False, proximity=False)
            dt = 1 / args.freq
            trace = None
            if env.recorder:
                directory = Path(station.recorder_cfg.path)
                directory.mkdir(parents=True, exist_ok=True)
                session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
                calibration.save(directory / f"tablet_{session}_calibration.json")
                trace = (directory / f"tablet_{session}_input.jsonl").open("w")
            try:
                while True:
                    tick_start = time.now()
                    for sample in tablet.poll():
                        if trace:
                            trace.write(json.dumps({"received_at": time.now() - t_start, **asdict(sample)}) + "\n")
                        controller.add_sample(sample)
                    drawing_step(env, controller, dt, time.now() + 2 * dt + station.arm_latency, sample)
                    time.precise_wait(tick_start + dt)
            except KeyboardInterrupt:
                logger.info("Drawing stopped; retracting.")
            finally:
                try:
                    measured = read_pose(env)
                    controller.retract(measured)
                    deadline = time.now() + args.clearance_mm / args.lift_speed_mm_s + 3
                    while controller.queue and time.now() < deadline:
                        tick_start = time.now()
                        drawing_step(env, controller, dt, time.now() + 2 * dt + station.arm_latency, sample)
                        time.precise_wait(tick_start + dt)
                    if controller.queue:
                        logger.error("Retraction did not finish before its timeout; check the arm.")
                except Exception:
                    logger.exception("Could not retract through RIO; check the arm before resuming.")
                finally:
                    if trace:
                        trace.close()
                    if env.recorder:
                        env.recorder.save(wait=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("devices", help="List readable Linux pen input devices.")
    checking = subparsers.add_parser("check-input", help="Verify tablet movement/contact through the drawing input reader.")
    checking.add_argument("--device", default="auto")
    checking.add_argument("--seconds", type=float, default=15.0)
    checking.add_argument(
        "--require-buttons", action="store_true", help="Also require a press and release of each side button."
    )
    teaching = subparsers.add_parser("calibrate", help="Hand-guide the arm to three contact poses; no pose trajectories.")
    teaching.add_argument("--robot-ip", required=True)
    teaching.add_argument("--output", default="paper.json")
    teaching.add_argument("--flip-normal", action="store_true", help="Reverse the default normal pointing toward base +Z.")
    for command in ("preview", "run"):
        child = subparsers.add_parser(command)
        child.add_argument("--calibration", required=True)
        child.add_argument("--device", default="auto", help="Pen node path, preferably /dev/input/by-id/...-event-ifXX.")
        child.add_argument("--freq", type=int, default=50)
        child.add_argument("--clearance-mm", type=float, default=5.0)
        child.add_argument("--speed-mm-s", type=float, default=20.0)
        child.add_argument("--lift-speed-mm-s", type=float, default=5.0)
        child.add_argument("--max-queue", type=int, default=10000)
        if command == "preview":
            child.add_argument("--output", default="tablet_preview.svg")
            child.add_argument("--demo", action="store_true", help="Use synthetic strokes; needs neither tablet nor robot.")
        else:
            child.add_argument("--robot-ip", help="Override the IP in the station.")
            child.add_argument("--station", help="Registered Lite6 drawing station with your cameras/recorder settings.")
            child.add_argument("--record-path", help="Override the recorder directory (default: data/drawing/).")
            child.add_argument("--no-record", action="store_true")
    args = parser.parse_args()
    if hasattr(args, "freq") and not 1 <= args.freq <= 250:
        parser.error("--freq must be between 1 and 250 Hz.")
    try:
        if args.command == "devices":
            devices, inaccessible = LinuxTablet.discover()
            for path, name in devices:
                print(f"{path}: {name}")
            if inaccessible:
                print(f"Permission denied: {', '.join(inaccessible)}. See docs/hardware/teleop/huion.md.")
            if not devices:
                print("No readable pen input nodes found.")
        elif args.command == "check-input":
            check_tablet(args)
        elif args.command == "calibrate":
            calibrate(args)
        elif args.command == "preview":
            preview(args)
        else:
            run(args)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
