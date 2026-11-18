# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

"""Huion/evdev drawing with RIO: devices, check-input, calibrate, preview, and run.

All drawing geometry uses meters and rotation vectors. Calibration enables
gravity-compensated hand guiding. Only the explicit `run` command starts RIO.
"""

import argparse
import json
import multiprocessing as mp
import threading
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
                        f"button1={int(sample.button1)} button2={int(sample.button2)} "
                        f"tilt={sample.tilt_x_deg}/{sample.tilt_y_deg} deg",
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


def inspect_start(args):
    """Read calibration and robot pose without enabling motors or changing modes."""
    from xarm.wrapper import XArmAPI

    calibration = PaperCalibration.load(args.calibration)
    arm = XArmAPI(args.robot_ip, is_radian=True, do_not_open=True)
    try:
        arm.connect()
        code, values = arm.get_position_aa(is_radian=True)
        _sdk_ok(code, "Reading robot TCP pose")
        pose = np.asarray(values, dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError("Robot returned an invalid TCP pose.")
        pose[:3] *= 0.001
    finally:
        arm.disconnect()
    reference = calibration.drawing_pose(pose)
    u, v, height = calibration.coordinates(reference[:3])
    print(f"Reported TCP XYZ (mm): {(pose[:3] * 1000).round(3).tolist()}")
    if calibration.tip_offset_m is not None:
        offset = np.asarray(calibration.tip_offset_m)
        tip = pose[:3] + Rotation.from_rotvec(pose[3:]).apply(offset)
        print(f"Saved TCP-to-tip offset (mm): {(offset * 1000).tolist()}")
        print(f"Computed physical tip XYZ (mm): {(tip * 1000).round(3).tolist()}")
    else:
        print("Saved tip offset: unknown; only manually aligned startup is supported.")
    print(f"Paper coordinates: U={u:.3f}, V={v:.3f} (inside: 0 to 1); height={height * 1000:+.2f} mm")
    controller = DrawingController(calibration, pose, draw_height=args.air_gap_mm * 0.001, clearance=args.clearance_mm * 0.001)
    print(f"PASS: startup position accepted; automatic wrist alignment={controller.aligning}. No motion commands sent.")


def calibrate(args):
    from xarm.wrapper import XArmAPI

    from examples.cfg.lite6_tablet import Lite6TabletStation

    station = Lite6TabletStation()
    offset_mm = getattr(args, "tip_offset_mm", None)
    if offset_mm is None:
        offset_mm = station.tip_offset_mm
    tip_offset = None if offset_mm is None else np.asarray(offset_mm, dtype=float) * 0.001
    if tip_offset is not None and (tip_offset.shape != (3,) or not np.isfinite(tip_offset).all()):
        raise ValueError("tip_offset_mm requires three finite numbers in the reported TCP frame.")
    mode = getattr(station, "calibration_mode", "plane") if tip_offset is not None else "legacy"
    if mode not in ("plane", "repeat", "legacy"):
        raise ValueError("calibration_mode must be 'plane' or 'repeat'.")
    repeats = getattr(station, "calibration_repeats", 2) if mode == "repeat" else 1
    extra_contacts = getattr(station, "calibration_surface_samples", 3) if mode == "plane" else 0
    tolerance_mm = (
        getattr(station, "calibration_plane_tolerance_mm", 2.0)
        if mode == "plane"
        else getattr(station, "calibration_max_discrepancy_mm", 2.0)
    )
    if not isinstance(repeats, int) or repeats < 1:
        raise ValueError("calibration_repeats must be a positive integer.")
    if not np.isfinite(tolerance_mm) or tolerance_mm <= 0:
        raise ValueError("Calibration discrepancy tolerance must be finite and positive.")
    if mode == "plane" and (not isinstance(extra_contacts, int) or extra_contacts < 1):
        raise ValueError("calibration_surface_samples must be a positive integer.")
    arm = XArmAPI(args.robot_ip, is_radian=True, do_not_open=True)
    poses = []
    groups = []
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
            "Hand guiding is enabled (mode 2). Move the arm by hand to teach the three marked corners.\n"
            "Motors remain enabled for gravity compensation; no pose trajectories are sent.\n"
            "The corners define the actual drawing rectangle, not necessarily the full sheet."
        )
        if tip_offset is None:
            print("No tip offset supplied: keep orientation within 3 degrees of the first point.")
        else:
            print(
                f"TCP-to-tip offset: {offset_mm} mm in the reported TCP frame. Wrist orientation may change.\n"
                "The LAST contact defines the reference wrist orientation; tilt following adjusts the pen lean from it."
            )
            if repeats > 1:
                print(
                    f"Re-touch each marked corner {repeats} times, changing wrist tilt but keeping the tip at the SAME point.\n"
                    f"Maximum permitted tip-position discrepancy per corner: {tolerance_mm:g} mm."
                )
            elif mode == "plane":
                print(
                    f"After the three corners, touch {extra_contacts} distinct points anywhere INSIDE the drawing area.\n"
                    f"No exact-point repeats: maximum fitted-plane residual is {tolerance_mm:g} mm per contact."
                )
        labels = ["origin (tablet top-left)", "+U corner (tablet top-right)", "+V corner (tablet bottom-left)"]
        labels += [
            f"additional surface point {i + 1}/{extra_contacts} (different position INSIDE drawing area)"
            for i in range(extra_contacts)
        ]
        for label in labels:
            group = []
            for repeat in range(repeats):
                input(f"Place pen in light contact at {label} ({repeat + 1}/{repeats}), then press Enter: ")
                code, errors = arm.get_err_warn_code()
                _sdk_ok(code, "Checking controller before recording contact")
                if any(errors):
                    raise RuntimeError(
                        f"Calibration stopped: controller error/warning {errors}. Resolve it in Studio before retrying."
                    )
                code, values = arm.get_position_aa(is_radian=True)
                if code != 0:
                    raise RuntimeError(f"Reading robot TCP pose failed: {code}")
                pose = np.asarray(values, dtype=float)
                pose[:3] *= 0.001  # SDK millimeters -> RIO meters; orientation is axis-angle/radians.
                if pose.shape != (6,) or not np.isfinite(pose).all():
                    raise ValueError("Robot returned an invalid TCP pose.")
                if poses and tip_offset is None:
                    angle = (Rotation.from_rotvec(poses[0][3:]).inv() * Rotation.from_rotvec(pose[3:])).magnitude()
                    if angle > np.deg2rad(3):
                        raise ValueError(
                            "Unknown tip offset: keep orientation within 3 degrees or supply --tip-offset-mm X Y Z."
                        )
                poses.append(pose)
                group.append(pose)
                print(f"TCP: {pose.tolist()}")
                if tip_offset is not None:
                    tip = pose[:3] + Rotation.from_rotvec(pose[3:]).apply(tip_offset)
                    print(f"Computed tip XYZ (mm): {(tip * 1000).round(3).tolist()}")
            groups.append(group)
        input(
            "All contacts recorded; hand guiding is still enabled. Lift the pen clear of the paper and move it "
            "above the drawing area. With a known tip offset, startup can restore the final taught wrist orientation. "
            "Support the arm, then press Enter to hold this position and finish. "
            "This repositioning is not recorded as a calibration contact: "
        )
        code, errors = arm.get_err_warn_code()
        _sdk_ok(code, "Checking controller after repositioning")
        if any(errors):
            raise RuntimeError(f"Calibration stopped: controller error/warning {errors}. Resolve it in Studio before retrying.")
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
    if tip_offset is not None and mode == "plane":
        calibration = PaperCalibration.from_surface_contacts(poses, tip_offset, tolerance_mm * 0.001)
        fit = calibration.surface_fit or {}
        for i, residual in enumerate(fit["point_residuals_mm"]):
            print(f"Surface point {i + 1}: distance to fitted paper plane = {residual:+.3f} mm")
        print(f"Plane fit: max={fit['max_abs_residual_mm']:.3f} mm; RMS={fit['rms_residual_mm']:.3f} mm")
    elif tip_offset is not None and repeats > 1:
        calibration = PaperCalibration.from_repeated_contacts(groups, tip_offset, tolerance_mm * 0.001)
        for check in calibration.contact_checks or []:
            print(
                f"{check['corner']}: XYZ discrepancy={check['max_discrepancy_mm']:.3f} mm; "
                f"Z spread={check['z_spread_mm']:.3f} mm"
            )
    else:
        calibration = PaperCalibration.from_contact_poses(poses, tip_offset)
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
        u, v, height = self.calibration.coordinates(self.calibration.drawing_pose(pose)[:3])
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
        lift_speed=(args.lift_speed_mm_s if args.lift_speed_mm_s is not None else 2.0) * 0.001,
        max_queue=args.max_queue,
        draw_height=getattr(args, "air_gap_mm", 0.0) * 0.001,
        follow_tilt=bool(getattr(args, "follow_tilt", False)),
        realtime=bool(getattr(args, "realtime", False)),
        tilt_limit_deg=getattr(args, "tilt_limit_deg", 60.0),
        tilt_speed=np.deg2rad(getattr(args, "tilt_speed_deg_s", 30.0)),
        pen_axis_tcp=getattr(args, "pen_axis_tcp", (0.0, 0.0, -1.0)),
        tilt_signs=getattr(args, "tilt_signs", (1.0, 1.0)),
        button_clutch=bool(getattr(args, "button_clutch", False)),
        clutch_button=getattr(args, "clutch_button", 1),
        clutch_position_speed=getattr(args, "clutch_position_speed_mm_s", 20.0) * 0.001,
        clutch_rotation_speed=np.deg2rad(getattr(args, "clutch_rotation_speed_deg_s", 15.0)),
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
    arm = env.robot.arm
    if isinstance(arm, threading.Thread) and not arm.is_alive():
        raise RuntimeError(
            "RIO arm command thread has stopped, even if pose reports are still arriving. Check its error output."
        )
    state = arm.get_state()
    timestamp = float(state.get("timestamp", float("nan")))
    if not np.isfinite(timestamp) or not -0.1 <= time.now() - timestamp <= 0.5:
        raise RuntimeError("Robot state is missing or stale; no drawing command sent.")
    measured = np.asarray(state["eef_pose"], dtype=float)
    if measured.shape != (6,) or not np.isfinite(measured).all():
        raise RuntimeError("Robot reported an invalid TCP pose.")
    return measured


class RunMonitor:
    """Report input and measured progress; stop a pending move that makes no progress."""

    def __init__(self, pose, now):
        self.initial = np.asarray(pose).copy()
        self.last_progress_pose = self.initial.copy()
        self.last_progress_time = now
        self.last_print = -float("inf")
        self.reports = 0

    def update(self, controller, measured, sample, reports, now):
        self.reports += reports
        displacement = np.linalg.norm(measured[:3] - self.last_progress_pose[:3])
        rotation = (Rotation.from_rotvec(measured[3:]).inv() * Rotation.from_rotvec(self.last_progress_pose[3:])).magnitude()
        if displacement > 0.0001 or rotation > np.deg2rad(0.05):
            self.last_progress_pose = measured.copy()
            self.last_progress_time = now
        position_error = np.linalg.norm(measured[:3] - controller.position)
        angle_error = (Rotation.from_rotvec(measured[3:]).inv() * Rotation.from_rotvec(controller.rotvec)).magnitude()
        pending = controller.aligning or position_error > controller.arrival_tolerance or angle_error > np.deg2rad(0.5)
        if not pending:
            self.last_progress_time = now
        if now - self.last_print >= 1:
            _, _, height = controller.calibration.coordinates(controller.calibration.drawing_pose(measured)[:3])
            target = (
                controller.calibration.rotvec
                if controller.aligning
                else (controller.queue[0].rotvec if controller.queue else controller.rotvec)
            )
            remaining = (Rotation.from_rotvec(measured[3:]).inv() * Rotation.from_rotvec(target)).magnitude()
            moved = np.linalg.norm(measured[:3] - self.initial[:3]) * 1000
            phase = controller.phase if controller.aligning or controller.queue else ("tracking" if pending else "ready")
            if controller.button_clutch:
                phase = controller.clutch_status
            print(
                f"phase={phase}; TCP moved={moved:.2f} mm; tip height={height * 1000:+.2f} mm; "
                f"wrist remaining={np.rad2deg(remaining):.2f} deg; "
                f"tracking={position_error * 1000:.2f} mm/{np.rad2deg(angle_error):.2f} deg; "
                f"tablet reports={self.reports}, tip-down={sample.touching}, tilt={sample.tilt_x_deg}/{sample.tilt_y_deg} deg",
                flush=True,
            )
            self.last_print = now
        if pending and now - self.last_progress_time > 3:
            raise RuntimeError(
                f"No measured robot progress for 3 seconds during {controller.phase}. "
                f"Tracking error={position_error * 1000:.2f} mm/{np.rad2deg(angle_error):.2f} deg. "
                "Stopping; check controller mode, faults and the RIO command thread."
            )


def drawing_step(env, controller, dt, target_time, sample):
    """Send and record the exact RIO action, including input provenance."""
    measured = read_pose(env)
    if not controller.realtime and np.linalg.norm(measured[:3] - controller.position) > 0.01:
        raise RuntimeError("Robot is more than 10 mm behind the drawing target; stopping.")
    angle = (Rotation.from_rotvec(measured[3:]).inv() * Rotation.from_rotvec(controller.rotvec)).magnitude()
    if not controller.realtime and angle > np.deg2rad(5):
        raise RuntimeError("Robot orientation is more than 5 degrees behind the target; stopping.")
    pose = controller.step(dt, measured_pose=measured)
    # No gripper client is created by Lite6TabletStation; its placeholder action stays fixed.
    action = env.robot.build_action(pose, gripper_cmd=0.0)
    env.move(action, t_cmd_target=target_time)
    step = env.get_state(action=action)
    step.meta = {
        "tablet": asdict(sample),
        "drawing_phase": controller.phase,
        "drawing_queue": len(controller.queue),
        "drawing_target_time": target_time,
        "drawing_air_gap_m": controller.draw_height,
        "drawing_follow_tilt": controller.follow_tilt,
        "drawing_realtime": controller.realtime,
        "drawing_clutch_state": controller.clutch_status,
        "drawing_clutch_button": controller.clutch_button if controller.button_clutch else None,
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
    station = Lite6TabletStation()
    if args.station:
        from examples import cfg

        if args.station not in cfg.__all__:
            raise ValueError(f"Unknown station: {args.station}")
        station = getattr(cfg, args.station)()
    if getattr(args, "follow_tilt", None) is None:
        args.follow_tilt = getattr(station, "follow_pen_tilt", False)
    if getattr(args, "realtime", None) is None:
        args.realtime = getattr(station, "realtime", True)
    if args.lift_speed_mm_s is None:
        args.lift_speed_mm_s = getattr(station, "down_speed_mm_s", 2.0)
    if getattr(args, "button_clutch", None) is None:
        args.button_clutch = getattr(station, "button_clutch", True)
    for name in (
        "tilt_limit_deg",
        "tilt_speed_deg_s",
        "pen_axis_tcp",
        "tilt_signs",
        "clutch_button",
        "clutch_position_speed_mm_s",
        "clutch_rotation_speed_deg_s",
    ):
        if hasattr(station, name):
            setattr(args, name, getattr(station, name))
    preview_pose = np.concatenate((np.asarray(calibration.origin), calibration.rotvec))
    make_controller(args, calibration, preview_pose)
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
    station.arm_cfg.cfg["max_pos_speed"] = float("inf") if args.realtime else max(args.speed_mm_s, args.lift_speed_mm_s) * 0.001
    if args.realtime:
        station.arm_cfg.cfg["max_rot_speed"] = float("inf")
    if args.no_record:
        station.recorder = None
    if args.record_path:
        station.recorder_cfg.path = args.record_path
    with LinuxTablet(select_device(args.device)) as tablet:
        if args.follow_tilt and not getattr(tablet, "tilt_supported", False):
            raise ValueError("Selected pen node does not expose both tilt axes. Use --no-follow-tilt for fixed-angle drawing.")
        if args.button_clutch:
            buttons = getattr(tablet, "buttons_supported", (False, False))
            if not buttons[args.clutch_button - 1]:
                raise ValueError(f"Selected pen node does not expose clutch button {args.clutch_button}.")
        mp.set_start_method(station.mp_method, force=True)
        servers, _, env = make_env(station)
        with ServerManager(station.mw, [server for server in servers.values() if server is not None]), env:
            measured = read_pose(env)
            controller = make_controller(args, calibration, measured)
            if controller.aligning:
                print("Startup will lift the pen clear, then slowly align the wrist with TCP-to-tip compensation.")
            if args.air_gap_mm:
                print(f"Air test: drawing targets are {args.air_gap_mm:g} mm above the taught paper plane.")
            print(
                f"Tilt following={args.follow_tilt}; real-time targets={args.realtime}; "
                f"downward approach={args.lift_speed_mm_s:g} mm/s."
            )
            input("Lift the stylus off the tablet. Press Enter to begin drawing (Ctrl+C retracts and saves): ")
            tablet.poll()  # Discard all pre-session input; never replay old contact events.
            if tablet.events.touching:
                raise ValueError("Stylus is touching the tablet. Lift it and restart.")
            tablet.grab()
            print("Tablet pen input node captured exclusively for teleoperation; released on exit.", flush=True)
            if np.linalg.norm(read_pose(env) - measured) > 0.001:
                raise ValueError("Robot pose changed while waiting to start; restart.")
            controller = make_controller(args, calibration, measured)
            input_ready = not controller.aligning
            t_start = time.now()
            env.set_start_time(t_start)
            env.set_instruction(station.instruction)
            sample = TabletSample(0.0, 0.0, False, proximity=False)
            monitor = RunMonitor(measured, time.now())
            last_clutch_status = None
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
                    reports = tablet.poll()
                    accept_input = input_ready and not controller.aligning
                    input_pose = read_pose(env) if reports and controller.button_clutch else None
                    for sample in reports:
                        if trace:
                            trace.write(json.dumps({"received_at": time.now() - t_start, **asdict(sample)}) + "\n")
                        if accept_input:
                            controller.add_sample(sample, measured_pose=input_pose)
                    if not controller.aligning and not tablet.events.touching:
                        input_ready = True
                    was_aligning = controller.aligning
                    lead = dt if args.realtime else 2 * dt
                    drawing_step(env, controller, dt, time.now() + lead + station.arm_latency, sample)
                    monitor.update(controller, read_pose(env), sample, len(reports), time.now())
                    if was_aligning and not controller.aligning:
                        print("Wrist aligned. Lift the stylus, then place it at the stroke start.")
                    if controller.button_clutch and controller.clutch_status != last_clutch_status:
                        last_clutch_status = controller.clutch_status
                        messages = {
                            "ready": f"READY: hold the stylus at this point and press button {controller.clutch_button}.",
                            "hover-ready": "Position matched above the paper. Touch the tablet and wait for READY.",
                            "lowering": "Button accepted. Lowering the pen; hold the stylus still until LIVE TRACKING.",
                            "tracking": "LIVE TRACKING enabled. Lift the stylus to end the stroke and reset the clutch.",
                            "positioning": "Positioning above the stylus smoothly. Wait for READY before pressing the button.",
                            "waiting": "Waiting for stylus proximity. Each stroke requires a fresh button press.",
                            "lifting": "Lifting clear; live tracking is disabled.",
                        }
                        if last_clutch_status in messages:
                            print(messages[last_clutch_status], flush=True)
                    time.precise_wait(tick_start + dt)
            except KeyboardInterrupt:
                logger.info("Drawing stopped; retracting.")
            finally:
                try:
                    measured = read_pose(env)
                    controller.retract(measured)
                    deadline = time.now() + (args.air_gap_mm + args.clearance_mm) / args.lift_speed_mm_s + 3
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
    teaching.add_argument(
        "--tip-offset-mm",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Override tip_offset_mm in examples/cfg/lite6_tablet.py (mm in the reported TCP frame).",
    )
    teaching.add_argument("--flip-normal", action="store_true", help="Reverse the default normal pointing toward base +Z.")
    inspecting = subparsers.add_parser("inspect-start", help="Read startup tip position relative to paper; no motion commands.")
    inspecting.add_argument("--robot-ip", required=True)
    inspecting.add_argument("--calibration", required=True)
    inspecting.add_argument("--air-gap-mm", type=float, default=0.0)
    inspecting.add_argument("--clearance-mm", type=float, default=5.0)
    for command in ("preview", "run"):
        child = subparsers.add_parser(command)
        child.add_argument("--calibration", required=True)
        child.add_argument("--device", default="auto", help="Pen node path, preferably /dev/input/by-id/...-event-ifXX.")
        child.add_argument("--freq", type=int, default=50)
        child.add_argument("--clearance-mm", type=float, default=5.0)
        child.add_argument("--speed-mm-s", type=float, default=20.0)
        child.add_argument(
            "--lift-speed-mm-s",
            type=float,
            default=None,
            help="Downward approach speed in live mode; lift/lower speed otherwise. Defaults to station down_speed_mm_s.",
        )
        child.add_argument("--max-queue", type=int, default=10000)
        child.add_argument(
            "--button-clutch",
            action=argparse.BooleanOptionalAction,
            default=None,
            help="Require a fresh stylus button press after positioning for every stroke (station default).",
        )
        child.add_argument(
            "--follow-tilt",
            action=argparse.BooleanOptionalAction,
            default=None,
            help="Copy pen lean; run defaults to follow_pen_tilt in the station file.",
        )
        child.add_argument(
            "--realtime",
            action=argparse.BooleanOptionalAction,
            default=None,
            help="Use live lateral/upward/tilt targets; downward approach remains limited.",
        )
        if command == "preview":
            child.add_argument("--output", default="tablet_preview.svg")
            child.add_argument("--demo", action="store_true", help="Use synthetic strokes; needs neither tablet nor robot.")
        else:
            child.add_argument("--robot-ip", help="Override the IP in the station.")
            child.add_argument("--station", help="Registered Lite6 drawing station with your cameras/recorder settings.")
            child.add_argument("--record-path", help="Override the recorder directory (default: data/drawing/).")
            child.add_argument("--no-record", action="store_true")
            child.add_argument(
                "--air-gap-mm",
                type=float,
                default=0.0,
                help="Raise drawing targets above the calibrated paper plane for an air test; zero makes contact.",
            )
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
        elif args.command == "inspect-start":
            inspect_start(args)
        elif args.command == "preview":
            preview(args)
        else:
            run(args)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
