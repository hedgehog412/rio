# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

import argparse
import json
import os
from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest
import rio_hw.middleware
import rio_hw.time
import xarm.wrapper
from scipy.spatial.transform import Rotation

import examples.teleop_tablet as entrypoint
import rio.envs.env
from examples.teleop_tablet import SvgPreview, calibrate, check_tablet, drawing_step, preview, run
from rio.control.tablet import DrawingController, PaperCalibration, TabletSample
from rio.control.tablet_input import (
    ABS_PRESSURE,
    ABS_X,
    ABS_Y,
    BTN_STYLUS,
    BTN_STYLUS2,
    BTN_TOOL_PEN,
    BTN_TOUCH,
    EV_ABS,
    EV_KEY,
    EV_SYN,
    INPUT_EVENT,
    SYN_DROPPED,
    SYN_REPORT,
    LinuxTablet,
    TabletEvents,
)

pytestmark = pytest.mark.unit


@pytest.fixture
def paper():
    # A tilted paper plane with a 100 x 100 mm drawing area.
    return PaperCalibration([0.2, 0.0, 0.1], [0.3, 0.0, 0.1], [0.2, 0.08, 0.16], [0.0, np.pi, 0.0])


def start_pose(paper):
    return np.r_[np.asarray(paper.origin) + 0.005 * paper.normal, paper.rotvec]


def packet(*events):
    return b"".join(INPUT_EVENT.pack(123, 456, *event) for event in [*events, (EV_SYN, SYN_REPORT, 0)])


def drain(controller):
    commands, phases = [], []
    for _ in range(20000):
        if not controller.queue:
            return np.asarray(commands), phases
        commands.append(controller.step(0.02))
        phases.append(controller.phase)
    pytest.fail("Drawing queue did not drain.")


def test_mapping_tilted_plane_and_round_trip(paper, tmp_path):
    surface = paper.map_sample(TabletSample(0.25, 0.75, True))
    assert surface is not None
    np.testing.assert_allclose(paper.coordinates(surface), [0.25, 0.75, 0], atol=1e-12)
    np.testing.assert_allclose(paper.normal, [0, -0.6, 0.8], atol=1e-12)
    paper.save(tmp_path / "paper.json")
    loaded = PaperCalibration.load(tmp_path / "paper.json")
    mapped = loaded.map_sample(TabletSample(0.25, 0.75, True))
    assert mapped is not None
    np.testing.assert_allclose(mapped, surface)


def test_known_tip_offset_compensates_arbitrary_teaching_orientations(paper, tmp_path):
    tips = np.asarray([paper.origin, paper.corner_u, paper.corner_v])
    rotations = Rotation.from_euler("xyz", [[0, 0, 0], [35, -20, 10], [-15, 40, 80]], degrees=True)
    offset = np.array([0.012, -0.007, 0.1])
    contacts = np.column_stack((tips - rotations.apply(np.tile(offset, (3, 1))), rotations.as_rotvec()))
    calibration = PaperCalibration.from_contact_poses(list(contacts), offset)
    targets = np.asarray([calibration.origin, calibration.corner_u, calibration.corner_v])
    reference = Rotation.from_rotvec(calibration.rotvec)
    np.testing.assert_allclose(targets + reference.apply(offset), tips, atol=1e-12)
    np.testing.assert_allclose(reference.as_matrix(), rotations[-1].as_matrix(), atol=1e-12)
    np.testing.assert_allclose(calibration.normal, paper.normal, atol=1e-12)
    # Final teaching pose is already a valid start for drawing, despite wrist changes.
    controller = DrawingController(calibration, contacts[-1])
    controller.add_sample(TabletSample(0.4, 0.4, True))
    commands, _ = drain(controller)
    expected_tip = paper.map_sample(TabletSample(0.4, 0.4, True))
    assert expected_tip is not None
    np.testing.assert_allclose(commands[-1, :3] + reference.apply(offset), expected_tip, atol=1e-12)
    calibration.save(tmp_path / "offset.json")
    loaded = PaperCalibration.load(tmp_path / "offset.json")
    assert loaded.tip_offset_m is not None
    np.testing.assert_allclose(loaded.tip_offset_m, offset)


def test_known_axial_tip_offset_allows_large_pen_axis_rotation(paper):
    rotations = Rotation.from_euler("z", [[0], [60], [180]], degrees=True)
    offset = np.array([0, 0, 0.1])
    tips = np.asarray([paper.origin, paper.corner_u, paper.corner_v])
    contacts = np.column_stack((tips - offset, rotations.as_rotvec()))
    calibration = PaperCalibration.from_contact_poses(list(contacts), offset)
    np.testing.assert_allclose(calibration.axes, paper.axes, atol=1e-12)


def test_explicit_zero_tip_offset_allows_rotation_when_tcp_is_at_tip(paper):
    contacts = np.column_stack(([paper.origin, paper.corner_u, paper.corner_v], [[0, 0, 0], [0.5, 0, 0], [0, 0.6, 0]]))
    with pytest.raises(ValueError, match="Unknown tip offset"):
        PaperCalibration.from_contact_poses(list(contacts))
    calibration = PaperCalibration.from_contact_poses(list(contacts), np.zeros(3))
    np.testing.assert_allclose(calibration.axes, paper.axes)


@pytest.mark.parametrize("offset", [[0, 0], [0, float("nan"), 0], [0, 0, float("inf")]])
def test_contact_calibration_rejects_invalid_tip_offset(paper, offset):
    contacts = [np.r_[point, paper.rotvec] for point in (paper.origin, paper.corner_u, paper.corner_v)]
    with pytest.raises(ValueError, match="finite three-element"):
        PaperCalibration.from_contact_poses(contacts, np.asarray(offset))


@pytest.fixture
def repeated_contacts(paper):
    offset = np.array([0.012, -0.007, 0.1])
    rotations = Rotation.from_euler("xyz", [[0, 0, 0], [20, -25, 30]], degrees=True)
    groups = [
        list(np.column_stack((np.asarray(tip) - rotations.apply(np.tile(offset, (2, 1))), rotations.as_rotvec())))
        for tip in (paper.origin, paper.corner_u, paper.corner_v)
    ]
    return offset, groups


def test_repeated_contacts_keep_paper_height_and_tilt_and_save_per_corner_errors(paper, repeated_contacts, tmp_path):
    offset, groups = repeated_contacts
    calibration = PaperCalibration.from_repeated_contacts(groups, offset)
    checks = calibration.contact_checks
    assert checks is not None and len(checks) == 3
    assert all(check["max_discrepancy_mm"] < 1e-9 and check["z_spread_mm"] < 1e-9 for check in checks)
    tips = np.asarray([calibration.origin, calibration.corner_u, calibration.corner_v])
    tips += Rotation.from_rotvec(calibration.rotvec).apply(offset)
    np.testing.assert_allclose(tips, [paper.origin, paper.corner_u, paper.corner_v], atol=1e-12)
    np.testing.assert_allclose(calibration.normal, paper.normal, atol=1e-12)
    calibration.validate_start(groups[-1][-1], 0.005)
    calibration.save(tmp_path / "checked.json")
    assert PaperCalibration.load(tmp_path / "checked.json").contact_checks == checks


def test_repeated_contacts_reject_wrong_tip_length(repeated_contacts):
    offset, groups = repeated_contacts
    wrong = offset + np.array([0, 0, 0.02])
    with pytest.raises(ValueError, match="Contact discrepancy exceeds"):
        PaperCalibration.from_repeated_contacts(groups, wrong)


def test_repeated_contacts_reject_one_inconsistent_corner_and_report_all(paper, repeated_contacts):
    offset, groups = repeated_contacts
    groups[1][1][:3] += 0.005 * paper.normal
    with pytest.raises(ValueError, match=r"origin:.*\+U: XYZ=5.000 mm, Z=4.000 mm.*\+V:"):
        PaperCalibration.from_repeated_contacts(groups, offset)


def test_repeated_contacts_cannot_identify_common_offset_error_without_rotation(paper):
    offset = np.array([0, 0, 0.1])
    groups = [[np.r_[np.asarray(tip) - offset, [0, 0, 0]]] * 2 for tip in (paper.origin, paper.corner_u, paper.corner_v)]
    calibration = PaperCalibration.from_repeated_contacts(groups, offset + np.array([0, 0, 0.02]))
    # Common bias is indistinguishable from a different paper height in this capture.
    assert calibration.contact_checks is not None
    assert all(check["max_discrepancy_mm"] == 0 for check in calibration.contact_checks)


@pytest.mark.parametrize("tolerance", [0, -0.001, float("nan")])
def test_repeated_contacts_require_positive_finite_tolerance(repeated_contacts, tolerance):
    offset, groups = repeated_contacts
    with pytest.raises(ValueError, match="finite and positive"):
        PaperCalibration.from_repeated_contacts(groups, offset, tolerance)


def test_tablet_crop_rejects_outside_and_maps_corners(paper):
    paper.tablet_bounds = (0.1, 0.2, 0.9, 0.8)
    assert paper.map_sample(TabletSample(0.05, 0.5, True)) is None
    assert paper.map_sample(TabletSample(float("nan"), 0.5, True)) is None
    assert paper.map_sample(TabletSample(0.5, 0.5, False, proximity=False)) is None
    np.testing.assert_allclose(paper.map_sample(TabletSample(0.1, 0.2, True)), paper.origin)
    np.testing.assert_allclose(paper.map_sample(TabletSample(0.9, 0.2, True)), paper.corner_u)


@pytest.mark.parametrize("corner", [[0.2, 0, 0.1], [0.25, 0, 0.1], [float("nan"), 0, 0]])
def test_invalid_plane_rejected(corner):
    with pytest.raises(ValueError):
        PaperCalibration([0.2, 0, 0.1], [0.3, 0, 0.1], corner, [0, 0, 0])


def test_start_orientation_and_workspace_are_checked(paper):
    pose = start_pose(paper)
    paper.validate_start(pose, 0.005)
    with pytest.raises(ValueError, match="orientation"):
        paper.validate_start(np.r_[pose[:3], [0, 0, 0]], 0.005)
    with pytest.raises(ValueError, match="rectangle"):
        paper.validate_start(pose + np.r_[0.5, 0, 0, 0, 0, 0], 0.005)
    with pytest.raises(ValueError, match="rectangle"):
        paper.validate_start(np.r_[pose[:3] - 0.01 * paper.normal, paper.rotvec], 0.005)


def test_decoder_uses_complete_frames_and_device_ranges():
    decoder = TabletEvents((100, 1100), (200, 2200), (0, 8191))
    data = packet(
        (EV_ABS, ABS_X, 350),
        (EV_ABS, ABS_Y, 1700),
        (EV_ABS, ABS_PRESSURE, 4096),
        (EV_KEY, BTN_TOOL_PEN, 1),
        (EV_KEY, BTN_TOUCH, 1),
    )
    assert decoder.feed(data[:-3]) == []
    sample = decoder.feed(data[-3:])[0]
    assert (sample.u, sample.v) == (0.25, 0.75)
    assert sample.touching and sample.proximity
    assert sample.pressure == pytest.approx(4096 / 8191)
    assert sample.timestamp == pytest.approx(123.000456)
    # A pressure-only packet keeps the previous coordinates/contact state.
    sample2 = decoder.feed(packet((EV_ABS, ABS_PRESSURE, 2000)))[0]
    assert sample2.u == sample.u and sample2.touching


def test_decoder_preserves_fast_lift_and_recontact():
    decoder = TabletEvents((0, 100), (0, 100))
    reports = decoder.feed(
        packet((EV_KEY, BTN_TOOL_PEN, 1), (EV_KEY, BTN_TOUCH, 1))
        + packet((EV_KEY, BTN_TOUCH, 0))
        + packet((EV_KEY, BTN_TOUCH, 1))
        + packet((EV_KEY, BTN_TOOL_PEN, 0))
    )
    assert [sample.touching for sample in reports] == [True, False, True, False]
    assert not reports[-1].proximity


def test_decoder_preserves_both_button_transitions_without_tip_contact():
    decoder = TabletEvents((0, 100), (0, 100), (0, 8191))
    reports = decoder.feed(
        packet((EV_KEY, BTN_TOOL_PEN, 1), (EV_ABS, ABS_PRESSURE, 123))
        + packet((EV_KEY, BTN_STYLUS, 1))
        + packet((EV_KEY, BTN_STYLUS2, 1))
        + packet((EV_KEY, BTN_STYLUS, 0))
        + packet((EV_KEY, BTN_STYLUS2, 0))
    )
    assert [(s.button1, s.button2) for s in reports] == [
        (False, False),
        (True, False),
        (True, True),
        (False, True),
        (False, False),
    ]
    assert all(s.proximity and not s.touching for s in reports)
    assert all(s.pressure == pytest.approx(123 / 8191) for s in reports)


def test_kernel_event_loss_is_fatal():
    decoder = TabletEvents((0, 100), (0, 100))
    with pytest.raises(RuntimeError, match="dropped"):
        decoder.feed(packet((EV_SYN, SYN_DROPPED, 0)))


def test_reader_idle_does_not_clear_contact_and_disconnect_fails():
    read_fd, write_fd = os.pipe2(os.O_NONBLOCK)
    tablet = LinuxTablet.__new__(LinuxTablet)
    tablet.fd = read_fd
    tablet.events = TabletEvents((0, 100), (0, 100))
    try:
        os.write(write_fd, packet((EV_KEY, BTN_TOOL_PEN, 1), (EV_KEY, BTN_TOUCH, 1)))
        assert tablet.poll()[0].touching
        assert tablet.poll() == []  # evdev is silent for a stationary pen, not disconnected.
        assert tablet.events.touching
        os.close(write_fd)
        write_fd = None
        with pytest.raises(ConnectionError, match="disconnected"):
            tablet.poll()
    finally:
        tablet.close()
        if write_fd is not None:
            os.close(write_fd)


def test_strokes_do_not_connect_and_every_command_is_speed_limited(paper):
    controller = DrawingController(paper, start_pose(paper))
    for sample in (
        TabletSample(0.1, 0.1, True),
        TabletSample(0.3, 0.1, True),
        # A lift whose coordinates are already at the NEXT stroke must not draw there.
        TabletSample(0.7, 0.7, False),
        TabletSample(0.7, 0.7, True),
        TabletSample(0.9, 0.7, True),
        TabletSample(0.9, 0.7, False),
    ):
        controller.add_sample(sample)
    commands, phases = drain(controller)
    distances = np.linalg.norm(np.diff(np.vstack((start_pose(paper)[:3], commands[:, :3])), axis=0), axis=1)
    assert distances.max() <= 0.02 * 0.02 + 1e-12
    coords = np.array([paper.coordinates(command[:3]) for command in commands])
    assert coords[:, 2].min() >= -1e-10
    contact = np.isclose(coords[:, 2], 0, atol=1e-10)
    assert not np.any(contact & (coords[:, 0] > 0.301) & (coords[:, 0] < 0.699))
    assert phases.count("lower") > 1
    np.testing.assert_allclose(commands[:, 3:], np.tile(paper.rotvec, (len(commands), 1)))
    svg = SvgPreview(paper)
    for command, phase in zip(commands, phases, strict=True):
        svg.add_pose(command, phase)
    assert len(svg.paths) == 2


def test_lift_waits_for_measured_arrival_before_travel(paper):
    pose = np.r_[paper.origin, paper.rotvec]
    controller = DrawingController(paper, pose)
    controller.add_sample(TabletSample(0.9, 0.9, True))
    for _ in range(200):
        command = controller.step(0.02, measured_position=np.asarray(paper.origin))
    np.testing.assert_allclose(paper.coordinates(command[:3]), [0, 0, 0.005], atol=1e-12)
    assert controller.phase == "lift"
    controller.step(0.02, measured_position=command[:3])
    command = controller.step(0.02, measured_position=command[:3])
    assert paper.coordinates(command[:3])[0] > 0
    assert controller.phase == "travel"


def test_crop_exit_lifts_then_reentry_lowers(paper):
    controller = DrawingController(paper, start_pose(paper))
    controller.add_sample(TabletSample(0.2, 0.2, True))
    controller.add_sample(TabletSample(1.1, 0.2, True))
    controller.add_sample(TabletSample(0.8, 0.8, True))
    phases = [point.phase for point in controller.queue]
    assert phases == ["lift", "travel", "lower", "lift", "travel", "lower"]


def test_backlog_is_bounded(paper):
    controller = DrawingController(paper, start_pose(paper), max_queue=4)
    controller.add_sample(TabletSample(0.1, 0.1, True))
    controller.add_sample(TabletSample(0.2, 0.1, True))
    with pytest.raises(RuntimeError, match="backlog"):
        controller.add_sample(TabletSample(0.3, 0.1, True))


def test_abort_discards_strokes_and_retracts_without_lateral_motion(paper):
    controller = DrawingController(paper, start_pose(paper))
    controller.add_sample(TabletSample(0.8, 0.8, True))
    measured = np.r_[paper.map_sample(TabletSample(0.25, 0.25, True)), paper.rotvec]
    controller.retract(measured)
    commands, _ = drain(controller)
    local = np.array([paper.coordinates(pose[:3]) for pose in commands])
    np.testing.assert_allclose(local[:, :2], np.full((len(local), 2), 0.25), atol=1e-12)
    assert local[-1, 2] == pytest.approx(0.005)


def test_air_test_lifts_from_paper_and_preserves_gap_on_tilted_plane(paper):
    controller = DrawingController(paper, np.r_[paper.origin, paper.rotvec], draw_height=0.01)
    controller.add_sample(TabletSample(0.2, 0.2, True))
    controller.add_sample(TabletSample(0.8, 0.8, True))
    commands, phases = drain(controller)
    local = np.array([paper.coordinates(pose[:3]) for pose in commands])
    lift = np.array([phase == "lift" for phase in phases])
    draw = np.array([phase == "draw" for phase in phases])
    travel = np.array([phase == "travel" for phase in phases])
    np.testing.assert_allclose(local[lift, :2], 0, atol=1e-12)
    np.testing.assert_allclose(local[travel, 2], 0.015, atol=1e-12)
    np.testing.assert_allclose(local[draw, 2], 0.01, atol=1e-12)
    assert np.all(local[~lift, 2] >= 0.01 - 1e-12)
    controller.retract(commands[-1])
    retraction, _ = drain(controller)
    np.testing.assert_allclose(paper.coordinates(retraction[-1, :3]), [0.8, 0.8, 0.015], atol=1e-12)


@pytest.mark.parametrize("gap", [-0.001, float("nan"), float("inf")])
def test_air_test_rejects_invalid_or_negative_gap(paper, gap):
    with pytest.raises(ValueError, match="draw_height"):
        DrawingController(paper, start_pose(paper), draw_height=gap)


class FakeEnv:
    def __init__(self, pose):
        self.pose = pose.copy()
        self.commands = []
        self.records = []
        self.robot = SimpleNamespace(
            arm=SimpleNamespace(get_state=lambda: {"eef_pose": self.pose, "timestamp": rio_hw.time.now()}),
            build_action=lambda pose, gripper_cmd: np.r_[pose, gripper_cmd],
        )
        self.recorder = SimpleNamespace(record_step=self.records.append)

    def move(self, action, t_cmd_target):
        self.commands.append((action.copy(), t_cmd_target))
        self.pose = action[:6].copy()

    def get_state(self, action):
        return SimpleNamespace(action=action, meta={})

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def set_start_time(self, value):
        self.start_time = value

    def set_instruction(self, value):
        self.instruction = value


def test_rio_step_records_executed_action_and_tablet_metadata(paper):
    env = FakeEnv(start_pose(paper))
    controller = DrawingController(paper, env.pose)
    sample = TabletSample(0.4, 0.4, True, pressure=0.3)
    controller.add_sample(sample)
    for i in range(10):
        drawing_step(env, controller, 0.02, 100 + i * 0.02, sample)
    assert len(env.records) == 10
    assert env.records[-1].meta["tablet"]["pressure"] == 0.3
    assert all(record.action.shape == (7,) and record.action[-1] == 0 for record in env.records)
    np.testing.assert_allclose(env.records[-1].action, env.commands[-1][0])


def test_excessive_robot_tracking_error_stops_before_sending(paper):
    env = FakeEnv(start_pose(paper))
    controller = DrawingController(paper, env.pose)
    env.pose[:3] += 0.02
    with pytest.raises(RuntimeError, match="behind"):
        drawing_step(env, controller, 0.02, 100, TabletSample(0, 0, False))
    assert not env.commands


def test_stale_feedback_stops_before_sending(paper):
    env = FakeEnv(start_pose(paper))
    controller = DrawingController(paper, env.pose)
    env.robot.arm.get_state = lambda: {"eef_pose": env.pose, "timestamp": rio_hw.time.now() - 1}
    with pytest.raises(RuntimeError, match="stale"):
        drawing_step(env, controller, 0.02, 100, TabletSample(0, 0, False))
    assert not env.commands


def test_demo_preview_needs_no_tablet_or_robot(paper, tmp_path, monkeypatch):
    paper.save(tmp_path / "paper.json")
    monkeypatch.setattr(LinuxTablet, "discover", lambda: pytest.fail("Demo must not discover hardware."))
    args = argparse.Namespace(
        calibration=tmp_path / "paper.json",
        output=str(tmp_path / "preview.svg"),
        demo=True,
        freq=50,
        clearance_mm=5,
        speed_mm_s=20,
        lift_speed_mm_s=5,
        max_queue=10000,
    )
    preview(args)
    assert (tmp_path / "preview.svg").read_text().count("<path ") == 2
    assert json.loads((tmp_path / "paper.json").read_text())["origin"] == paper.origin


def test_calibration_hand_guides_then_holds_and_converts_sdk_units(paper, tmp_path, monkeypatch):
    poses = iter([np.r_[np.asarray(point) * 1000, paper.rotvec] for point in (paper.origin, paper.corner_u, paper.corner_v)])
    calls = []
    # No pose/joint command methods: calibration may change modes, never send a trajectory.
    arm = SimpleNamespace(
        connect=lambda: calls.append("connect"),
        disconnect=lambda: calls.append("disconnect"),
        get_err_warn_code=lambda: (0, [0, 0]),
        get_is_moving=lambda: False,
        motion_enable=lambda enabled: calls.append(("enable", enabled)) or 0,
        set_mode=lambda mode: calls.append(("mode", mode)) or 0,
        set_state=lambda state: calls.append(("state", state)) or 0,
        get_position_aa=lambda **_: (0, next(poses).tolist()),
    )
    monkeypatch.setattr(xarm.wrapper, "XArmAPI", lambda *_, **__: arm)
    monkeypatch.setattr("builtins.input", lambda _: "")
    output = tmp_path / "paper.json"
    calibrate(argparse.Namespace(robot_ip="fake", output=output, flip_normal=False))
    taught = PaperCalibration.load(output)
    np.testing.assert_allclose(taught.origin, paper.origin)
    np.testing.assert_allclose(taught.corner_u, paper.corner_u)
    np.testing.assert_allclose(taught.corner_v, paper.corner_v)
    assert calls == [
        "connect",
        ("enable", True),
        ("mode", 2),
        ("state", 0),
        ("state", 4),
        ("mode", 0),
        ("state", 0),
        "disconnect",
    ]


@pytest.mark.parametrize("offset_source", ["cli", "file"])
def test_calibrate_with_tip_offset_accepts_rotated_contacts(paper, tmp_path, monkeypatch, offset_source):
    offset_mm = np.array([12, -7, 100])
    rotations = Rotation.from_euler("xyz", [[0, 0, 0], [30, 40, 50], [40, -30, 80]], degrees=True)
    tips_mm = np.asarray([paper.origin, paper.corner_u, paper.corner_v]) * 1000
    poses = iter(
        np.repeat(np.column_stack((tips_mm - rotations.apply(np.tile(offset_mm, (3, 1))), rotations.as_rotvec())), 2, axis=0)
    )
    arm = SimpleNamespace(
        connect=lambda: None,
        disconnect=lambda: None,
        get_err_warn_code=lambda: (0, [0, 0]),
        get_is_moving=lambda: False,
        motion_enable=lambda _: 0,
        set_mode=lambda _: 0,
        set_state=lambda _: 0,
        get_position_aa=lambda **_: (0, next(poses).tolist()),
    )
    monkeypatch.setattr(xarm.wrapper, "XArmAPI", lambda *_, **__: arm)
    monkeypatch.setattr("builtins.input", lambda _: "")
    monkeypatch.setattr(
        "examples.cfg.lite6_tablet.Lite6TabletStation",
        lambda: SimpleNamespace(tip_offset_mm=offset_mm.tolist() if offset_source == "file" else [0, 0, 0]),
    )
    output = tmp_path / "paper.json"
    calibrate(
        argparse.Namespace(
            robot_ip="fake",
            output=output,
            flip_normal=False,
            tip_offset_mm=offset_mm.tolist() if offset_source == "cli" else None,
        )
    )
    taught = PaperCalibration.load(output)
    targets_mm = np.asarray([taught.origin, taught.corner_u, taught.corner_v]) * 1000
    np.testing.assert_allclose(targets_mm + Rotation.from_rotvec(taught.rotvec).apply(offset_mm), tips_mm, atol=1e-10)
    assert taught.contact_checks is not None and len(taught.contact_checks) == 3


def test_failed_repeat_check_preserves_existing_calibration(repeated_contacts, tmp_path, monkeypatch):
    offset, groups = repeated_contacts
    groups[1][1][:3] += [0, 0, 0.01]
    sdk_poses = np.concatenate(groups).copy()
    sdk_poses[:, :3] *= 1000
    poses = iter(sdk_poses)
    arm = SimpleNamespace(
        connect=lambda: None,
        disconnect=lambda: None,
        get_err_warn_code=lambda: (0, [0, 0]),
        get_is_moving=lambda: False,
        motion_enable=lambda _: 0,
        set_mode=lambda _: 0,
        set_state=lambda _: 0,
        get_position_aa=lambda **_: (0, next(poses).tolist()),
    )
    monkeypatch.setattr(xarm.wrapper, "XArmAPI", lambda *_, **__: arm)
    monkeypatch.setattr("builtins.input", lambda _: "")
    output = tmp_path / "paper.json"
    output.write_text("previous calibration")
    with pytest.raises(ValueError, match="Contact discrepancy exceeds"):
        calibrate(argparse.Namespace(robot_ip="fake", output=output, flip_normal=False, tip_offset_mm=(offset * 1000).tolist()))
    assert output.read_text() == "previous calibration"


@pytest.mark.parametrize("failure", [KeyboardInterrupt, RuntimeError])
def test_hand_guiding_is_disabled_on_aborted_calibration(tmp_path, monkeypatch, failure):
    calls = []

    def read_pose(**_):
        raise failure("Teaching aborted")

    arm = SimpleNamespace(
        connect=lambda: None,
        disconnect=lambda: calls.append("disconnect"),
        get_err_warn_code=lambda: (0, [0, 0]),
        get_is_moving=lambda: False,
        motion_enable=lambda enabled: calls.append(("enable", enabled)) or 0,
        set_mode=lambda mode: calls.append(("mode", mode)) or 0,
        set_state=lambda state: calls.append(("state", state)) or 0,
        get_position_aa=read_pose,
    )
    monkeypatch.setattr(xarm.wrapper, "XArmAPI", lambda *_, **__: arm)
    monkeypatch.setattr("builtins.input", lambda _: "")
    with pytest.raises(failure):
        calibrate(argparse.Namespace(robot_ip="fake", output=tmp_path / "paper.json", flip_normal=False))
    assert calls[-4:] == [("state", 4), ("mode", 0), ("state", 0), "disconnect"]
    assert ("enable", False) not in calls
    assert not (tmp_path / "paper.json").exists()


def test_hand_guiding_failure_does_not_continue_teaching(tmp_path, monkeypatch):
    calls = []
    arm = SimpleNamespace(
        connect=lambda: None,
        disconnect=lambda: calls.append("disconnect"),
        get_err_warn_code=lambda: (0, [0, 0]),
        get_is_moving=lambda: False,
        motion_enable=lambda _: 0,
        set_mode=lambda mode: calls.append(("mode", mode)) or (23 if mode == 2 else 0),
        set_state=lambda state: calls.append(("state", state)) or 0,
    )
    monkeypatch.setattr(xarm.wrapper, "XArmAPI", lambda *_, **__: arm)
    monkeypatch.setattr("builtins.input", lambda _: "")
    with pytest.raises(RuntimeError, match=r"hand guiding.*23"):
        calibrate(argparse.Namespace(robot_ip="fake", output=tmp_path / "paper.json", flip_normal=False))
    assert calls == [("mode", 2), ("state", 4), ("mode", 0), ("state", 0), "disconnect"]
    assert not (tmp_path / "paper.json").exists()


def test_controller_fault_during_teaching_prevents_recording_contact(tmp_path, monkeypatch):
    errors = iter([(0, [0, 0]), (0, [37, 0])])
    calls = []
    arm = SimpleNamespace(
        connect=lambda: None,
        disconnect=lambda: calls.append("disconnect"),
        get_err_warn_code=lambda: next(errors),
        get_is_moving=lambda: False,
        motion_enable=lambda _: 0,
        set_mode=lambda mode: calls.append(("mode", mode)) or 0,
        set_state=lambda state: calls.append(("state", state)) or 0,
    )
    monkeypatch.setattr(xarm.wrapper, "XArmAPI", lambda *_, **__: arm)
    monkeypatch.setattr("builtins.input", lambda _: "")
    with pytest.raises(RuntimeError, match=r"controller error/warning \[37, 0\]"):
        calibrate(argparse.Namespace(robot_ip="fake", output=tmp_path / "paper.json", flip_normal=False))
    assert calls[-4:] == [("state", 4), ("mode", 0), ("state", 0), "disconnect"]
    assert not (tmp_path / "paper.json").exists()


@pytest.mark.parametrize(
    "samples,passes",
    [
        ([TabletSample(0.1, 0.1, True), TabletSample(0.9, 0.9, False)], True),
        ([TabletSample(0.1, 0.1, False), TabletSample(0.9, 0.9, False)], False),
        ([TabletSample(0.1, 0.1, True), TabletSample(0.9, 0.9, True)], False),
        ([TabletSample(0.1, 0.1, True), TabletSample(0.1, 0.9, False)], False),
        ([], False),
    ],
)
def test_input_check_requires_movement_contact_and_lift(monkeypatch, capsys, samples, passes):
    clock = SimpleNamespace(value=0.0)
    monkeypatch.setattr(entrypoint.wall_time, "monotonic", lambda: clock.value)
    monkeypatch.setattr(entrypoint.wall_time, "sleep", lambda dt: setattr(clock, "value", clock.value + dt))
    reports = iter([samples])
    tablet = SimpleNamespace(name="Fake Huion", poll=lambda: next(reports, []))
    monkeypatch.setattr(entrypoint, "LinuxTablet", lambda _: nullcontext(tablet))
    args = argparse.Namespace(device="fake", seconds=0.02)
    if passes:
        check_tablet(args)
        assert "PASS" in capsys.readouterr().out
    else:
        with pytest.raises(RuntimeError, match="incomplete"):
            check_tablet(args)


@pytest.mark.parametrize("button2", [True, False])
def test_input_check_reports_quick_button_edges_and_requires_both(monkeypatch, capsys, button2):
    clock = SimpleNamespace(value=0.0)
    monkeypatch.setattr(entrypoint.wall_time, "monotonic", lambda: clock.value)
    monkeypatch.setattr(entrypoint.wall_time, "sleep", lambda dt: setattr(clock, "value", clock.value + dt))
    samples = [
        TabletSample(0.1, 0.1, True),
        TabletSample(0.9, 0.9, False),
        TabletSample(0.9, 0.9, False, button1=True),
        TabletSample(0.9, 0.9, False),
        TabletSample(0.9, 0.9, False, button2=button2),
        TabletSample(0.9, 0.9, False),
    ]
    reports = iter([samples])
    tablet = SimpleNamespace(name="Fake Huion", buttons_supported=(True, button2), poll=lambda: next(reports, []))
    monkeypatch.setattr(entrypoint, "LinuxTablet", lambda _: nullcontext(tablet))
    args = argparse.Namespace(device="fake", seconds=0.02, require_buttons=True)
    if button2:
        check_tablet(args)
        output = capsys.readouterr().out
        assert "button1=1 button2=0" in output
        assert "button1=0 button2=1" in output
        assert "both stylus side buttons were pressed and released" in output
    else:
        with pytest.raises(RuntimeError, match="Button check incomplete"):
            check_tablet(args)
        assert "Missing button capabilities" in capsys.readouterr().out


@pytest.mark.parametrize("failure", [KeyboardInterrupt, ConnectionError])
@pytest.mark.parametrize("air_gap_mm", [0, 10])
def test_run_retracts_and_saves_after_interrupt_or_disconnect(paper, tmp_path, monkeypatch, failure, air_gap_mm):
    paper.save(tmp_path / "paper.json")
    clock = SimpleNamespace(value=100.0)
    monkeypatch.setattr(rio_hw.time, "now", lambda: clock.value)
    monkeypatch.setattr(rio_hw.time, "precise_wait", lambda target: setattr(clock, "value", max(clock.value, target)))
    monkeypatch.setattr(rio_hw.middleware, "ServerManager", lambda *_: nullcontext())
    monkeypatch.setattr("builtins.input", lambda _: "")
    env = FakeEnv(start_pose(paper))
    saved = []
    env.recorder.save = lambda **_: saved.append(True)
    monkeypatch.setattr(rio.envs.env, "make_env", lambda _: ({}, {}, env))

    class FakeTablet:
        events = SimpleNamespace(touching=False)

        def __init__(self, _):
            self.calls = 0

        def poll(self):
            self.calls += 1
            if self.calls == 2:
                return [TabletSample(0.2, 0.2, True)]
            if self.calls >= 300:
                raise failure()
            return []

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

    monkeypatch.setattr(entrypoint, "LinuxTablet", FakeTablet)
    args = argparse.Namespace(
        calibration=tmp_path / "paper.json",
        device="fake",
        station=None,
        robot_ip="fake",
        record_path=str(tmp_path / "recordings"),
        no_record=False,
        freq=50,
        clearance_mm=5,
        speed_mm_s=20,
        lift_speed_mm_s=5,
        max_queue=10000,
        air_gap_mm=air_gap_mm,
    )
    if failure is ConnectionError:
        with pytest.raises(ConnectionError):
            run(args)
    else:
        run(args)
    assert saved == [True]
    assert any(record.meta["drawing_phase"] == "lower" for record in env.records)
    np.testing.assert_allclose(paper.coordinates(env.pose[:3]), [0.2, 0.2, 0.005 + air_gap_mm * 0.001], atol=1e-9)
    assert env.records[-1].meta["drawing_air_gap_m"] == air_gap_mm * 0.001
    assert env.records[-1].meta["drawing_phase"] == "lift"
    traces = list((tmp_path / "recordings").glob("*_input.jsonl"))
    assert len(traces) == 1
    assert json.loads(traces[0].read_text())["touching"]
