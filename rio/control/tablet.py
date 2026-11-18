# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

"""Paper-plane mapping and ordered, speed-limited drawing targets (meters/radians)."""

import json
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class TabletSample:
    u: float
    v: float
    touching: bool
    proximity: bool = True
    pressure: float = 0.0
    timestamp: float = 0.0
    button1: bool = False
    button2: bool = False
    tilt_x_deg: float | None = None
    tilt_y_deg: float | None = None


@dataclass
class PaperCalibration:
    """TCP targets for a paper rectangle at a fixed drawing orientation.

    With a known tip offset, contact poses may have different orientations. Their
    tip positions are converted into TCP targets for the chosen drawing orientation.
    The robot's TCP configuration and pen mounting must stay unchanged afterward.
    """

    origin: list[float]
    corner_u: list[float]
    corner_v: list[float]
    rotvec: list[float]
    normal_sign: int = 1
    tablet_bounds: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)
    tip_offset_m: list[float] | None = None
    contact_checks: list[dict] | None = None
    surface_fit: dict | None = None

    def __post_init__(self):
        points = np.asarray([self.origin, self.corner_u, self.corner_v, self.rotvec], dtype=float)
        if points.shape != (4, 3) or not np.isfinite(points).all():
            raise ValueError("Calibration requires finite 3D points and a rotation vector.")
        if self.normal_sign not in (-1, 1):
            raise ValueError("normal_sign must be -1 or 1.")
        if self.tip_offset_m is not None:
            offset = np.asarray(self.tip_offset_m, dtype=float)
            if offset.shape != (3,) or not np.isfinite(offset).all():
                raise ValueError("tip_offset_m must be a finite three-element vector.")
        if self.surface_fit is not None:
            residual = self.surface_fit.get("max_abs_residual_mm", 0.0)
            if not np.isfinite(residual) or residual < 0:
                raise ValueError("Surface fit residual must be finite and nonnegative.")
        a, b = points[1] - points[0], points[2] - points[0]
        lengths = np.linalg.norm(a), np.linalg.norm(b)
        if min(lengths) < 0.005 or np.linalg.norm(np.cross(a, b)) / np.prod(lengths) < 0.1:
            raise ValueError("Paper corners must span a nondegenerate drawing area (at least 5 mm per side).")
        bounds = np.asarray(self.tablet_bounds, dtype=float)
        if bounds.shape != (4,) or not np.isfinite(bounds).all():
            raise ValueError("tablet_bounds must contain four finite values.")
        x0, y0, x1, y1 = bounds
        if not (0 <= x0 < x1 <= 1 and 0 <= y0 < y1 <= 1):
            raise ValueError("tablet_bounds must be [u_min, v_min, u_max, v_max] within [0, 1].")

    @property
    def axes(self) -> np.ndarray:
        return np.column_stack((np.subtract(self.corner_u, self.origin), np.subtract(self.corner_v, self.origin)))

    @property
    def normal(self) -> np.ndarray:
        normal = np.cross(*self.axes.T)
        return self.normal_sign * normal / np.linalg.norm(normal)

    def map_sample(self, sample: TabletSample) -> np.ndarray | None:
        x0, y0, x1, y1 = self.tablet_bounds
        uv = np.array([(sample.u - x0) / (x1 - x0), (sample.v - y0) / (y1 - y0)])
        if not sample.proximity or not np.isfinite(uv).all() or np.any(uv < 0) or np.any(uv > 1):
            return None
        return np.asarray(self.origin) + self.axes @ uv

    def coordinates(self, position: np.ndarray) -> np.ndarray:
        basis = np.column_stack((self.axes, self.normal))
        return np.linalg.solve(basis, np.asarray(position) - self.origin)

    def validate_start(self, pose: np.ndarray, clearance: float):
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError("Robot must publish a finite six-element axis-angle TCP pose.")
        angle = (Rotation.from_rotvec(pose[3:]).inv() * Rotation.from_rotvec(self.rotvec)).magnitude()
        if angle > np.deg2rad(3):
            raise ValueError("Start orientation differs from calibration by more than 3 degrees; align the pen first.")
        u, v, h = self.coordinates(pose[:3])
        # The final measured contact may sit just below an averaged plane. The
        # controller's first movement is a gated vertical lift, never lateral.
        start_tolerance = 0.0005
        if self.surface_fit is not None:
            start_tolerance = max(start_tolerance, self.surface_fit.get("max_abs_residual_mm", 0.0) * 0.001 + 0.0001)
        details = f"U={u:.3f}, V={v:.3f}; height above paper={h * 1000:+.2f} mm"
        if not (-0.01 <= u <= 1.01 and -0.01 <= v <= 1.01):
            raise ValueError(
                f"Starting tip is outside the calibrated drawing rectangle: {details}. "
                "Move the physical tip over the taught area (U/V from 0 to 1), then retry."
            )
        if h < -start_tolerance:
            raise ValueError(
                f"Starting tip is below the calibrated paper plane: {details}; "
                f"permitted contact discrepancy={start_tolerance * 1000:.2f} mm. "
                "Lift the tip clear. If this disagrees with the real paper, check the saved tip offset and TCP configuration."
            )

    def drawing_pose(self, pose: np.ndarray) -> np.ndarray:
        """Express the current tip location as a TCP at the drawing orientation."""
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (6,) or not np.isfinite(pose).all():
            raise ValueError("Robot must publish a finite six-element axis-angle TCP pose.")
        if self.tip_offset_m is None:
            return pose.copy()  # Legacy files still require manual orientation alignment.
        offset = np.asarray(self.tip_offset_m)
        tip = pose[:3] + Rotation.from_rotvec(pose[3:]).apply(offset)
        return np.r_[tip - Rotation.from_rotvec(self.rotvec).apply(offset), self.rotvec]

    def tilt_rotation(self, sample: TabletSample, pen_axis_tcp, signs, limit_deg: float) -> np.ndarray:
        """Map stylus body lean onto paper U/V, retaining the taught barrel twist."""
        if sample.tilt_x_deg is None or sample.tilt_y_deg is None:
            raise ValueError("Tilt following requires ABS_TILT_X and ABS_TILT_Y on the selected pen node.")
        angles = np.asarray([sample.tilt_x_deg, sample.tilt_y_deg]) * signs
        if not np.isfinite(angles).all() or np.any(np.abs(angles) >= 90):
            raise ValueError("Stylus tilt must contain finite angles strictly between -90 and 90 degrees.")
        u = self.axes[:, 0] / np.linalg.norm(self.axes[:, 0])
        v = self.axes[:, 1] - np.dot(self.axes[:, 1], u) * u
        v /= np.linalg.norm(v)
        slope = np.tan(np.deg2rad(angles))
        length = np.linalg.norm(slope)
        slope *= min(1.0, np.tan(np.deg2rad(limit_deg)) / max(length, 1e-12))
        target = self.normal + slope[0] * u + slope[1] * v
        target /= np.linalg.norm(target)
        reference = Rotation.from_rotvec(self.rotvec)
        source = reference.apply(pen_axis_tcp)
        axis = np.cross(source, target)
        sine = np.linalg.norm(axis)
        cosine = np.clip(np.dot(source, target), -1, 1)
        if sine < 1e-10:
            if cosine > 0:
                return np.asarray(self.rotvec).copy()
            axis = u - np.dot(u, source) * source
            if np.linalg.norm(axis) < 1e-8:
                axis = v - np.dot(v, source) * source
            axis /= np.linalg.norm(axis)
        else:
            axis /= sine
        return (Rotation.from_rotvec(axis * np.arctan2(sine, cosine)) * reference).as_rotvec()

    def save(self, path: str | Path):
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def from_contact_poses(cls, poses: list[np.ndarray], tip_offset_m: np.ndarray | None = None) -> "PaperCalibration":
        """Fit from tip contacts using an offset expressed in the reported TCP frame.

        An explicit offset, including zero for a TCP already at the tip, permits
        arbitrary teaching orientations. Drawing uses the last taught orientation,
        so the arm can start directly from its final calibration pose. With an
        unknown offset, retain the legacy constant-orientation assumption.
        """
        contacts = np.asarray(poses, dtype=float)
        if contacts.shape != (3, 6) or not np.isfinite(contacts).all():
            raise ValueError("Exactly three finite six-element contact poses are required.")
        rotations = Rotation.from_rotvec(contacts[:, 3:])
        if tip_offset_m is None:
            if np.any((rotations[0].inv() * rotations).magnitude() > np.deg2rad(3)):
                raise ValueError("Unknown tip offset: keep orientation within 3 degrees or supply --tip-offset-mm X Y Z.")
            return cls(*contacts[:, :3].tolist(), rotvec=contacts[0, 3:].tolist())
        offset = np.asarray(tip_offset_m, dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("tip_offset_m must be a finite three-element vector.")
        tips = contacts[:, :3] + rotations.apply(np.tile(offset, (3, 1)))
        # p_tip = p_tcp + R_tcp @ offset. Express all contacts at the final R_tcp.
        targets = tips - rotations[-1].apply(offset)
        return cls(*targets.tolist(), rotvec=contacts[-1, 3:].tolist(), tip_offset_m=offset.tolist())

    @classmethod
    def from_repeated_contacts(
        cls, groups: list[list[np.ndarray]], tip_offset_m: np.ndarray, max_discrepancy_m: float = 0.002
    ) -> "PaperCalibration":
        """Check repeated contacts at each corner before fitting the paper plane.

        Each group must touch the SAME marked point; wrist angles may differ.
        This measures tip-position consistency, not absolute offset accuracy.
        The final contact at each corner defines the plane, preserving a valid
        start at the last recorded pose. No tool offset is adjusted automatically.
        """
        if len(groups) != 3:
            raise ValueError("Repeated calibration requires three corner groups.")
        offset = np.asarray(tip_offset_m, dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("tip_offset_m must be a finite three-element vector.")
        if not np.isfinite(max_discrepancy_m) or max_discrepancy_m <= 0:
            raise ValueError("Maximum contact discrepancy must be finite and positive.")
        checks: list[dict] = []
        last_poses = []
        for label, group in zip(("origin", "+U", "+V"), groups, strict=True):
            poses = np.asarray(group, dtype=float)
            if poses.ndim != 2 or poses.shape[0] < 2 or poses.shape[1] != 6 or not np.isfinite(poses).all():
                raise ValueError("Each corner requires at least two finite six-element contact poses.")
            tips = poses[:, :3] + Rotation.from_rotvec(poses[:, 3:]).apply(np.tile(offset, (len(poses), 1)))
            spread = float(np.linalg.norm(tips[:, None, :] - tips[None, :, :], axis=2).max())
            checks.append(
                {
                    "corner": label,
                    "samples": len(poses),
                    "max_discrepancy_mm": spread * 1000,
                    "z_spread_mm": float(np.ptp(tips[:, 2])) * 1000,
                    "contact_poses_m_rad": poses.tolist(),
                    "tip_positions_m": tips.tolist(),
                }
            )
            last_poses.append(poses[-1])
        failed = [check for check in checks if check["max_discrepancy_mm"] > max_discrepancy_m * 1000 + 1e-9]
        if failed:
            details = "; ".join(
                f"{c['corner']}: XYZ={c['max_discrepancy_mm']:.3f} mm, Z={c['z_spread_mm']:.3f} mm" for c in checks
            )
            raise ValueError(
                f"Contact discrepancy exceeds {max_discrepancy_m * 1000:g} mm. {details}. "
                "Check the measured tip offset, pen mounting, and repeated contact locations; calibration was not saved."
            )
        calibration = cls.from_contact_poses(last_poses, offset)
        calibration.contact_checks = checks
        return calibration

    @classmethod
    def from_surface_contacts(
        cls, poses: list[np.ndarray], tip_offset_m: np.ndarray, max_residual_m: float = 0.002
    ) -> "PaperCalibration":
        """Fit a paper plane from distinct contacts; the first three map the area.

        Remaining contacts may be anywhere inside that area. Every computed tip
        should lie near the fitted plane; no repeated XYZ location is required.
        This checks planarity, not absolute accuracy of the supplied tip offset.
        """
        contacts = np.asarray(poses, dtype=float)
        offset = np.asarray(tip_offset_m, dtype=float)
        if contacts.ndim != 2 or contacts.shape[0] < 4 or contacts.shape[1] != 6 or not np.isfinite(contacts).all():
            raise ValueError("Surface calibration requires at least four finite six-element contact poses.")
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("tip_offset_m must be a finite three-element vector.")
        if not np.isfinite(max_residual_m) or max_residual_m <= 0:
            raise ValueError("Maximum plane residual must be finite and positive.")
        rotations = Rotation.from_rotvec(contacts[:, 3:])
        tips = contacts[:, :3] + rotations.apply(np.tile(offset, (len(contacts), 1)))
        center = tips.mean(axis=0)
        _, singular, vh = np.linalg.svd(tips - center, full_matrices=False)
        if singular[1] <= max(singular[0] * 1e-3, 1e-6):
            raise ValueError("Surface contacts must span an area, not a line.")
        normal = vh[-1]
        if normal[2] < 0:
            normal = -normal
        residuals = (tips - center) @ normal
        maximum = float(np.abs(residuals).max())
        if maximum > max_residual_m + 1e-12:
            details = ", ".join(f"point {i + 1}: {r * 1000:+.3f} mm" for i, r in enumerate(residuals))
            raise ValueError(
                f"Paper-plane residual exceeds {max_residual_m * 1000:g} mm. {details}. "
                "Check contact, paper flatness, and tip offset; calibration was not saved."
            )
        # The three projected anchors map the drawing area. Commands use the final
        # wrist orientation and the same offset as the measured contact poses.
        anchors = tips[:3] - residuals[:3, None] * normal
        targets = anchors - rotations[-1].apply(offset)
        calibration = cls(*targets.tolist(), rotvec=contacts[-1, 3:].tolist(), tip_offset_m=offset.tolist())
        calibration.surface_fit = {
            "samples": len(contacts),
            "max_abs_residual_mm": maximum * 1000,
            "rms_residual_mm": float(np.sqrt(np.mean(residuals**2))) * 1000,
            "tolerance_mm": max_residual_m * 1000,
            "point_residuals_mm": (residuals * 1000).tolist(),
            "normal_base": normal.tolist(),
            "centroid_tip_m": center.tolist(),
            "contact_poses_m_rad": contacts.tolist(),
            "tip_positions_m": tips.tolist(),
        }
        return calibration

    @classmethod
    def load(cls, path: str | Path) -> "PaperCalibration":
        return cls(**json.loads(Path(path).read_text()))


class DrawingController:
    """Preserve every stroke while moving lift -> travel -> lower -> draw.

    Input waypoints are queued rather than replacing the latest target. A loop
    iteration may consume multiple short draw segments, but never skips a lift.
    Transition waypoints wait for measured arrival when feedback is supplied.
    """

    @dataclass
    class Waypoint:
        position: np.ndarray
        phase: str
        gate: bool = False
        rotvec: np.ndarray | None = None

    def __init__(
        self,
        calibration: PaperCalibration,
        start_pose: np.ndarray,
        clearance: float = 0.005,
        speed: float = 0.02,
        lift_speed: float = 0.005,
        arrival_tolerance: float = 0.0005,
        max_queue: int = 10000,
        draw_height: float = 0.0,
        alignment_clearance: float = 0.02,
        alignment_speed: float = np.deg2rad(3),
        follow_tilt: bool = False,
        realtime: bool = False,
        tilt_limit_deg: float = 60.0,
        tilt_speed: float = np.deg2rad(30),
        pen_axis_tcp: tuple[float, float, float] = (0.0, 0.0, -1.0),
        tilt_signs: tuple[float, float] = (1.0, 1.0),
    ):
        if not all(
            np.isfinite(x) and x > 0
            for x in (clearance, speed, lift_speed, arrival_tolerance, alignment_clearance, alignment_speed)
        ):
            raise ValueError("Clearance, speeds, and arrival tolerance must be finite and positive.")
        if max_queue < 4:
            raise ValueError("max_queue must be at least four.")
        if not np.isfinite(draw_height) or draw_height < 0:
            raise ValueError("draw_height must be finite and nonnegative.")
        if follow_tilt and calibration.tip_offset_m is None:
            raise ValueError("Tilt following needs a known tip_offset_m saved in the calibration.")
        axis = np.asarray(pen_axis_tcp, dtype=float)
        signs = np.asarray(tilt_signs, dtype=float)
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
            raise ValueError("pen_axis_tcp must be a finite nonzero three-element vector from tip toward pen body.")
        if signs.shape != (2,) or not np.isfinite(signs).all() or not np.isin(signs, [-1, 1]).all():
            raise ValueError("tilt_signs must be two values of +1 or -1.")
        if not np.isfinite(tilt_limit_deg) or not 0 < tilt_limit_deg < 90 or not np.isfinite(tilt_speed) or tilt_speed <= 0:
            raise ValueError("Tilt limit must be between 0 and 90 degrees and tilt_speed must be finite and positive.")
        self.follow_tilt, self.realtime = follow_tilt, realtime
        self.tilt_limit_deg, self.tilt_speed = tilt_limit_deg, tilt_speed
        self.pen_axis_tcp, self.tilt_signs = axis / np.linalg.norm(axis), signs
        hover_height = draw_height + clearance
        drawing_pose = calibration.drawing_pose(start_pose)
        calibration.validate_start(drawing_pose, hover_height)
        self.calibration = calibration
        self.position = np.array(start_pose[:3], dtype=float)
        self.rotvec = np.array(start_pose[3:], dtype=float)
        angle = (Rotation.from_rotvec(self.rotvec).inv() * Rotation.from_rotvec(calibration.rotvec)).magnitude()
        self.aligning = calibration.tip_offset_m is not None and angle > 1e-8
        self.alignment_speed = alignment_speed
        self.alignment_height = max(alignment_clearance, hover_height, calibration.coordinates(drawing_pose[:3])[2])
        if self.aligning:
            # Keep the straight TCP-to-tip segment above the plane during the
            # entire shortest rotation, including an intermediate upward tilt.
            rotation = Rotation.from_rotvec(self.rotvec)
            delta = (Rotation.from_rotvec(calibration.rotvec) * rotation.inv()).as_rotvec()
            samples = Rotation.from_rotvec(np.linspace(0, 1, 181)[:, None] * delta) * rotation
            offset = np.asarray(calibration.tip_offset_m, dtype=float)
            reaches = samples.apply(np.tile(offset, (181, 1))) @ calibration.normal
            between_samples = np.linalg.norm(offset) * np.linalg.norm(delta) / 360
            self.alignment_height = max(self.alignment_height, float(reaches.max()) + between_samples + clearance)
        else:
            self.rotvec = np.asarray(calibration.rotvec).copy()
        self.alignment_phase = "startup-lift"
        self.clearance = clearance
        self.draw_height = draw_height
        self.hover_height = hover_height
        self.speed = speed
        self.lift_speed = lift_speed
        self.arrival_tolerance = arrival_tolerance
        self.max_queue = max_queue
        self.queue: deque[DrawingController.Waypoint] = deque()
        self.planned_rotvec = np.asarray(calibration.rotvec).copy()
        self.touching = False
        self.phase = "lift"
        self.planned_surface = drawing_pose[:3] - calibration.coordinates(drawing_pose[:3])[2] * calibration.normal
        self._append(self.planned_surface + hover_height * calibration.normal, "lift", True)

    def _alignment_step(self, dt: float, measured_pose: np.ndarray | None) -> np.ndarray:
        """Lift first, then rotate about the elevated tip with bounded TCP speed."""
        normal = self.calibration.normal
        offset = np.asarray(self.calibration.tip_offset_m)
        rotation = Rotation.from_rotvec(self.rotvec)
        self.phase = self.alignment_phase
        if self.alignment_phase == "startup-lift":
            equivalent = self.calibration.drawing_pose(np.r_[self.position, self.rotvec])
            height = self.calibration.coordinates(equivalent[:3])[2]
            step = float("inf") if self.realtime else self.lift_speed * dt
            self.position += max(0.0, min(self.alignment_height - height, step)) * normal
            if height + step >= self.alignment_height - 1e-10:
                if measured_pose is None or np.linalg.norm(measured_pose[:3] - self.position) <= self.arrival_tolerance:
                    self.alignment_phase = "align"
        else:
            goal = Rotation.from_rotvec(self.calibration.rotvec)
            delta = (goal * rotation.inv()).as_rotvec()
            angle = np.linalg.norm(delta)
            # |d TCP| <= |offset| * d angle for a rotation about the tip.
            max_angle = min(self.alignment_speed, min(self.speed, self.lift_speed) / max(np.linalg.norm(offset), 1e-9)) * dt
            if self.realtime:
                max_angle = np.deg2rad(5)  # Subdivide rotations for continuous tip compensation.
            fraction = min(1.0, max_angle / max(angle, 1e-15))
            tip = self.position + rotation.apply(offset)
            updated = Rotation.from_rotvec(delta * fraction) * rotation
            self.position = tip - updated.apply(offset)
            self.rotvec = updated.as_rotvec()
            if fraction == 1.0:
                arrived = measured_pose is None or (
                    np.linalg.norm(measured_pose[:3] - self.position) <= self.arrival_tolerance
                    and (Rotation.from_rotvec(measured_pose[3:]).inv() * goal).magnitude() <= np.deg2rad(0.5)
                )
                if arrived:
                    self.aligning = False
                    self.rotvec = np.asarray(self.calibration.rotvec).copy()
        return np.r_[self.position, self.rotvec]

    def _append(self, position: np.ndarray, phase: str, gate: bool = False, rotvec=None):
        rotation = self.planned_rotvec if rotvec is None else np.asarray(rotvec)
        if self.realtime and phase == "draw" and self.queue and self.queue[-1].phase == "draw":
            self.queue.pop()  # Latest continuous stroke target; never replace lift/contact boundaries.
        if len(self.queue) >= self.max_queue:
            raise RuntimeError("Tablet backlog exceeded max_queue; drawing stopped rather than dropping strokes.")
        if (
            self.queue
            and phase == self.queue[-1].phase
            and np.linalg.norm(position - self.queue[-1].position) < 1e-9
            and (Rotation.from_rotvec(rotation).inv() * Rotation.from_rotvec(self.queue[-1].rotvec)).magnitude() < 1e-9
        ):
            return
        self.queue.append(self.Waypoint(np.array(position, dtype=float), phase, gate, np.array(rotation)))

    def add_sample(self, sample: TabletSample):
        surface = self.calibration.map_sample(sample)
        touching = sample.touching and surface is not None
        normal = self.calibration.normal
        rotation = self.planned_rotvec
        if self.follow_tilt and surface is not None:
            rotation = self.calibration.tilt_rotation(sample, self.pen_axis_tcp, self.tilt_signs, self.tilt_limit_deg)
        if self.touching and not touching:
            # Lift at the LAST CONTACT location, not the new hover location.
            self._append(self.planned_surface + self.hover_height * normal, "lift", True)
        if surface is not None:
            if touching:
                if not self.touching:
                    self._append(surface + self.hover_height * normal, "travel", True, rotation)
                    self._append(surface + self.draw_height * normal, "lower", True, rotation)
                else:
                    self._append(surface + self.draw_height * normal, "draw", rotvec=rotation)
            else:
                # Hover positions are replaceable; contact/lift waypoints are not.
                if self.queue and self.queue[-1].phase == "travel":
                    self.queue.pop()
                self._append(surface + self.hover_height * normal, "travel", True, rotation)
            self.planned_surface = surface
            self.planned_rotvec = rotation.copy()
        self.touching = touching

    def retract(self, measured_pose: np.ndarray | None = None):
        """Discard unfinished strokes and retract vertically from measured position."""
        if measured_pose is not None:
            equivalent = self.calibration.drawing_pose(measured_pose)
            self.calibration.validate_start(equivalent, max(self.hover_height, self.alignment_height))
            self.position = np.array(measured_pose[:3], dtype=float)
            self.rotvec = np.array(measured_pose[3:], dtype=float)
        self.aligning = False
        self.queue.clear()
        self.touching = False
        height = self.calibration.coordinates(self.calibration.drawing_pose(np.r_[self.position, self.rotvec])[:3])[2]
        reference = self.calibration.drawing_pose(np.r_[self.position, self.rotvec])[:3]
        target = reference + max(0.0, self.hover_height - height) * self.calibration.normal
        self.planned_rotvec = self.rotvec.copy()
        self._append(target, "lift", True, self.rotvec)

    def step(
        self, dt: float, measured_position: np.ndarray | None = None, measured_pose: np.ndarray | None = None
    ) -> np.ndarray:
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive.")
        if measured_pose is not None:
            measured_position = measured_pose[:3]
        if self.aligning:
            return self._alignment_step(dt, measured_pose)
        remaining = dt
        while self.queue:
            waypoint = self.queue[0]
            self.phase = waypoint.phase
            current = self.calibration.drawing_pose(np.r_[self.position, self.rotvec])[:3]
            delta = waypoint.position - current
            distance = float(np.linalg.norm(delta))
            speed = self.lift_speed if waypoint.phase in ("lift", "lower") else self.speed
            rotation = Rotation.from_rotvec(self.rotvec)
            goal = Rotation.from_rotvec(waypoint.rotvec)
            turn = (goal * rotation.inv()).as_rotvec()
            angle = float(np.linalg.norm(turn))
            offset = np.asarray(self.calibration.tip_offset_m if self.calibration.tip_offset_m is not None else [0, 0, 0])
            if self.realtime:
                duration = max(0.0, -float(delta @ self.calibration.normal)) / self.lift_speed
                fraction = min(1.0, remaining / max(duration, 1e-15), np.deg2rad(5) / max(angle, 1e-15))
            else:
                duration = max((distance + np.linalg.norm(offset) * angle) / speed, angle / self.tilt_speed)
                fraction = min(1.0, remaining / max(duration, 1e-15))
            updated = Rotation.from_rotvec(turn * fraction) * rotation
            reference = current + delta * fraction
            self.position = reference + Rotation.from_rotvec(self.calibration.rotvec).apply(offset) - updated.apply(offset)
            self.rotvec = np.asarray(waypoint.rotvec).copy() if fraction == 1.0 else updated.as_rotvec()
            if fraction < 1.0:
                break
            remaining = max(0.0, remaining - duration)
            if waypoint.gate and measured_position is not None:
                orientation_error = (
                    0.0 if measured_pose is None else (Rotation.from_rotvec(measured_pose[3:]).inv() * goal).magnitude()
                )
                if np.linalg.norm(measured_position - self.position) > self.arrival_tolerance or orientation_error > np.deg2rad(
                    0.5
                ):
                    break
            self.queue.popleft()
            # Give the backend a complete cycle at every contact transition.
            if waypoint.gate or remaining <= 0:
                break
        return np.concatenate((self.position, self.rotvec))
