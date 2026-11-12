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
        if not (-0.01 <= u <= 1.01 and -0.01 <= v <= 1.01 and -start_tolerance <= h <= clearance + 0.05):
            raise ValueError("Start TCP must be over the calibrated rectangle, on or above its contact plane.")

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
    ):
        if not all(np.isfinite(x) and x > 0 for x in (clearance, speed, lift_speed, arrival_tolerance)):
            raise ValueError("Clearance, speeds, and arrival tolerance must be finite and positive.")
        if max_queue < 4:
            raise ValueError("max_queue must be at least four.")
        if not np.isfinite(draw_height) or draw_height < 0:
            raise ValueError("draw_height must be finite and nonnegative.")
        hover_height = draw_height + clearance
        calibration.validate_start(np.asarray(start_pose), hover_height)
        self.calibration = calibration
        self.position = np.array(start_pose[:3], dtype=float)
        self.clearance = clearance
        self.draw_height = draw_height
        self.hover_height = hover_height
        self.speed = speed
        self.lift_speed = lift_speed
        self.arrival_tolerance = arrival_tolerance
        self.max_queue = max_queue
        self.queue: deque[DrawingController.Waypoint] = deque()
        self.touching = False
        self.phase = "lift"
        self.planned_surface = self.position - calibration.coordinates(self.position)[2] * calibration.normal
        self._append(self.planned_surface + hover_height * calibration.normal, "lift", True)

    def _append(self, position: np.ndarray, phase: str, gate: bool = False):
        if len(self.queue) >= self.max_queue:
            raise RuntimeError("Tablet backlog exceeded max_queue; drawing stopped rather than dropping strokes.")
        if self.queue and phase == self.queue[-1].phase and np.linalg.norm(position - self.queue[-1].position) < 1e-9:
            return
        self.queue.append(self.Waypoint(np.array(position, dtype=float), phase, gate))

    def add_sample(self, sample: TabletSample):
        surface = self.calibration.map_sample(sample)
        touching = sample.touching and surface is not None
        normal = self.calibration.normal
        if self.touching and not touching:
            # Lift at the LAST CONTACT location, not the new hover location.
            self._append(self.planned_surface + self.hover_height * normal, "lift", True)
        if surface is not None:
            if touching:
                if not self.touching:
                    self._append(surface + self.hover_height * normal, "travel", True)
                    self._append(surface + self.draw_height * normal, "lower", True)
                else:
                    self._append(surface + self.draw_height * normal, "draw")
            else:
                # Hover positions are replaceable; contact/lift waypoints are not.
                if self.queue and self.queue[-1].phase == "travel":
                    self.queue.pop()
                self._append(surface + self.hover_height * normal, "travel", True)
            self.planned_surface = surface
        self.touching = touching

    def retract(self, measured_pose: np.ndarray | None = None):
        """Discard unfinished strokes and retract vertically from measured position."""
        if measured_pose is not None:
            self.calibration.validate_start(measured_pose, self.hover_height)
            self.position = np.array(measured_pose[:3], dtype=float)
        self.queue.clear()
        self.touching = False
        height = self.calibration.coordinates(self.position)[2]
        target = self.position + max(0.0, self.hover_height - height) * self.calibration.normal
        self._append(target, "lift", True)

    def step(self, dt: float, measured_position: np.ndarray | None = None) -> np.ndarray:
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive.")
        remaining = dt
        while self.queue:
            waypoint = self.queue[0]
            self.phase = waypoint.phase
            delta = waypoint.position - self.position
            distance = float(np.linalg.norm(delta))
            speed = self.lift_speed if waypoint.phase in ("lift", "lower") else self.speed
            if distance > speed * remaining + 1e-12:
                self.position += delta * (speed * remaining / distance)
                break
            self.position = waypoint.position.copy()
            remaining = max(0.0, remaining - distance / speed)
            if waypoint.gate and measured_position is not None:
                if np.linalg.norm(measured_position - waypoint.position) > self.arrival_tolerance:
                    break
            self.queue.popleft()
            # Give the backend a complete cycle at every contact transition.
            if waypoint.gate or remaining <= 0:
                break
        return np.concatenate((self.position, self.calibration.rotvec))
