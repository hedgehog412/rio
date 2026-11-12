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
    """Three TCP contact positions, all taught with the same pen orientation.

    The fixed pen offset is absorbed into these TCP positions. The robot's TCP
    configuration and pen mounting must remain unchanged after calibration.
    """

    origin: list[float]
    corner_u: list[float]
    corner_v: list[float]
    rotvec: list[float]
    normal_sign: int = 1
    tablet_bounds: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)

    def __post_init__(self):
        points = np.asarray([self.origin, self.corner_u, self.corner_v, self.rotvec], dtype=float)
        if points.shape != (4, 3) or not np.isfinite(points).all():
            raise ValueError("Calibration requires finite 3D points and a rotation vector.")
        if self.normal_sign not in (-1, 1):
            raise ValueError("normal_sign must be -1 or 1.")
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
        if not (-0.01 <= u <= 1.01 and -0.01 <= v <= 1.01 and -0.0005 <= h <= clearance + 0.05):
            raise ValueError("Start TCP must be over the calibrated rectangle, on or above its contact plane.")

    def save(self, path: str | Path):
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

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
