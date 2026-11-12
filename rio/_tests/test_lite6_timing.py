# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

"""Exercise the real driver interpolator with Lite6's configured precision."""

import numpy as np
import pytest
from rio_hw.interpolators import TrajectoryInterpolator

from examples.cfg.lite6_leader_follower import Lite6LeaderFollowerStation


@pytest.mark.unit
@pytest.mark.parametrize("now", [1794274487.5398, 1794274500.0])
def test_lite6_waypoints_preserve_epoch_precision(now):
    """Speed-limited float32 calculations must not round epoch seconds."""
    config = Lite6LeaderFollowerStation().arm_cfg
    # Match XarmArm's default when a station does not supply a dtype.
    dtype = getattr(config, "dtype", np.float32)
    initial = np.zeros(6, dtype=dtype)
    target = np.full(6, 0.5, dtype=dtype)
    trajectory = TrajectoryInterpolator(times=[now], values=[initial])

    scheduled = trajectory.schedule_waypoint(
        value=target,
        time=now + 0.1,
        max_speed=config.max_motor_speed,
        curr_time=now,
        last_waypoint_time=now,
    )

    minimum_duration = float(np.linalg.norm(target.astype(np.float64) - initial)) / config.max_motor_speed
    expected_arrival = now + minimum_duration
    np.testing.assert_allclose(scheduled.times[-1], expected_arrival, rtol=0, atol=1e-6)
    assert np.all(np.diff(scheduled.times) > 0)
    np.testing.assert_allclose(scheduled(now), initial, rtol=0, atol=1e-12)
    np.testing.assert_allclose(scheduled(expected_arrival), target, rtol=0, atol=1e-6)
    assert minimum_duration > 0.1  # Exercise the speed-limited duration path.


@pytest.mark.unit
def test_lite6_feasible_target_keeps_requested_arrival_time():
    """Small tracking updates arrive on schedule without artificial stretching."""
    config = Lite6LeaderFollowerStation()
    now = 1794274487.5398
    initial = np.zeros(6, dtype=config.arm_cfg.dtype)
    target = np.full(6, 0.001, dtype=config.arm_cfg.dtype)
    requested_arrival = now + 2 / config.freq
    trajectory = TrajectoryInterpolator([now], [initial])
    scheduled = trajectory.schedule_waypoint(
        target,
        requested_arrival,
        max_speed=config.arm_cfg.max_motor_speed,
        curr_time=now,
        last_waypoint_time=now,
    )
    np.testing.assert_allclose(scheduled.times[-1], requested_arrival, rtol=0, atol=1e-6)
    np.testing.assert_allclose(scheduled(np.array([requested_arrival]))[0], target, rtol=0, atol=1e-12)


@pytest.mark.unit
@pytest.mark.parametrize("speed", [0.2, 1.0, 3.1415])
def test_lite6_follower_speed_override_reaches_driver(speed):
    config = Lite6LeaderFollowerStation(follower_max_motor_speed=speed)
    assert config.arm_cfg.max_motor_speed == speed


@pytest.mark.unit
@pytest.mark.parametrize("speed", [0, -1, float("nan"), float("inf"), 3.1416])
def test_lite6_rejects_invalid_speed_overrides(speed):
    with pytest.raises(ValueError, match="follower_max_motor_speed"):
        Lite6LeaderFollowerStation(follower_max_motor_speed=speed)


@pytest.mark.unit
@pytest.mark.parametrize("freq", [0, 251])
def test_lite6_rejects_invalid_target_rates(freq):
    with pytest.raises(ValueError, match="freq"):
        Lite6LeaderFollowerStation(freq=freq)
