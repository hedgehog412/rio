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
    target = np.full(6, 0.1, dtype=dtype)
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
