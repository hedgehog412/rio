# Lite6 leader–follower recording trial

This uses `Lite6LeaderFollowerStation` from `main` with the direct-encoding changes. Run these commands on your existing robot workstation checkout: its custom hardware drivers must already exist in `third_party/rio-hw`. That directory is ignored by Git and is not included in a fresh clone. Keep that local copy, including its Gphoto camera driver.

## Update and check the setup

```bash
cd /path/to/rio
git fetch origin
git switch feat/direct-recording
git pull --ff-only origin feat/direct-recording

test -f third_party/rio-hw/pyproject.toml
uv sync

STATION=Lite6LeaderFollowerStation uv run -m examples.teleop_leader_follower --help
uv run pytest rio/_tests/test_direct_recording.py rio/_tests/test_recorder.py
```

If the local driver directory is missing, restore your working `rio-hw` checkout there before `uv sync`. The stock cloud-installed driver does not include Gphoto support, so it cannot validate your complete camera setup.

Confirm the settings in `examples/cfg/lite6_leader_follower.py` match the workstation:

- Follower: `192.168.2.176`, Lite6 joint-position controller.
- Leader: `192.168.1.168`, Lite6 guide controller.
- RealSense serial: `238222074772`.
- Gphoto ports: left `usb:001,048`, right `usb:001,033`. USB bus/device numbers can change after reconnecting; check with `gphoto2 --auto-detect`.

The loop defaults to 15 Hz, with 30 Hz camera acquisition. Recording samples camera observations at the outer loop rate. Gello-specific leader smoothing is disabled for this real-arm setup; the follower's configured joint filter remains enabled. Hardware addresses and camera configurations are fixed CLI fields, so edit the station file if they need changing.

The follower uses `dtype="float64"`. With NumPy 2, a speed-limited duration calculated from float32 joint data can cause an epoch timestamp to be evaluated in float32, rounding it by many seconds. This can trigger `Times must be monotonically increasing` or schedule a waypoint far from its intended arrival time. The station's float64 configuration preserves timestamp precision through the existing driver without changing its speed limit or removing timestamp checks. The regression test uses the actual driver interpolator and verifies arrival times for timestamps that float32 would round both backward and forward.

## Teleoperation without recording

```bash
STATION=Lite6LeaderFollowerStation uv run -m examples.teleop_leader_follower \
  --recorder None \
  --instruction "Lite6 leader-follower motion test"
```

The script starts the configured camera and robot nodes, checks leader/follower alignment and prompts for Enter. Align the joint positions before starting. Press Ctrl-C to exit.

## Compare recording modes

Use separate directories and comparable motions and episode lengths. Keep all cameras producing data from the first recorded step: direct encoding fixes the feature names at that point.

Deferred encoding:

```bash
STATION=Lite6LeaderFollowerStation uv run -m examples.teleop_leader_follower \
  --recorder-cfg.path data/lite6_deferred_trial/ \
  --recorder-cfg.no-direct-encoding \
  --instruction "Lite6 encoding comparison"
```

Direct encoding:

```bash
STATION=Lite6LeaderFollowerStation uv run -m examples.teleop_leader_follower \
  --recorder-cfg.path data/lite6_direct_trial/ \
  --recorder-cfg.direct-encoding \
  --instruction "Lite6 encoding comparison"
```

Each invocation records one episode after Enter. Press Ctrl-C to stop and wait for the save to finish before starting another run. Both commands retain the configured H.264 codec, CRF 23 and fast preset.

In another terminal, from the same checkout, watch disk use during recording and saving:

```bash
watch -n 1 'du -sh data/lite6_deferred_trial data/lite6_direct_trial'
```

A directory may be absent until its first run. Compare peak sizes while recording; the final RGB files should be much closer in size. Check that saved files load in your usual replay/export workflow, including camera frame counts and action/timestamp alignment.

The integration checks and CLI parsing were verified in the cloud workspace. This does not establish physical robot, Gphoto or RealSense operation on the workstation.
