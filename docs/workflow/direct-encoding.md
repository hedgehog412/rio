# Try direct video encoding

RIO's published `robodm==0.1.0` dependency writes pickled image arrays during recording and compresses them when saving. Long episodes therefore need much more temporary disk space than their final `.vla` files. Raising the PyAV version alone does not change that write path.

This branch pins Robo-DM to commit `f60b3f9cdb5393190cd2d51e238d31002bc2d39a`, which supports `force_direct_encoding`, and PyAV to `14.2.0`. Later PyAV 14 releases lack Linux wheels. The plain raw codec remains the default for depth and other non-image data, preserving their values. The newer dependency's PyArrow compaction path did not produce readable mixed-data files in our checks.

## Enable it

From this branch:

```bash
uv sync
uv run pytest rio/_tests/test_direct_recording.py rio/_tests/test_recorder.py
```

Enable direct encoding in your station's recorder configuration:

```python
RecorderCfg(path="data/direct_trial/", direct_encoding=True)
```

Or use the command line, substituting your station and teleoperation module:

```bash
STATION=Xarm7EEFStation uv run -m examples.teleop_eef \
  --recorder-cfg.path data/direct_trial/ \
  --recorder-cfg.direct-encoding
```

The option defaults to false. Use `--recorder-cfg.no-direct-encoding` and a separate output directory for a comparison with deferred encoding.

Direct encoding registers every stream from the first nonempty step before writing any packets. Without this initialization, Robo-DM's container recreation can lose the first video frame. Supply all feature names in the first step and keep image dimensions and dtypes stable for an episode. Introducing new feature names logs an error and rejects that step; start a new episode when the schema changes.

Saving flushes buffered video packets and closes the container without the compaction pass. Always stop and save normally before inspecting a file. Video encoders and container muxing still buffer data; direct encoding does not promise crash recovery or that every frame is immediately durable.

H.264 RGB is lossy, as in RIO's existing default. Depth, actions and other raw fields retain their exact array values. Explicit `video_codec="rawvideo"` remains lossless and does not reduce RGB storage. Depth is also stored as raw data, so depth cameras can still consume substantial disk space.

## Measure without hardware

Run the modes sequentially, using new output directories each time:

```bash
uv run scripts/benchmark_recording.py --mode legacy --output work/legacy
uv run scripts/benchmark_recording.py --mode direct --output work/direct
# Repeat with depth:
uv run scripts/benchmark_recording.py --mode legacy --depth --output work/legacy_depth
uv run scripts/benchmark_recording.py --mode direct --depth --output work/direct_depth
```

Each invocation records 120 steps from two synthetic 640x480 RGB cameras at 30 Hz timestamps, measures disk use during recording and saving, and validates a decoded round trip. `measurements.json` includes peak disk bytes, peak process RSS before loading, unpaced recording throughput and close time. Disk is sampled every 5 ms and after each step, so brief peaks may be missed. The legacy comparison uses the pinned dependency with deferred encoding, rather than a separate installation of PyPI 0.1.0.

The synthetic images are shifted, seeded noise. They are reproducible but do not predict real camera compression ratios or end-to-end teleoperation latency. Throughput includes synthetic frame generation and writes; it measures the synchronous recorder path without middleware queues or camera acquisition.

A sequential run in this cloud workspace with PyAV 14.2.0 and Python 3.11 produced the following results (decimal MB). Both modes passed decoded frame-count and non-image integrity checks:

| Input | Mode | Peak disk | Final file | Steps/s | Close time | Peak RSS |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Two RGB cameras | Deferred | 223.37 MB | 2.34 MB | 67.7 | 1.06 s | 290.35 MB |
| Two RGB cameras | Direct | 2.34 MB | 2.34 MB | 134.2 | 0.16 s | 314.77 MB |
| Two RGB + depth cameras | Deferred | 518.62 MB | 149.85 MB | 40.1 | 1.24 s | 339.55 MB |
| Two RGB + depth cameras | Direct | 149.85 MB | 149.85 MB | 51.0 | 0.19 s | 349.77 MB |

Direct encoding reduced peak disk about 99% for RGB and 71% with raw depth. Process memory did not decrease.

Validation: the recorder test files passed all 9 tests. Changed-file Ruff, formatting and typo checks passed. The complete suite had 67 passing tests, 17 skips and 11 policy-test failures; the same 11 failures reproduced on the unchanged branch. Repository-wide type checking reported 222 diagnostics on both branches. Global `make check` also found existing lint failures in unrelated files. These baseline failures remain unresolved by this recording change.

## Test on your collection machine

1. Run the benchmark at your actual resolution and camera count using `--height`, `--width`, `--cameras`, `--frames` and `--fps`. Add `--depth` if applicable.
2. Record short comparable demonstrations with direct encoding disabled and enabled into separate directories. Sample both dataset-directory size and available filesystem space during recording and saving. Use the same codec and codec options in both runs.
3. Confirm each saved trajectory loads in your replay/export workflow. Check every camera's frame count, timestamp alignment, action count, depth dtype and several decoded frames.
4. Try a normal-length episode. Check CPU usage, control-loop timing and recorder queue backlog. Encoding must sustain the incoming camera rate; smaller files alone do not establish that the recorder keeps up.

Keep the deferred-mode comparison until these checks pass on your hardware. Changing the dependency also affects dataset loading and export, so check representative existing files before switching a collection environment.
