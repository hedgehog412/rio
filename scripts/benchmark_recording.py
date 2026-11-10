# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

"""Compare recorder storage and throughput without cameras or robot hardware.

Run each mode in a separate process for comparable peak RSS measurements:
    uv run scripts/benchmark_recording.py --mode legacy --output work/legacy
    uv run scripts/benchmark_recording.py --mode direct --output work/direct
"""

import argparse
import json
import logging
import threading
from importlib.metadata import version
from pathlib import Path
from time import perf_counter

import numpy as np
import psutil
import robodm
from loguru import logger

from rio.data.recorder import Recorder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["legacy", "direct"], required=True)
    parser.add_argument("--output", type=Path, required=True, help="New directory for the trajectory and measurements")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--cameras", type=int, default=2)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--depth", action="store_true", help="Include uint16 depth for each camera (stored losslessly)")
    args = parser.parse_args()
    if min(args.frames, args.cameras, args.height, args.width, args.fps) <= 0:
        parser.error("frame count, camera count, dimensions and fps must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    logger.remove()
    logging.disable(logging.CRITICAL)
    # Exercise the synchronous write path without starting middleware workers.
    recorder = Recorder(  # ty: ignore[call-non-callable]
        path=str(args.output),
        start_recording=False,
        direct_encoding=args.mode == "direct",
        video_codec="libx264",
        codec_options={"preset": "fast", "crf": "23"},
    )
    recorder.path = str(args.output / "trajectory.vla")
    recorder._create_datamanager()
    assert recorder.datamanager is not None
    recorder.is_closed = False
    rng = np.random.default_rng(42)
    images = [rng.integers(0, 256, (args.height, args.width, 3), dtype=np.uint8) for _ in range(args.cameras)]
    depth = rng.integers(0, 5000, (args.height, args.width), dtype=np.uint16)
    process = psutil.Process()
    peak_disk = 0
    peak_rss = 0
    stop = threading.Event()

    def sample():
        nonlocal peak_disk, peak_rss
        size = 0
        for path in args.output.iterdir():
            try:
                size += path.stat().st_size
            except FileNotFoundError:
                pass  # Compaction renamed a file between listing and stat.
        peak_disk = max(peak_disk, size)
        peak_rss = max(peak_rss, process.memory_info().rss)

    def monitor():
        while not stop.wait(0.005):
            sample()

    thread = threading.Thread(target=monitor, daemon=True)
    thread.start()
    started = perf_counter()
    try:
        for i in range(args.frames):
            state = {"timestep": i / args.fps, "action": np.full(7, i, np.float32), "instruction": "benchmark"}
            for camera, image in enumerate(images):
                state[f"camera_{camera}/rgb"] = np.roll(image, i, axis=1)
                if args.depth:
                    state[f"camera_{camera}/depth"] = depth
            recorder._record(state, timestamp=i / args.fps)
            sample()
        recording_seconds = perf_counter() - started
        recorder.datamanager.close(compact=args.mode == "legacy")
        close_seconds = perf_counter() - started - recording_seconds
        sample()
    finally:
        stop.set()
        thread.join()
    final_bytes = Path(recorder.path).stat().st_size
    trajectory = robodm.Trajectory(recorder.path, mode="r")
    data = trajectory.load()
    trajectory.close()
    expected_features = 3 + args.cameras * (2 if args.depth else 1)
    assert len(data) == expected_features
    assert all(len(values) == args.frames for values in data.values()), {key: len(value) for key, value in data.items()}
    np.testing.assert_allclose(data["timestep"], np.arange(args.frames) / args.fps, atol=1e-6)
    np.testing.assert_array_equal(data["action"], np.repeat(np.arange(args.frames, dtype=np.float32)[:, None], 7, axis=1))
    for camera in range(args.cameras):
        assert data[f"camera_{camera}/rgb"].shape == (args.frames, args.height, args.width, 3)
        if args.depth:
            np.testing.assert_array_equal(data[f"camera_{camera}/depth"], np.repeat(depth[None], args.frames, axis=0))
    result = {
        "mode": args.mode,
        "frames": args.frames,
        "cameras": args.cameras,
        "resolution": [args.height, args.width],
        "depth": args.depth,
        "target_fps": args.fps,
        "recording_seconds": recording_seconds,
        "encoding_steps_per_second": args.frames / recording_seconds,
        "close_seconds": close_seconds,
        "peak_disk_bytes": peak_disk,
        "final_bytes": final_bytes,
        "peak_rss_bytes": peak_rss,
        "roundtrip_passed": True,
        "versions": {package: version(package) for package in ["av", "robodm", "numpy"]},
        "robodm_source": "f60b3f9cdb5393190cd2d51e238d31002bc2d39a",
        "note": (
            "Unpaced synthetic shifted noise; peak disk sampled every 5 ms and after each step. RSS measured before loading."
        ),
    }
    (args.output / "measurements.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
