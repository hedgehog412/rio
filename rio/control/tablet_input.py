# SPDX-FileCopyrightText: 2026 RIO Developers
# SPDX-License-Identifier: Apache-2.0

"""Read Linux evdev pen reports without a GUI, mouse mapping, or extra dependency."""

import fcntl
import os
import struct
from pathlib import Path

from .tablet import TabletSample

EV_SYN, EV_KEY, EV_ABS = 0, 1, 3
SYN_REPORT, SYN_DROPPED = 0, 3
ABS_X, ABS_Y, ABS_PRESSURE = 0, 1, 24
BTN_TOOL_PEN, BTN_TOUCH = 320, 330
BTN_STYLUS, BTN_STYLUS2 = 331, 332
INPUT_EVENT = struct.Struct("@llHHi")
ABS_INFO = struct.Struct("@iiiiii")


def _read_ioctl(fd: int, number: int, size: int) -> bytes:
    buffer = bytearray(size)
    fcntl.ioctl(fd, (2 << 30) | (size << 16) | (ord("E") << 8) | number, buffer)
    return bytes(buffer)


class TabletEvents:
    """Decode complete SYN_REPORT frames; never confuse proximity with contact."""

    def __init__(self, x_range: tuple[int, int], y_range: tuple[int, int], pressure_range: tuple[int, int] = (0, 1)):
        if any(hi <= lo for lo, hi in (x_range, y_range, pressure_range)):
            raise ValueError("Tablet axis ranges must be increasing.")
        self.ranges = {ABS_X: x_range, ABS_Y: y_range, ABS_PRESSURE: pressure_range}
        self.values = {ABS_X: x_range[0], ABS_Y: y_range[0], ABS_PRESSURE: pressure_range[0]}
        self.touching = False
        self.proximity = False
        self.button1 = False
        self.button2 = False
        self.buffer = b""

    def feed(self, data: bytes) -> list[TabletSample]:
        self.buffer += data
        reports = []
        while len(self.buffer) >= INPUT_EVENT.size:
            seconds, micros, event_type, code, value = INPUT_EVENT.unpack(self.buffer[: INPUT_EVENT.size])
            self.buffer = self.buffer[INPUT_EVENT.size :]
            if event_type == EV_SYN and code == SYN_DROPPED:
                raise RuntimeError("Kernel dropped tablet events; stop and restart to avoid joining strokes.")
            if event_type == EV_ABS and code in self.values:
                self.values[code] = value
            elif event_type == EV_KEY and code == BTN_TOUCH:
                self.touching = bool(value)
            elif event_type == EV_KEY and code == BTN_TOOL_PEN:
                self.proximity = bool(value)
            elif event_type == EV_KEY and code == BTN_STYLUS:
                self.button1 = bool(value)
            elif event_type == EV_KEY and code == BTN_STYLUS2:
                self.button2 = bool(value)
            elif event_type == EV_SYN and code == SYN_REPORT:
                normalized = {axis: (self.values[axis] - lo) / (hi - lo) for axis, (lo, hi) in self.ranges.items()}
                reports.append(
                    TabletSample(
                        u=normalized[ABS_X],
                        v=normalized[ABS_Y],
                        pressure=max(0.0, min(1.0, normalized[ABS_PRESSURE])),
                        touching=self.touching and self.proximity,
                        proximity=self.proximity,
                        timestamp=seconds + micros * 1e-6,
                        button1=self.button1,
                        button2=self.button2,
                    )
                )
        return reports


class LinuxTablet:
    """Nonblocking access to an explicitly selected /dev/input/event* pen node."""

    def __init__(self, path: str):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            self.name = _read_ioctl(self.fd, 0x06, 256).split(b"\0", 1)[0].decode(errors="replace")
            keys = _read_ioctl(self.fd, 0x20 + EV_KEY, 96)
            if not all(keys[key // 8] & (1 << (key % 8)) for key in (BTN_TOOL_PEN, BTN_TOUCH)):
                raise ValueError(f"{path} is not a pen input node with proximity and tip contact.")
            self.buttons_supported = tuple(bool(keys[key // 8] & (1 << (key % 8))) for key in (BTN_STYLUS, BTN_STYLUS2))
            x = ABS_INFO.unpack(_read_ioctl(self.fd, 0x40 + ABS_X, ABS_INFO.size))
            y = ABS_INFO.unpack(_read_ioctl(self.fd, 0x40 + ABS_Y, ABS_INFO.size))
            try:
                pressure = ABS_INFO.unpack(_read_ioctl(self.fd, 0x40 + ABS_PRESSURE, ABS_INFO.size))
                pressure_range = (pressure[1], pressure[2]) if pressure[2] > pressure[1] else (0, 1)
            except OSError:
                pressure_range = (0, 1)
            self.events = TabletEvents((x[1], x[2]), (y[1], y[2]), pressure_range)
            self.events.values.update({ABS_X: x[0], ABS_Y: y[0]})
            keys_now = _read_ioctl(self.fd, 0x18, 96)
            self.events.touching = bool(keys_now[BTN_TOUCH // 8] & (1 << (BTN_TOUCH % 8)))
            self.events.proximity = bool(keys_now[BTN_TOOL_PEN // 8] & (1 << (BTN_TOOL_PEN % 8)))
            self.events.button1 = bool(keys_now[BTN_STYLUS // 8] & (1 << (BTN_STYLUS % 8)))
            self.events.button2 = bool(keys_now[BTN_STYLUS2 // 8] & (1 << (BTN_STYLUS2 % 8)))
        except BaseException:
            os.close(self.fd)
            raise

    def poll(self) -> list[TabletSample]:
        reports = []
        # Bound work per robot cycle. A flooding device must not starve control.
        for _ in range(16):
            try:
                data = os.read(self.fd, INPUT_EVENT.size * 256)
            except BlockingIOError:
                return reports
            if not data:
                raise ConnectionError("Tablet disconnected.")
            reports.extend(self.events.feed(data))
        raise RuntimeError("Tablet input flood; stopping drawing.")

    def close(self):
        os.close(self.fd)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    @classmethod
    def discover(cls) -> tuple[list[tuple[str, str]], list[str]]:
        devices, inaccessible = [], []
        for path in sorted(Path("/dev/input").glob("event*")):
            try:
                with cls(str(path)) as tablet:
                    devices.append((str(path), tablet.name))
            except PermissionError:
                inaccessible.append(str(path))
            except (OSError, ValueError):
                continue
        return devices, inaccessible
