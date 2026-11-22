"""Read-only views of the Round 6 truth CSV and native PX4 ULog."""

import csv
import io
import struct
from pathlib import Path

import numpy as np
from pyulog import ULog

FIELDS = (
    "time_usec", "sim_s", "armed", "height_m",
    "position_enu_0", "position_enu_1", "position_enu_2",
    "q_enu_0", "q_enu_1", "q_enu_2", "q_enu_3",
    "velocity_ned_0", "velocity_ned_1", "velocity_ned_2",
)
STAMP, TIME, ARMED, HEIGHT = range(4)
POS, QUAT, VEL = slice(4, 7), slice(7, 11), slice(11, 14)
MODES = {4: "Hold", 17: "Takeoff", 18: "Land"}
SPEEDS = (.25, .5, 1., 2., 4.)


class TruthTail:
    """Only advance past complete CSV lines, including during a writer flush."""

    def __init__(self, path):
        self.path = path
        self.offset = 0
        self.columns = None

    def read(self):
        with self.path.open("rb") as stream:
            stream.seek(self.offset)
            chunk = stream.read(2**20)
        end = chunk.rfind(b"\n") + 1
        if end == 0:
            return np.empty((0, len(FIELDS)))
        lines = chunk[:end].decode("ascii").splitlines()
        self.offset += end
        if self.columns is None:
            header = lines.pop(0).split(",")
            self.columns = [header.index(name) for name in FIELDS]
        rows = [[float(row[i]) for i in self.columns] for row in csv.reader(lines)]
        return np.asarray(rows).reshape(-1, len(FIELDS))


def recorded_truth(path):
    tail = TruthTail(path)
    chunks = []
    while len(rows := tail.read()):
        chunks.append(rows)
    if not chunks:
        raise ValueError("The capture contains no complete truth samples.")
    data = np.concatenate(chunks)
    if not np.isfinite(data).all() or np.any(np.diff(data[:, STAMP]) <= 0):
        raise ValueError("Truth capture has non-finite values or unordered timestamps.")
    return data


def latest_ulog(run):
    archived = run / "px4.ulg"
    if archived.is_file():
        return archived
    return max((run / "rootfs" / "log").glob("*/*.ulg"), default=None)


class FlightLog:
    """Native mode and setpoint, sampled at or before a truth timestamp."""

    TOPICS = ("vehicle_status", "vehicle_local_position_setpoint")

    def __init__(self, path=None):
        self.datasets = {}
        if path is None:
            return
        # Snapshot the active file so its length cannot change during PyULog parsing.
        file = ULog(io.BytesIO(Path(path).read_bytes()), message_name_filter_list=self.TOPICS)
        for entry in file.data_list:
            if entry.name in self.TOPICS and entry.multi_id == 0:
                self.datasets[entry.name] = entry.data

    def at(self, stamp):
        def value(topic, key):
            data = self.datasets.get(topic)
            if data is None:
                return None
            index = int(np.searchsorted(data["timestamp"], stamp, side="right")) - 1
            return data[key][index].item() if index >= 0 else None

        mode = value("vehicle_status", "nav_state")
        down = value("vehicle_local_position_setpoint", "z")
        target = -down if down is not None and np.isfinite(down) else None
        return MODES.get(mode, f"PX4 mode {mode}" if mode is not None else "Awaiting ULog"), target


def load_active_ulog(run, done, publish):
    """Refresh native metadata on a background thread; tolerate an incomplete file tail."""
    seen = None
    while not done.is_set():
        try:
            path = latest_ulog(run)
            if path is not None:
                version = path, path.stat().st_size
                if version != seen:
                    publish(FlightLog(path))
                    seen = version
        except (OSError, ValueError, IndexError, struct.error):
            pass  # The logger may still be writing this record or replacing the archived copy.
        done.wait(1.)


class Timeline:
    def __init__(self, samples, now):
        self.samples = samples
        self.times = samples[:, TIME]
        self.start = float(self.times[0])
        self.end = float(self.times[-1])
        self.anchor = self.start
        self.wall = now
        self.speed_index = SPEEDS.index(1.)
        self.playing = True

    @property
    def speed(self):
        return SPEEDS[self.speed_index]

    def position(self, now):
        elapsed = max(0., now - self.wall) * self.speed if self.playing else 0.
        return min(self.end, self.anchor + elapsed)

    def seek(self, target, now):
        self.anchor = min(self.end, max(self.start, target))
        self.wall = now

    def update(self, now):
        when = self.position(now)
        if when >= self.end:
            self.seek(self.end, now)
            self.playing = False
        index = int(np.searchsorted(self.times, when, side="right")) - 1
        return self.samples[max(0, index)]

    def key(self, key, now):
        position = self.position(now)
        if key == 32:  # GLFW space
            if not self.playing and position == self.end:
                position = self.start
            self.seek(position, now)
            self.playing = not self.playing
        elif key == ord("R"):
            self.seek(self.start, now)
        elif key in (263, 262):  # GLFW left/right
            self.seek(position + (-1. if key == 263 else 1.), now)
        elif ord("0") <= key <= ord("9"):
            fraction = (key - ord("0")) / 9
            self.seek(self.start + fraction * (self.end - self.start), now)
        elif key in (ord("["), ord("]")):
            self.seek(position, now)
            direction = -1 if key == ord("[") else 1
            self.speed_index = min(len(SPEEDS) - 1, max(0, self.speed_index + direction))
