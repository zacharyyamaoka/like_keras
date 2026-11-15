"""The fake moteus bus in Python: kpi/tasks/arm_cell/fake_bus.hpp, the same board file and the same rules.

- `FakeBoards` maps the board file (or a private buffer) and is what the tests read: the boards a C++ or a Python
  bus fed, the event log of every frame (who sent it, query / stop / position), and the board watchdog applied at
  the moment you look (`modes(at=...)`), so a killed node's boards are in timeout mode 0.25 s after its last frame.
- `FakeBusCore` is the bus side: BUS_LOCK (flock) before the board file opens (R1, T7), take_over (a stop only out of
  mode 11, then a hold in the same tick: R2, T6), one exchange per tick (`exchange_s` on the wire), stop_feeding.
WHY no real bus: kpi/ runs beside a powered arm; `twin = False` is refused, never opened (NO HARDWARE).
"""

from __future__ import annotations

import fcntl
import mmap
import os
import time

import numpy as np
from robot_msgs.control import ServoState
from robot_msgs.std import Header

MAGIC = 0x42464B4C  # "LKFB"
VERSION = 1
BOARDS = 16
RING = 65536
MODE_STOPPED, MODE_FAULT, MODE_POSITION, MODE_TIMEOUT = 0, 1, 10, 11
EVENT_QUERY, EVENT_STOP, EVENT_POSITION = 1, 2, 3

HEADER = np.dtype([("magic", "<u4"), ("version", "<u4"), ("joints", "<u4"), ("reserved", "<u4"),
                   ("watchdog_s", "<f8"), ("events", "<u8"), ("lapses", "<u8"), ("reserved2", "<u8")])
BOARD = np.dtype([("q", "<f8"), ("qd", "<f8"), ("last_fed", "<f8"), ("mode", "<i8"), ("fault", "<i8"), ("reserved", "<f8")])
EVENT = np.dtype([("t", "<f8"), ("wall", "<f8"), ("q0", "<f8"), ("pid", "<i4"), ("kind", "<i4"), ("mode_after", "<i4"),
                  ("reserved", "<i4")])
BOARDS_AT = HEADER.itemsize
RING_AT = BOARDS_AT + BOARDS * BOARD.itemsize
SIZE = RING_AT + RING * EVENT.itemsize
assert HEADER.itemsize == 48 and BOARD.itemsize == 48 and EVENT.itemsize == 40 and RING_AT == 816


class FakeBoards:
    """The boards behind the fake bus: a shared file (a test reads what a node in another process fed) or private."""

    def __init__(self, path: str | os.PathLike | None = None, joints: int | None = None) -> None:
        if path:
            self._fd = os.open(os.fspath(path), os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o644)
            os.ftruncate(self._fd, SIZE)
            self._memory = mmap.mmap(self._fd, SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        else:
            self._fd = None
            self._memory = bytearray(SIZE)
        self.header = np.frombuffer(self._memory, HEADER, 1, 0)
        self.boards = np.frombuffer(self._memory, BOARD, BOARDS, BOARDS_AT)
        self.ring = np.frombuffer(self._memory, EVENT, RING, RING_AT)
        if joints is not None and (self.header["magic"][0] != MAGIC or self.header["joints"][0] != joints):
            np.frombuffer(self._memory, np.uint8)[:] = 0
            self.header["magic"], self.header["version"], self.header["joints"] = MAGIC, VERSION, joints

    @property
    def joints(self) -> int:
        return int(self.header["joints"][0])

    def modes(self, at: float | None = None) -> list[int]:
        """Each board's mode, with the watchdog applied at board time `at` (epoch s; default: now)."""
        at = time.time() if at is None else at
        watchdog = float(self.header["watchdog_s"][0])
        modes = []
        for board in self.boards[: self.joints]:
            mode = int(board["mode"])
            if mode == MODE_POSITION and board["last_fed"] > 0.0 and at - board["last_fed"] > watchdog:
                mode = MODE_TIMEOUT
            modes.append(mode)
        return modes

    def events(self) -> np.ndarray:
        """Every frame still in the ring, oldest first."""
        total = int(self.header["events"][0])
        if total <= RING:
            return self.ring[:total].copy()
        start = total % RING
        return np.concatenate([self.ring[start:], self.ring[:start]])

    def close(self) -> None:
        del self.header, self.boards, self.ring
        if self._fd is not None:
            self._memory.close()
            os.close(self._fd)
            self._fd = None


class FakeBusCore:
    def __init__(self) -> None:
        self._lock_fd = None
        self._boards: FakeBoards | None = None
        self._seq = 0
        self._hold = None  # (position, velocity) of the frame a missing command repeats (R5)

    def open_exclusive(self, params) -> None:
        """R1: BUS_LOCK first, then the board file ("the port"). Raises (the activate refusal) before any frame."""
        if not params.twin:
            raise RuntimeError("REFUSED: kpi has no real bus; set twin = true (the fake bus)")
        if not params.lock_path:
            raise RuntimeError("REFUSED: the bus needs lock_path (BUS_LOCK, R1)")
        if not 1 <= params.joints <= BOARDS:
            raise RuntimeError("REFUSED: joints must be 1-16")
        fd = os.open(params.lock_path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            holder = os.pread(fd, 31, 0).decode(errors="replace")
            os.close(fd)
            raise RuntimeError(f"REFUSED: bus lock {params.lock_path} held by pid {holder}") from None
        os.ftruncate(fd, 0)
        os.pwrite(fd, str(os.getpid()).encode(), 0)
        self._lock_fd = fd
        self._joints = params.joints
        self._exchange_s = params.exchange_s
        self._boards = FakeBoards(params.board_file or None, params.joints)
        self._boards.header["watchdog_s"] = params.watchdog_s
        self._names = tuple(f"j{joint + 1}" for joint in range(params.joints))

    def take_over(self) -> None:
        """R2: a stop only when a board latched its watchdog (mode 11), then a hold in the same tick."""
        boards = self._boards.boards[: self._joints]
        latched = bool(np.any(boards["mode"] == MODE_TIMEOUT))
        self._record(0.0, EVENT_QUERY)
        if latched:
            boards["mode"], boards["qd"] = MODE_STOPPED, 0.0
            self._record(0.0, EVENT_STOP)
        boards["mode"] = np.where(boards["mode"] == MODE_FAULT, MODE_FAULT, MODE_POSITION)
        boards["qd"], boards["last_fed"] = 0.0, 0.0  # the watchdog counts from the first tick's frame
        self._hold = (boards["q"].copy(), np.zeros(self._joints))
        self._record(0.0, EVENT_POSITION)

    def exchange(self, now: float, command) -> ServoState:
        """One frame: the reply (what the boards held before it), then `command` (or the last one sent) applied."""
        wall = time.time()
        header, boards = self._boards.header, self._boards.boards[: self._joints]
        lapsed = (boards["mode"] == MODE_POSITION) & (boards["last_fed"] > 0.0) & (now - boards["last_fed"] > header["watchdog_s"][0])
        if np.any(lapsed):
            boards["mode"][lapsed] = MODE_TIMEOUT
            header["lapses"] += int(np.count_nonzero(lapsed))
        self._seq += 1
        state = ServoState(name=self._names, position=boards["q"].copy(), velocity=boards["qd"].copy(),
                           fault=boards["fault"].copy(), seq=self._seq, header=Header(stamp=now))
        if command is not None:
            velocity = np.zeros(len(command.position)) if command.velocity is None else command.velocity
            self._hold = (np.asarray(command.position), np.asarray(velocity))
        position, velocity = self._hold
        count = min(self._joints, len(position))
        feedable = (boards["mode"][:count] != MODE_TIMEOUT) & (boards["mode"][:count] != MODE_FAULT)
        for joint in np.flatnonzero(feedable):
            boards["mode"][joint] = MODE_POSITION
            if not np.isnan(position[joint]):
                boards["q"][joint] = position[joint]
            boards["qd"][joint] = velocity[joint]
            boards["last_fed"][joint] = now
        self._record(now, EVENT_POSITION, wall)
        if self._exchange_s > 0.0:
            time.sleep(self._exchange_s)  # the frame's time on the wire; device emulation, never a test's sleep
        return state

    def stop_feeding(self) -> None:
        """Shutdown is to stop feeding: each board falls into its timeout mode on its own (R2). Never a stop frame."""
        if self._boards is not None:
            self._boards.close()
            self._boards = None
        if self._lock_fd is not None:
            os.close(self._lock_fd)  # releases the flock
            self._lock_fd = None

    @property
    def lapses(self) -> int:
        return 0 if self._boards is None else int(self._boards.header["lapses"][0])

    def _record(self, now: float, kind: int, wall: float | None = None) -> None:
        header = self._boards.header
        index = int(header["events"][0]) % RING
        board = self._boards.boards[0]
        self._boards.ring[index] = (now, time.time() if wall is None else wall, float(board["q"]), os.getpid(), kind,
                                    int(board["mode"]), 0)
        header["events"] += 1
