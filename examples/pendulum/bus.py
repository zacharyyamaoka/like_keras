"""The pendulum's I/O box: a servo command in, the measured servo state out, bound to a twin or to the real board.

    PendulumBus(bind="twin")                       # a damped pendulum behind a moteus-like PD loop, no hardware
    PendulumBus(bind="real", can_id=15)            # the rack's board on fdcanusb 272813EF, under its bus lock

Both binds keep the same ports, so the core System, the viewer and every script never change between them.
"""

from __future__ import annotations

import asyncio
import dataclasses
import fcntl
import math
import os
import sys
import time

import numpy as np
from robot_msgs.control import ServoCommand, ServoState
from robot_msgs.std import Header

import like_keras as lk

TAU = 2.0 * math.pi
RACK_ADAPTER = "272813EF"
# WHY this lock: the rack's own runners take it (bam_rack_twin board.hold_bus_lock with BAM_RACK_BUS_LOCK), so a
# rack session and this core can never drive the adapter at once; the full arm holds a different lock on 0607623A.
RACK_LOCK = os.path.expanduser(f"~/.cache/rack-commissioning/bus.{RACK_ADAPTER}.lock")


# Per station: the cap and window this demo uses, and a twin that behaves roughly like it. WHY these numbers
# (bam_rack_twin hardware/tape.py on the rack branch): CAN 11 = JMC60 pendulum, 2 kg x 0.34 m (4.4 N.m of gravity),
# tape cap 3.0 N.m, window -0.65..+1.30 rad with its bottom near +0.26; CAN 15 = the GIM43 small arm, cap 1.0 N.m.
# Board gains in the twin are moteus kp/kd per rev converted to per rad (CAN 11 runs kp 400 / kd 8 since 11-15).
STATIONS = {
    11: {"cap_nm": 3.0, "window_rad": 0.70, "twin": {"inertia": 0.23, "mgl": 4.4, "damping": 0.05, "kp": 63.7, "kd": 1.27}},
    15: {"cap_nm": 1.0, "window_rad": 0.70, "twin": {"inertia": 0.004, "mgl": 0.12, "damping": 0.004, "kp": 6.0, "kd": 0.12}},
}


class Twin:
    """theta'' = (tau - mgl sin(theta - bottom) - b theta') / J, tau = the board's PD + feed-forward, capped."""

    def __init__(self, start: float = 0.0, inertia: float = 0.004, mgl: float = 0.12, damping: float = 0.004,
                 kp: float = 6.0, kd: float = 0.12) -> None:
        self.q, self.qd, self.tau = start, 0.0, 0.0
        self.bottom, self.inertia, self.mgl, self.damping, self.kp, self.kd = start, inertia, mgl, damping, kp, kd

    def step(self, dt: float, position: float, velocity: float, effort: float, cap: float, limp: bool) -> None:
        substeps = 10
        for _ in range(substeps):
            if limp or math.isnan(position):
                tau = 0.0
            else:
                tau = self.kp * (position - self.q) + self.kd * (velocity - self.qd) + effort
                tau = max(-cap, min(cap, tau))
            qdd = (tau - self.mgl * math.sin(self.q - self.bottom) - self.damping * self.qd) / self.inertia
            self.qd += qdd * dt / substeps
            self.q += self.qd * dt / substeps
            self.tau = tau


class PendulumBus(lk.Component):
    command = lk.In(ServoCommand, lagged=True)  # this tick's command goes out at the next exchange
    state = lk.Out(ServoState)

    @dataclasses.dataclass(frozen=True)
    class Params:
        bind: str = "twin"  # "twin" or "real"
        can_id: int = 15
        adapter: str = RACK_ADAPTER
        cap_nm: float = 0.0  # 0 = the station's (STATIONS)
        window_rad: float = 0.0  # targets are clamped to home +- this: a machine limit, not a planner's; 0 = station's
        watchdog_s: float = 0.25

    # ------------------------------------------------------------ lifecycle

    def activate(self):
        station = STATIONS[self.params.can_id]
        self._cap = self.params.cap_nm or station["cap_nm"]
        self._window = self.params.window_rad or station["window_rad"]
        self._name = (f"can{self.params.can_id}",)
        self._seq = 0
        self._clamped = False
        self._last = time.monotonic()
        if self.params.bind == "real":
            self._open_real()
        else:
            self._twin = Twin(start=0.0, **station["twin"])
            self._home = 0.0
        self._last_target = self._home
        self.log.info("%s bus up: home %.2f deg, window +-%.0f deg, cap %.2f N.m", self.params.bind,
                      math.degrees(self._home), math.degrees(self._window), self._cap)

    def deactivate(self):
        self._return_home()
        if self.params.bind == "real":
            try:
                self._loop.run_until_complete(self._controller.set_stop())
            finally:
                fcntl.flock(self._lock, fcntl.LOCK_UN)
                self._lock.close()
                self._loop.close()

    # ------------------------------------------------------------ one exchange per tick

    def forward(self, command):
        now = time.monotonic()
        dt, self._last = now - self._last, now
        if command is None:  # nothing yet (the first tick, or the guard sent nothing): hold home
            position, velocity, effort = self._home, 0.0, 0.0
        else:
            position = float(command.position[0])
            velocity = 0.0 if command.velocity is None else float(command.velocity[0])
            effort = 0.0 if command.feedforward_effort is None else float(command.feedforward_effort[0])
        low, high = self._home - self._window, self._home + self._window
        if not math.isnan(position) and not low <= position <= high:
            if not self._clamped:
                self.log.warning("target %.1f deg outside the window: clamped", math.degrees(position))
            self._clamped = True
            position, velocity = min(high, max(low, position)), 0.0
        else:
            self._clamped = False
        q, qd, tau, temperature, fault = self._exchange(position, velocity, effort, min(dt, 0.05))
        self._seq += 1
        return ServoState(name=self._name, position=np.array([q]), velocity=np.array([qd]), effort=np.array([tau]),
                          temperature=np.array([temperature]), fault=np.array([fault], dtype=np.int64), seq=self._seq,
                          header=Header(stamp=self.now()))

    # ------------------------------------------------------------ the real board

    def _open_real(self):
        import moteus

        self._lock = open(RACK_LOCK, "a")  # noqa: SIM115  WHY: held for the whole activation, released in deactivate
        try:
            fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock.close()
            raise RuntimeError(f"{RACK_LOCK} is held: a rack session owns fdcanusb {self.params.adapter}") from None
        self._moteus = moteus
        self._loop = asyncio.new_event_loop()
        port = f"/dev/serial/by-id/usb-mjbots_fdcanusb_{self.params.adapter}-if00"
        async def connect():  # WHY inside the loop: moteus.Fdcanusb starts its reader task when it is built
            transport = moteus.Fdcanusb(port)
            controller = moteus.Controller(id=self.params.can_id, transport=transport)
            return transport, controller, await controller.query()

        self._transport, self._controller, reply = self._loop.run_until_complete(connect())
        values = reply.values
        fault = int(values.get(moteus.Register.FAULT, 0))
        if fault:
            raise RuntimeError(f"CAN {self.params.can_id} reports fault {fault}: refusing to take over")
        self._home = float(values[moteus.Register.POSITION]) * TAU  # output rev -> rad (the board is in its output frame)

    def _exchange(self, position, velocity, effort, dt=0.01):
        self._last_target = position
        if self.params.bind == "real":
            return self._exchange_real(position, velocity, effort)
        self._twin.step(dt, position, velocity, effort, self._cap, limp=False)
        return self._twin.q, self._twin.qd, self._twin.tau, 30.0, 0

    def _return_home(self, seconds: float = 2.5) -> None:
        """WHY (bam_ws memory can11 envelope trap, 2026-11-18): a board stopped away from its bottom lets the pendulum
        fall limp and swing. So shutting the core down first brings it back to home (min-jerk), then stops it."""
        q = self._exchange(self._last_target, 0.0, 0.0)[0]
        if math.isnan(q) or abs(q - self._home) < math.radians(2):
            return
        print(f"[bus] returning home from {math.degrees(q):.1f} deg before stopping", file=sys.stderr, flush=True)
        began = time.monotonic()
        while (u := (time.monotonic() - began) / seconds) < 1.0:
            s = u**3 * (10 - 15 * u + 6 * u**2)
            self._exchange(q + s * (self._home - q), 30 * u**2 * (1 - u) ** 2 / seconds * (self._home - q), 0.0)
            time.sleep(0.01)
        for _ in range(50):  # settle 0.5 s at home before going limp
            self._exchange(self._home, 0.0, 0.0)
            time.sleep(0.01)
        final = math.degrees(self._exchange(self._home, 0.0, 0.0)[0])
        print(f"[bus] home: {final:.1f} deg (target {math.degrees(self._home):.1f})", file=sys.stderr, flush=True)

    def _exchange_real(self, position, velocity, effort):
        register = self._moteus.Register
        command = self._controller.make_position(
            position=position / TAU, velocity=velocity / TAU, feedforward_torque=effort,
            maximum_torque=self._cap, watchdog_timeout=self.params.watchdog_s, query=True)
        replies = self._loop.run_until_complete(self._transport.cycle([command]))
        values = replies[0].values if replies else {}
        return (float(values.get(register.POSITION, math.nan)) * TAU, float(values.get(register.VELOCITY, math.nan)) * TAU,
                float(values.get(register.TORQUE, math.nan)), float(values.get(register.TEMPERATURE, math.nan)),
                int(values.get(register.FAULT, 0)))
