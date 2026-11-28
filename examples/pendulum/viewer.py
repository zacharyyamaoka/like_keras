"""The live feed: an attached cell that subscribes the core's `state` and `command` and logs every sample to Rerun.

    .venv/bin/python -m examples.pendulum.viewer --web-port 9096 --save /tmp/pendulum.rrd

It only reads (it never writes an exposed input, so it never takes the lease): it can stay up while scripts come and
go, and it can itself be restarted without touching the core. Open http://127.0.0.1:<web-port> to watch.
"""

from __future__ import annotations

import argparse
import json
import math
import signal

import rerun as rr
import rerun.blueprint as rrb
from robot_msgs.control import ServoCommand, ServoState

import like_keras as lk
from examples.pendulum.script import ADDRESS

ARM_M = 1.0  # the drawn arm's length


class RerunFeed(lk.Component):
    state = lk.In(ServoState, queue=64)  # every 100 Hz sample, not just the latest
    command = lk.In(ServoCommand)  # latest: what the guard sent the board this tick

    def activate(self):
        self._bottom = None
        self._moving = False
        self._still = 0

    def forward(self, state, command):
        if state is None:
            return
        q = float(state.position[0])
        if self._bottom is None:
            self._bottom = q  # WHY the first sample: the core starts with the pendulum at rest, hanging at its bottom
            _styles()
            rr.log("events", rr.TextLog(f"viewer attached: pendulum at {math.degrees(q):.1f} deg", level="INFO"))
        rr.set_time("time", timestamp=state.header.stamp)
        rr.log("joint/position_deg/measured", rr.Scalars(math.degrees(q)))
        rr.log("joint/velocity_deg_s/measured", rr.Scalars(math.degrees(float(state.velocity[0]))))
        rr.log("joint/effort_nm/measured", rr.Scalars(float(state.effort[0])))
        rr.log("pendulum/measured", _arm(q - self._bottom, ARM_M), rr.LineStrips2D.from_fields(colors=[MEASURED], radii=[0.03]))
        if command is not None:
            target = float(command.position[0])
            speed = 0.0 if command.velocity is None else float(command.velocity[0])
            rr.log("joint/position_deg/commanded", rr.Scalars(math.degrees(target)))
            rr.log("joint/velocity_deg_s/commanded", rr.Scalars(math.degrees(speed)))
            if command.max_effort is not None:
                cap = float(command.max_effort[0])
                rr.log("joint/effort_nm/cap", rr.Scalars([cap, -cap]))
            rr.log("pendulum/commanded", _arm(target - self._bottom, ARM_M * 0.95),
                   rr.LineStrips2D.from_fields(colors=[COMMANDED + [180]], radii=[0.015]))
            self._events(speed, target)
        return

    def _events(self, speed: float, target: float) -> None:
        if abs(speed) > 1e-6:
            if not self._moving:
                rr.log("events", rr.TextLog(f"command moving (from {math.degrees(target):.1f} deg)", level="INFO"))
            self._moving, self._still = True, 0
        elif self._moving:
            self._still += 1
            if self._still > 50:  # 0.5 s at rest: the plan ended (or its lease lapsed and the core holds)
                rr.log("events", rr.TextLog(f"holding at {math.degrees(target):.1f} deg", level="INFO"))
                self._moving = False


MEASURED, COMMANDED, CAP = [230, 120, 40], [60, 140, 255], [200, 60, 60]


def _styles() -> None:
    """One colour per meaning in every view: orange = what the board measured, blue = what the core commanded."""
    for quantity in ("position_deg", "velocity_deg_s", "effort_nm"):
        rr.log(f"joint/{quantity}/measured", rr.SeriesLines(colors=[MEASURED], names=["measured"], widths=[2]), static=True)
        rr.log(f"joint/{quantity}/commanded", rr.SeriesLines(colors=[COMMANDED], names=["commanded"], widths=[2]), static=True)
    rr.log("joint/effort_nm/cap", rr.SeriesLines(colors=[CAP, CAP], names=["+cap", "-cap"], widths=[1]), static=True)
    rr.log("pendulum/pivot", rr.Points2D([[0.0, 0.0]], radii=[0.05], colors=[[80, 80, 80]]), static=True)


def _arm(angle: float, length: float) -> rr.LineStrips2D:
    tip = [length * math.sin(angle), length * math.cos(angle)]  # Rerun's 2D y points down: the bob hangs below
    return rr.LineStrips2D([[[0.0, 0.0], tip]])


def blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="pendulum", name="pendulum (orange measured, blue commanded)",
                                  visual_bounds=rrb.VisualBounds2D(x_range=[-1.2, 1.2], y_range=[-0.3, 1.2])),
                rrb.TextLogView(origin="events", name="events"),
                row_shares=[3, 2],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="joint/position_deg", name="position (deg)"),
                rrb.TimeSeriesView(origin="joint/velocity_deg_s", name="velocity (deg/s)"),
                rrb.TimeSeriesView(origin="joint/effort_nm", name="torque (N.m) and cap"),
            ),
            column_shares=[2, 3],
        ),
        rrb.TimePanel(state="collapsed"),
        collapse_panels=True,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--address", default=ADDRESS)
    parser.add_argument("--grpc-port", type=int, default=9886)
    parser.add_argument("--web-port", type=int, default=9096)
    parser.add_argument("--save", default="", help="also write the whole feed to this .rrd")
    arguments = parser.parse_args(argv)
    rr.init("lk_pendulum", recording_id="pendulum-live")
    server = rr.GrpcServerSink(bind_ip="127.0.0.1", port=arguments.grpc_port)  # WHY loopback: the rig is not public
    sinks = [server] + ([rr.FileSink(arguments.save)] if arguments.save else [])
    rr.set_sinks(*sinks, default_blueprint=blueprint())
    url = server.uri
    rr.serve_web_viewer(web_port=arguments.web_port, open_browser=False, connect_to=url)
    s = lk.System("viewer")
    arm = s.add(lk.attach(arguments.address), "pendulum")
    s.add(RerunFeed(), "feed", state=arm.state, command=arm.command)
    dep = lk.Deployment(s)
    dep.place(s.pendulum, hz=100)
    dep.place(s.feed)  # rate-less: once per state sample
    dep.start()
    print(json.dumps({"ready": True, "web": f"http://127.0.0.1:{arguments.web_port}/?url={url}",
                      "grpc": url, "rrd": arguments.save or None}), flush=True)
    try:
        signal.sigwait({signal.SIGTERM, signal.SIGINT})
    finally:
        dep.stop()
    return 0


if __name__ == "__main__":
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGTERM, signal.SIGINT})
    from examples.pendulum import viewer  # noqa: PLW0406  WHY: components must name a module, never __main__ (cell.py)

    raise SystemExit(viewer.main())
