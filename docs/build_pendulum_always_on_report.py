"""Build the always-on pendulum report (examples/pendulum) from one run's evidence directory.

    /home/bam/like_keras/examples/pendulum/.venv/bin/python /home/bam/like_keras/docs/build_pendulum_always_on_report.py /archive/datasets/bam_rig/rack_2026-11-28/lk_pendulum_always_on

Reads: core.mcap (the core's exposed state + command, like_keras.record), rack_d405.mp4 + .json (the rack camera, per
frame wall times), viewer.webm + sync.json (the Rerun web viewer as recorded headless; its wall offset, found by
cross-correlating the drawn pendulum with the measured angle), events.json (what attached when).
Writes /home/bam/like_keras/reports/media/lk-pendulum-always-on-2026-11-28.html and its media folder beside it.
"""

from __future__ import annotations

import base64
import html
import json
import pathlib
import subprocess
import sys

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO / "src"), str(REPO)]
OUT = REPO / "reports" / "media"
NAME = "lk-pendulum-always-on-2026-11-28"
MEDIA = OUT / NAME
ORANGE, BLUE = "#e67828", "#3c8cff"


def series(evidence: pathlib.Path) -> dict:
    import robot_msgs

    from like_keras import codec, record

    channels, messages = record.read(evidence / "core.mcap")
    types, streams = robot_msgs.message_types(), {}
    for channel, _, frame in messages:
        topic, kind = channels[channel]
        streams.setdefault(topic, []).append(codec.decode(frame, types[kind]))
    states, commands = streams["pendulum.state"], streams["pendulum.command"]
    t = np.array([s.header.stamp for s in states])
    tc = np.array([c.header.stamp for c in commands])
    return {"t": t, "q": np.degrees([s.position[0] for s in states]), "tau": np.array([s.effort[0] for s in states]),
            "fault": np.array([s.fault[0] for s in states]), "c": np.interp(t, tc, np.degrees([c.position[0] for c in commands])),
            "cap": float(commands[-1].max_effort[0])}


def numbers(s: dict) -> dict:
    t, q, c = s["t"], s["q"], s["c"]
    dt = np.diff(t) * 1e3
    moving = np.abs(np.gradient(c, t)) > 1.0
    error = q - c
    return {"duration_s": t[-1] - t[0], "samples": len(t), "dt_p50_ms": np.median(dt), "dt_p99_ms": np.percentile(dt, 99),
            "dt_max_ms": dt.max(), "gaps_over_20ms": int((dt > 20).sum()), "faults": sorted(set(s["fault"].tolist())),
            "tau_max": np.abs(s["tau"]).max(), "cap": s["cap"], "rms_moving": np.sqrt(np.mean(error[moving] ** 2)),
            "peak_moving": np.abs(error[moving]).max(), "rms_rest": np.sqrt(np.mean(error[~moving] ** 2)),
            "home": q[0], "range": (q.min() - q[0], q.max() - q[0])}


def caption(events: list, wall: float) -> str:
    for start, end, text in events:
        if start <= wall < end:
            return text
    return "no script attached: the core holds, the feed keeps streaming"


def composite(evidence: pathlib.Path, start: float, end: float, out: pathlib.Path, speed: float = 1.0, fps: int = 25):
    """Rack camera (left, cropped to CAN 11) beside the Rerun feed (right), frame-picked by wall time, captioned."""
    events = json.loads((evidence / "events.json").read_text())
    base = events["epoch_base"]
    events = [(base + a, base + b, text) for a, b, text in events["events"]]
    walls = np.array(json.loads((evidence / "rack_d405.mp4.json").read_text())["wall"])
    webm0 = json.loads((evidence / "sync.json").read_text())["webm_epoch0"]
    camera, viewer = cv2.VideoCapture(str(evidence / "rack_d405.mp4")), cv2.VideoCapture(str(evidence / "viewer.webm"))
    viewer_fps = viewer.get(cv2.CAP_PROP_FPS)
    cam_at, view_at = _Sequential(camera), _Sequential(viewer)
    raw = out.with_suffix(".raw.mp4")
    width, height = 480 + 1067 + 1, 600 + 84
    writer = cv2.VideoWriter(str(raw), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    wall = start
    while wall < end:
        cam = cam_at(int(np.clip(np.searchsorted(walls, wall), 0, len(walls) - 1)))[120:720, 560:1040]
        view = view_at(max(0, int((wall - webm0) * viewer_fps)))
        view = cv2.resize(view, (1067, 600), interpolation=cv2.INTER_AREA)
        frame = np.full((height, width, 3), 250, np.uint8)
        frame[84:, :480], frame[84:, 481:] = cam, view
        stamp = __import__("time").strftime("%H:%M:%S", __import__("time").localtime(wall))
        cv2.putText(frame, f"{stamp}  {caption(events, wall)}", (14, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (30, 30, 30), 2,
                    cv2.LINE_AA)
        for text, x in (("rack camera (D405) - CAN 11", 10), ("Rerun live feed - an attached viewer (orange measured, blue commanded)", 491)):
            cv2.putText(frame, text, (x, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (90, 90, 90), 1, cv2.LINE_AA)
        writer.write(frame)
        wall += speed / fps
    writer.release()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(raw), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "28", "-preset", "slow", "-movflags", "+faststart", "-an", str(out)], check=True)
    raw.unlink()
    return out


def gif(video: pathlib.Path) -> pathlib.Path:
    out = video.with_suffix(".gif")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(video), "-vf",
                    "fps=5,scale=760:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=48:stats_mode=diff[p];[b][p]paletteuse=dither=none",
                    str(out)], check=True)
    return out


class _Sequential:
    """Frame `index` of a video read forwards only (WHY: the two videos decode to ~20 GB; indices only ever grow)."""

    def __init__(self, capture) -> None:
        self.capture, self.index, self.frame = capture, -1, None

    def __call__(self, index: int):
        while self.index < index:
            ok, frame = self.capture.read()
            if not ok:
                break
            self.index, self.frame = self.index + 1, frame
        return self.frame


def plot(s: dict, events: dict, out: pathlib.Path) -> pathlib.Path:
    base = events["epoch_base"]
    t0 = s["t"][0]
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(12, 5.2), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    for start, end, text in events["events"]:
        if text.startswith("script") and text.split(":")[0].endswith("attached"):  # a script held the write lease
            for axis in (top, bottom):
                axis.axvspan(base + start - t0, base + end - t0, color="#e4edff", zorder=0)
            label = text.split(":")[0].replace("script ", "#").replace(" attached", "")
            top.text(base + start - t0 + 0.3, 30, label, fontsize=10, color="#335")
        elif "SIGTERM" in text:
            top.axvline(base + start - t0, color="#888", lw=0.8, ls="--")
    top.plot(s["t"] - t0, s["c"] - s["q"][0], color=BLUE, lw=1.4, label="commanded (guard.safe)")
    top.plot(s["t"] - t0, s["q"] - s["q"][0], color=ORANGE, lw=1.2, label="measured (bus.state)")
    top.set_ylabel("angle from home (deg)")
    top.legend(loc="lower left", fontsize=9)
    top.set_ylim(-34, 34)
    bottom.plot(s["t"] - t0, s["tau"], color=ORANGE, lw=1.0, label="measured torque")
    bottom.axhline(s["cap"], color="#c83c3c", lw=0.8, label=f"cap {s['cap']:.1f} N.m")
    bottom.axhline(-s["cap"], color="#c83c3c", lw=0.8)
    bottom.set_ylabel("N.m")
    bottom.set_xlabel("seconds since the viewer attached (one core, never restarted)")
    bottom.legend(loc="lower left", fontsize=9)
    for axis in (top, bottom):
        axis.grid(alpha=0.3)
        axis.spines[["top", "right"]].set_visible(False)
    figure.tight_layout()
    figure.savefig(out, dpi=110)
    return out


def data_uri(path: pathlib.Path, kind: str) -> str:
    return f"data:{kind};base64," + base64.b64encode(path.read_bytes()).decode()


def page(n: dict, evidence: pathlib.Path) -> str:
    e = html.escape
    rel = NAME
    stills = sorted(evidence.glob("still_*.png"))
    still = stills[min(len(stills) - 1, 16)]
    (MEDIA / "viewer-still.png").write_bytes(still.read_bytes())
    files = [("core", "examples/pendulum/core.py", "the always-on core: bus -> JTC -> guard at 100 Hz, exposing trajectory / state / command"),
             ("bus", "examples/pendulum/bus.py", "the pendulum's I/O box, bound to a twin or to the real moteus board (lock, clamp window, return-home)"),
             ("viewer", "examples/pendulum/viewer.py", "the Rerun live feed: an attached, read-only cell"),
             ("script", "examples/pendulum/script.py", "one script = one attached cell that sends one trajectory under the write lease, then leaves"),
             ("up.sh", "examples/pendulum/up.sh", "brings the core up on a private dora 0.5 daemon and forwards Ctrl-C so the core gets home first")]
    rows = "".join(f"<tr><td><code>{e(a)}</code></td><td><a href='/home/bam/like_keras/{e(b)}'>{e(b)}</a></td><td>{e(c)}</td></tr>"
                   for a, b, c in files)
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Always-on pendulum</title><style>
:root{{--bg:#fbfbfa;--fg:#1d1d1f;--mute:#62656b;--card:#fff;--line:#e3e3e0;--ok:#1d7a46;--warn:#9a6200;--accent:#3c8cff}}
@media (prefers-color-scheme:dark){{:root{{--bg:#141517;--fg:#ececec;--mute:#a3a6ad;--card:#1c1d20;--line:#2e3035;--ok:#55c38a;--warn:#e0a640}}}}
body{{margin:0;background:var(--bg);color:var(--fg);font:16px/1.55 system-ui,-apple-system,Segoe UI,sans-serif}}
main{{max-width:1180px;margin:0 auto;padding:28px 16px 60px}}h1{{font-size:28px;margin:0 0 6px}}h2{{margin-top:38px;font-size:20px}}
.lede{{color:var(--mute);margin:0 0 18px}}.card{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:16px 18px;margin:14px 0}}
video,img{{max-width:100%;border-radius:8px;border:1px solid var(--line);display:block}}
.kpis{{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}}.kpi{{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:10px 12px}}
.kpi b{{display:block;font-size:22px}}.kpi span{{color:var(--mute);font-size:13px}}table{{border-collapse:collapse;width:100%;font-size:14px}}
td,th{{border-bottom:1px solid var(--line);padding:6px 8px;text-align:left;vertical-align:top}}code{{font-size:13px}}
pre{{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px;overflow-x:auto;font-size:13px}}
.ok{{color:var(--ok)}}.warn{{color:var(--warn)}}ul{{padding-left:20px}}</style></head><body><main>
<h1>Always-on pendulum: one core, scripts come and go</h1>
<p class="lede">like_keras on the real 1-DOF rack (CAN 11, the JMC shoulder pendulum on fdcanusb 272813EF), 2026-11-28 05:54–05:56.
The core (bus → JTC → guard, 100 Hz on a dora 0.5 dataflow) stayed up for the whole run. A Rerun viewer attached once and streamed
every sample. Five scripts attached one after another, each sending one trajectory and leaving; one was SIGKILLed mid-move. The core held
and the feed never stopped.</p>

<div class="card"><b>Hero: a script killed mid-move, the core holds, the next script takes over, then the core shuts down and brings the pendulum home</b>
<video src="{rel}/hero.mp4" autoplay muted loop playsinline controls poster="{rel}/viewer-still.png"></video>
<p class="lede">05:55:42 → 05:56:08, real time. Left: the rack's D405. Right: the Rerun web viewer as it ran live (orange = measured, blue = commanded).
No video? <a href="{rel}/hero.gif">GIF</a>.</p></div>

<div class="kpis">
<div class="kpi"><b class="ok">{n['samples']}</b><span>state samples over {n['duration_s']:.0f} s, one core, never restarted</span></div>
<div class="kpi"><b>{n['dt_p50_ms']:.2f} ms</b><span>tick, p50 (p99 {n['dt_p99_ms']:.2f}; worst {n['dt_max_ms']:.0f} ms, {n['gaps_over_20ms']} gaps &gt; 20 ms)</span></div>
<div class="kpi"><b>{n['rms_moving']:.2f}°</b><span>tracking RMS while moving (peak {n['peak_moving']:.2f}°; at rest {n['rms_rest']:.2f}°)</span></div>
<div class="kpi"><b>{n['tau_max']:.2f} N·m</b><span>peak torque, cap {n['cap']:.1f} N·m</span></div>
<div class="kpi"><b class="ok">{', '.join(map(str, n['faults']))}</b><span>board fault codes seen (0 = none)</span></div>
<div class="kpi"><b>{n['range'][0]:+.0f}° / {n['range'][1]:+.0f}°</b><span>swing about home ({n['home']:.1f}° on the board)</span></div>
</div>

<h2>The whole run at 4×</h2>
<div class="card"><video src="{rel}/full-4x.mp4" muted loop playsinline controls preload="metadata"></video>
<p class="lede">05:54:42 → 05:56:08. Scripts 1–5 each attach, send, leave. Between them nothing is attached except the viewer, and the core holds.</p></div>

<h2>What the core sent and what the board did</h2>
<div class="card"><img src="{rel}/trace.png" alt="commanded vs measured angle and torque over the run">
<p class="lede">Read from the core's own recording (<code>core.mcap</code>: its exposed <code>state</code> and <code>command</code> ports). Shaded = a script holds the write lease.</p></div>

<h2>How it is wired</h2>
<div class="card"><pre>
   always on (dora dataflow "pendulum", private daemon)            come and go (each its own process)
 ┌──────────────────────────────────────────────────────┐
 │ PendulumBus ──state──▶ JTC ──proposed──▶ Guard ──safe─┐│      viewer.py   subscribes state + command ──▶ Rerun
 │     ▲ (CAN 11, 100 Hz)   ▲ trajectory                  ││  ◀── script.py   writes trajectory (write lease)
 │     └──────────command (lagged)◀────────────────────────┘│      lk top      reads health + metrics
 │ exposed on unix:/run/user/1000/like_keras/pendulum.sock  │
 └──────────────────────────────────────────────────────┘
</pre>
<ul>
<li><b>Never goes down:</b> scripts reach the core only through <code>lk.attach(address)</code>. A script that leaves, or dies (SIGKILL, 05:55:48), loses its lease
after 0.25 s; the core retracts <code>trajectory</code> and the JTC holds where it is (CONTRACT R5). The next script plans from the measured state.</li>
<li><b>See what is sent:</b> the viewer subscribes to the exposed <code>command</code> (the guard's output, what the board actually got) and <code>state</code>;
<code>lk top</code> shows each box's health, tick timing, input ages and refused writes.</li>
<li><b>Same code, twin or real:</b> <code>--bind twin</code> swaps the bus for a damped pendulum behind a moteus-like PD; the core, viewer and scripts are unchanged.
Proved on the twin under inproc and dora first, then on the rack.</li>
</ul>
<table>{rows}</table></div>

<h2>Run it</h2>
<div class="card"><pre>bash /home/bam/like_keras/examples/pendulum/up.sh twin 11
PYTHONPATH=/home/bam/like_keras:/home/bam/like_keras/src /home/bam/like_keras/examples/pendulum/.venv/bin/python -m examples.pendulum.viewer
PYTHONPATH=/home/bam/like_keras:/home/bam/like_keras/src /home/bam/like_keras/examples/pendulum/.venv/bin/python -m examples.pendulum.script swing
PYTHONPATH=/home/bam/like_keras:/home/bam/like_keras/src /home/bam/like_keras/examples/pendulum/.venv/bin/python -m examples.pendulum.script sine</pre>
<p class="lede"><code>real 11</code> instead of <code>twin 11</code> drives the rack. Viewer at http://127.0.0.1:9096. Evidence: <code>{e(str(evidence))}</code>.</p></div>

<h2>Findings</h2>
<div class="card"><ul>
<li class="warn"><b>like_keras gap (dora backend):</b> <code>place(host=...)</code> sets the machine on component nodes but not on the gateway node
(<code>backends/dora/emit.py</code>), so a deployment can't start on the production daemon, which registers as machine <code>ubuntu</code>
(“failed to resolve daemon for machine None”). <code>up.sh</code> works around it with a private unnamed daemon. This is the dora box's code, so it is reported here, not patched.</li>
<li class="warn"><b>Tick gaps:</b> {n['gaps_over_20ms']} gaps over 20 ms (worst {n['dt_max_ms']:.0f} ms). The core ran as a plain user process, not at the rack runners' CPUWeight 10000.
Fine for this demo at 25–56°/s; a fast rung would need the rack's service launch.</li>
<li><b>Tracking</b> is the board's own PD (kp 400 / kd 8) with no gravity or inertia feed-forward: about 3 ticks of lag and stick-slip in velocity at the turnarounds.
The pendulum campaign's controller is better; it was not the point here.</li>
<li class="warn"><b>A killed script stops the arm instantly:</b> at the SIGKILL (05:55:48) the command froze on its last sample at 13.6° while moving at ~30°/s
(R5: retract, hold where it is). The board overshot 0.7° and settled at 12.9°. Harmless at this speed; at speed, a short decelerate-to-hold in the JTC on retraction would be kinder.</li>
<li><b>Peak torque {n['tau_max']:.2f} N·m</b> on the ±30° swing (4.4 N·m of gravity at 90°), under the 3.0 N·m tape cap. Bigger swings need a bigger cap.</li>
</ul></div>
</main></body></html>"""


def main() -> int:
    evidence = pathlib.Path(sys.argv[1])
    MEDIA.mkdir(parents=True, exist_ok=True)
    events = json.loads((evidence / "events.json").read_text())
    s = series(evidence)
    n = numbers(s)
    base = events["epoch_base"]
    composite(evidence, base + 542.0, base + 568.0, MEDIA / "hero.mp4")
    gif(MEDIA / "hero.mp4")
    composite(evidence, base + 482.0, base + 568.0, MEDIA / "full-4x.mp4", speed=4.0)
    plot(s, events, MEDIA / "trace.png")
    out = OUT / f"{NAME}.html"
    out.write_text(page(n, evidence))
    print(json.dumps({"report": str(out), **{k: (round(v, 3) if isinstance(v, float) else v) for k, v in n.items()
                                              if k not in ("range", "faults")}}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
