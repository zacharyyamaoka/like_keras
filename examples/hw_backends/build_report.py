"""Build the backends-latency report from measure.py's JSON: one self-contained HTML page, inline SVG charts.

    bash /home/bam/like_keras/examples/hw_backends/run.sh build_report
"""

from __future__ import annotations

import argparse
import base64
import html
import json
import math
from pathlib import Path

MEDIA = Path("/home/bam/like_keras/reports/media/hw-backends-2026-11-28")
OUT = Path("/home/bam/like_keras/reports/hw-backends-latency-2026-11-28.html")
ORDER = ["mujoco_nominal", "mujoco_twin", "analytical", "analytical_twin", "kinematic", "real", "remote_unix:kinematic:stepped",
         "remote_unix:analytical:stepped", "remote_unix:analytical:free", "remote_lan", "remote_cf_local"]
COLORS = {"read": "var(--c1)", "write": "var(--c2)", "round_trip": "var(--c3)"}
LO, HI = -3.0, 2.0  # log10 ms: 1 us .. 100 ms

PENDING = [
    ("6a. Remote, same machine (unix socket)", "measured",
     "like_keras main a6201f9's AttachedBackend to a serve_core child (stepped: kinematic and analytical; free: analytical). A stepped "
     "core looks for the next command at 1 kHz (like_keras STEPPED_HZ), so ~1 ms is that poll, not the socket; a free core samples "
     "at the plant's 100 Hz, so a read waits for the next 10 ms sample."),
    ("6b. Remote, LAN (camera box 192.168.1.66)", "measured",
     "Hairpin: the client here -> relay.py on the Windows box -> the core here; every message crosses the LAN twice. Windows "
     "Python has no AF_UNIX, so the box carries bytes and runs no client. ICMP floor to the box 0.195 ms avg."),
    ("6c. Remote, cloud VM", "blocked",
     "No cloud session could start: the bam, personal and dfuture CLIs answer 'Cloud sessions need a claude.ai sign-in'; personal2 "
     "gave no session in 180 s; jidoka is signed in but has no GitHub connection, so it uploads the folder: bam_ws is 361 MB (over the "
     "cap) and a 4.4 MB like_keras clone needs a folder-trust yes, which tools/cloud/pty_run.py leaves to Zach. The tunnel itself works: "
     "this core through a Cloudflare quick tunnel (no account) and back, row 6c' (cloud_core.sh is the VM side, ready)."),
    ("6d. Fingerprint mismatch", "measured", "refused before any write, on the unix socket (pytest) and over the LAN path (below)."),
]


def x_of(ms: float, width: float) -> float:
    return (min(max(math.log10(max(ms, 1e-4)), LO), HI) - LO) / (HI - LO) * width


def axis(width: float, y: float) -> str:
    parts = []
    for e, label in zip(range(-3, 3), ["1 µs", "10 µs", "0.1 ms", "1 ms", "10 ms", "100 ms"], strict=True):
        x = x_of(10.0**e, width)
        parts.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="0" y2="{y}" class="grid"/>'
                     f'<text x="{x:.1f}" y="{y + 14}" class="tick" text-anchor="middle">{label}</text>')
    return "".join(parts)


def latency_chart(case: dict) -> str:
    """Three rows (read, write, round trip): a log-x histogram, with p50 / p99 marks."""
    width, row = 560.0, 34.0
    left = 92
    body = [f'<g transform="translate({left},6)">', axis(width, 3 * row)]
    edges = [10 ** (-3 + 7 * i / 70) for i in range(71)]
    for i, key in enumerate(("read", "write", "round_trip")):
        s = case[key]
        y0 = i * row
        body.append(f'<text x="-8" y="{y0 + row / 2 + 4}" class="lab" text-anchor="end">{key.replace("_", " ")}</text>')
        if not s.get("n"):
            body.append(f'<text x="8" y="{y0 + row / 2 + 4}" class="tick">no samples</text>')
            continue
        peak = max(s["hist"]) or 1
        for count, a, b in zip(s["hist"], edges[:-1], edges[1:], strict=True):
            if not count:
                continue
            xa, xb = x_of(a, width), x_of(b, width)
            h = (row - 8) * (0.15 + 0.85 * math.sqrt(count / peak))
            body.append(f'<rect x="{xa:.1f}" y="{y0 + row - 4 - h:.1f}" width="{max(xb - xa - 0.5, 0.8):.1f}" height="{h:.1f}" '
                        f'fill="{COLORS[key]}" opacity="0.85"><title>{count} samples, {a:.4g}-{b:.4g} ms</title></rect>')
        for q, dash in (("p50", ""), ("p99", ' stroke-dasharray="3 2"')):
            x = x_of(s[q], width)
            body.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{y0 + 2}" y2="{y0 + row - 2}" class="mark"{dash}><title>{q} {s[q]:.4g} ms</title></line>')
    body.append("</g>")
    return f'<svg viewBox="0 0 {left + width + 20} {3 * row + 30}" class="chart" role="img">{"".join(body)}</svg>'


def step_chart(cases: dict) -> str:
    width, height, left = 560.0, 220.0, 50
    traces = [(n, c) for n, c in ((n, cases.get(n)) for n in ORDER[:6]) if c and c.get("step")]  # the local backends
    tmax = 0.6
    ymax = max(max(c["step"]["dq_deg"]) for _, c in traces) * 1.1
    ymin = min(0.0, min(min(c["step"]["dq_deg"]) for _, c in traces))
    palette = ["var(--c1)", "var(--c2)", "var(--c3)", "var(--c4)", "var(--c5)", "var(--c6)"]

    def px(t, q):
        return left + t / tmax * width, height - (q - ymin) / (ymax - ymin) * height

    body = []
    for tick in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6):
        x, _ = px(tick, 0)
        body.append(f'<line x1="{x}" x2="{x}" y1="0" y2="{height}" class="grid"/><text x="{x}" y="{height + 14}" class="tick" text-anchor="middle">{tick:.1f} s</text>')
    for tick in (0.0, 1.0, 2.0):
        _, y = px(0, tick)
        body.append(f'<line x1="{left}" x2="{left + width}" y1="{y}" y2="{y}" class="grid"/><text x="{left - 6}" y="{y + 4}" class="tick" text-anchor="end">{tick:.0f}°</text>')
    legend = []
    for i, (name, case) in enumerate(traces):
        s = case["step"]
        # simulated: plant time = ticks x period; real: wall time (its loop is the bus)
        times = [(k + 1) * s["period_s"] for k in range(len(s["dq_deg"]))] if case["simulated"] else s["t_s"]
        points = " ".join(f"{a:.1f},{b:.1f}" for a, b in (px(t, q) for t, q in zip(times, s["dq_deg"], strict=True)) if a <= left + width)
        dash = ' stroke-dasharray="6 4"' if name.startswith("analytical") else ""  # WHY: the nominal ODE lies on the nominal MuJoCo curve
        body.append(f'<polyline points="{points}" fill="none" stroke="{palette[i % 6]}" stroke-width="1.8"{dash}/>')
        legend.append(f'<span class="key"><i style="background:{palette[i % 6]}"></i>{html.escape(case["label"])}'
                      f'{"" if case["simulated"] else " (wall time)"}</span>')
    return (f'<svg viewBox="0 0 {left + width + 20} {height + 24}" class="chart" role="img">{"".join(body)}</svg>'
            f'<div class="legend">{"".join(legend)}</div>')


def fmt(s: dict, key: str) -> str:
    return "—" if not s.get("n") else f"{s[key]:.3f}" if s[key] < 10 else f"{s[key]:.1f}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--media", default=str(MEDIA))
    parser.add_argument("--out", default=str(OUT))
    arguments = parser.parse_args(argv)
    media = Path(arguments.media)
    cases = json.loads((media / "latency.json").read_text())["cases"]
    preview = json.loads((media / "remote_preview.json").read_text())
    rows, panels, configs = [], [], []
    for name in ORDER:
        c = cases.get(name)
        if not c:
            continue
        how = html.escape(c["note"]) if c.get("note") else ("attached, unix socket, this host" if name.startswith("remote_unix")
                                                             else "live" if not c["simulated"] else "sim")
        rows.append(
            f"<tr><td>{html.escape(c['label'])}</td>"
            + "".join(f"<td class=n>{fmt(c[k], q)}</td>" for k in ("read", "write", "round_trip") for q in ("p50", "p95", "p99"))
            + f"<td class=n>{fmt(c['round_trip'], 'max')}</td><td class=n>{c['round_trip_ticks']['p50']:.0f}</td>"
            f"<td class=n>{c['loop_hz']:,.0f}</td><td class=n>{c['read']['n']}/{c['round_trip']['n']}</td><td>{how}</td></tr>")
        panels.append(f'<figure><figcaption><b>{html.escape(c["label"])}</b> · read p50 {fmt(c["read"], "p50")} ms · write p50 '
                      f'{fmt(c["write"], "p50")} ms · round trip p50 {fmt(c["round_trip"], "p50")} ms · {c["loop_hz"]:,.0f} Hz loop'
                      f'</figcaption>{latency_chart(c)}</figure>')
        configs.append(f"<tr><td>{html.escape(c['label'])}</td><td><code>{html.escape(c['config'])}</code></td>"
                       f"<td>{html.escape(c['description']['label']) + ' · ' + c['description']['digest'] if c.get('description') else '—'}</td></tr>")
    pending = "".join(f"<tr><td>{html.escape(a)}</td><td><span class='pill {s}'>{s}</span></td><td>{html.escape(b)}</td></tr>" for a, s, b in PENDING)
    nominal = next((c["description"] for c in cases.values() if c.get("description", {}) and c["description"]["label"].startswith("nominal")), None)
    twin = next((c["description"] for c in cases.values() if c.get("description", {}) and c["description"]["label"].startswith("twin")), None)
    param_rows = ""
    if nominal and twin:
        for key, unit in (("gravity_nm", "N·m"), ("bottom_rad", "rad"), ("inertia_kg_m2", "kg·m²"), ("damping_nm_s_rad", "N·m·s/rad"),
                          ("coulomb_nm", "N·m"), ("kp_nm_rad", "N·m/rad"), ("kd_nm_s_rad", "N·m·s/rad"), ("cap_nm", "N·m")):
            param_rows += f"<tr><td>{key}</td><td class=n>{nominal[key]:.4g}</td><td class=n>{twin[key]:.4g}</td><td>{unit}</td></tr>"
        param_rows += ("<tr><td>MuJoCo-only knobs</td><td>none</td><td>LuGre friction (Fs 0.60, Fc 0.75 N·m), 2 ms command latch, 200 Hz current "
                       "loop, 4 A current limit</td><td></td></tr>")
    real = cases.get("real")
    photo = base64.b64encode((media / "obsbot_small.jpg").read_bytes()).decode() if (media / "obsbot_small.jpg").exists() else ""
    real_block = ""
    if real:
        real_block = (f"<p>Live, {real['utc']}: the CAN 11 slot was taken with <code>rack_can11_want</code> (deleted after), the rack's "
                      f"bus lock <code>bus.272813EF.lock</code> held for the run. The board was found at rest at {real['origin_deg']:.1f}° "
                      f"(its bottom) and every target was clamped to that ±3°; the steps were ±2°, the peak excursion "
                      f"{max(real['step']['dq_deg']):.2f}°. The full arm was idle (arm-hold holding, no homing). {real['read']['n']} "
                      f"write/read ticks, {real['round_trip']['n']} steps, 0 faults.</p>")
        if photo:
            real_block += (f'<figure><img src="data:image/jpeg;base64,{photo}" alt="OBSBOT frame of the workcell before the run">'
                           "<figcaption>OBSBOT, just before the real run: the full arm at rest. The rack pendulum is outside this "
                           "camera's view; the evidence of its motion is the step trace above (encoder).</figcaption></figure>")
    page = TEMPLATE.format(rows="".join(rows), panels="".join(panels), configs="".join(configs), pending=pending,
                           step=step_chart(cases), params=param_rows, real=real_block,
                           refusal=html.escape(preview["refusal"]), lan_refusal=html.escape((media / "lan_refusal.txt").read_text().strip()), core=html.escape(json.dumps(preview["core"])),
                           client=html.escape(preview["client"]), preview_status=html.escape(preview["status"]))
    Path(arguments.out).write_text(page, encoding="utf-8")
    print(f"wrote {arguments.out} ({len(page) / 1024:.0f} KB)")
    return 0


TEMPLATE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Hardware Backends Latency</title>
<style>
:root {{ --bg:#fbfbfa; --fg:#1d1d1f; --mut:#6b6b70; --line:#e4e4e2; --card:#fff;
  --c1:#2f6fdb; --c2:#d9822b; --c3:#2a9d6f; --c4:#9b59b6; --c5:#c0392b; --c6:#7f8c8d; }}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{ --bg:#161618; --fg:#ececee; --mut:#9a9aa0; --line:#2c2c30; --card:#1e1e21;
  --c1:#6c9ff0; --c2:#f0a35c; --c3:#4cc495; --c4:#bb8fd0; --c5:#ec7063; --c6:#a3adb0; }} }}
:root[data-theme="dark"] {{ --bg:#161618; --fg:#ececee; --mut:#9a9aa0; --line:#2c2c30; --card:#1e1e21; }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0; padding:24px 16px 64px; }}
main {{ max-width:1040px; margin:0 auto; }}
h1 {{ font-size:24px; margin:0 0 4px; }} h2 {{ font-size:18px; margin:32px 0 8px; }}
.verdict {{ font-size:17px; margin:8px 0 16px; }}
.mut {{ color:var(--mut); }}
.scroll {{ overflow-x:auto; }}
table {{ border-collapse:collapse; width:100%; font-size:13px; background:var(--card); }}
th, td {{ border-bottom:1px solid var(--line); padding:5px 7px; text-align:left; vertical-align:top; }}
th {{ font-weight:600; white-space:nowrap; }} td.n {{ text-align:right; font-variant-numeric:tabular-nums; white-space:nowrap; }}
code, pre {{ font:12px/1.45 ui-monospace, monospace; }}
pre {{ background:var(--card); border:1px solid var(--line); padding:10px; white-space:pre-wrap; word-break:break-all; }}
figure {{ margin:12px 0; background:var(--card); border:1px solid var(--line); padding:10px; }}
figcaption {{ font-size:13px; margin-bottom:6px; }}
figure img {{ max-width:100%; height:auto; display:block; }}
svg.chart {{ width:100%; height:auto; display:block; }}
.grid {{ stroke:var(--line); }} .tick {{ fill:var(--mut); font-size:10px; }} .lab {{ fill:var(--fg); font-size:12px; }}
.mark {{ stroke:var(--fg); stroke-width:1.4; }}
.legend {{ display:flex; flex-wrap:wrap; gap:6px 16px; font-size:12px; margin-top:6px; }}
.key i {{ display:inline-block; width:12px; height:3px; margin-right:5px; vertical-align:middle; }}
.pill {{ font-size:11px; padding:1px 7px; border-radius:9px; border:1px solid var(--line); white-space:nowrap; }}
.pill.blocked {{ color:var(--c5); }} .pill.measured {{ color:var(--c3); }}
ul {{ padding-left:20px; }}
</style></head><body><main>
<h1>One HardwareInterface, six backends</h1>
<p class="mut">The rack pendulum (CAN 11) behind one robot_msgs <code>HardwareInterface</code> · like_keras <code>examples/hw_backends</code> · 2026-11-28</p>
<p class="verdict"><b>One HardwareInterface reads and writes the pendulum on MuJoCo (nominal and twin of record), analytical, kinematic,
real CAN 11 (live) and remote (same machine and LAN, through like_keras's AttachedBackend), all timed.</b> The cloud leg is blocked
on a Claude cloud sign-in or folder trust, not on the network: the tunnel it would use was measured end to end from here.</p>
<ul>
<li>The interface itself costs ~0.02 ms (kinematic read). Everything above that is the backend: the real bus ~1.1–1.3 ms per CAN exchange, MuJoCo ~2.3 ms per 10 ms period (bam_rack_twin's 30 kHz moteus PID twin), the ODE ~0.04 ms.</li>
<li>Round trip = write a 2° step, then write/read until the joint has moved 0.02° toward it. On a sim that is one tick. On the real board it is ~5 bus ticks (~12 ms): the pendulum's inertia, not the link.</li>
<li>Remote adds ~1 ms on this host and over the LAN alike (p50): a stepped core checks for a command at 1 kHz. The LAN shows in the tail (p99 ~7 ms vs ~2 ms). Through Cloudflare's edge and back it is ~11 ms. A free-running core (real's default) answers on its own 100 Hz clock: a read waits up to one 10 ms sample, and a step shows two samples later (~20 ms).</li>
<li>"No friction" and "realistic" are one MuJoCo backend with two parameter sets (bam_rack_twin's twin of record, wrapped, not rewritten).</li>
<li>Found on the way: robot_msgs 4137538 (07:20 today, GroundTruth.meta: Any) makes ServoState un-encodable for like_keras's codec, so every attached core fails at start on robot_msgs main. The remote numbers here ran on robot_msgs 292b832.</li>
</ul>

<h2>Latency (ms)</h2>
<div class="scroll"><table>
<tr><th rowspan=2>case</th><th colspan=3>read()</th><th colspan=3>write()</th><th colspan=4>round trip</th><th rowspan=2>ticks to<br>see it (p50)</th><th rowspan=2>loop Hz</th><th rowspan=2>samples<br>ticks/steps</th><th rowspan=2>how</th></tr>
<tr><th>p50</th><th>p95</th><th>p99</th><th>p50</th><th>p95</th><th>p99</th><th>p50</th><th>p95</th><th>p99</th><th>max</th></tr>
{rows}
</table></div>
<p class="mut">Each read/write is one call through <code>HardwareInterface</code> (JointTable mapping included), over a write-then-read loop that holds position. Loop Hz = unpaced write+read ticks per second. A simulated backend advances one 10 ms period per write, so its "loop Hz" is how much faster than real time it can run.</p>
<div class="scroll"><table><tr><th>remote case</th><th>status</th><th>why</th></tr>{pending}</table></div>

<h2>Per backend</h2>
<p class="mut">Histogram of every sample on a shared log axis; solid mark p50, dashed p99.</p>
{panels}

<h2>Step responses</h2>
<p class="mut">The first 2° step of each case. Sims on plant time; the real board on wall time. The frictionless MuJoCo and the analytical ODE are one curve (the ODE is dashed on top; the test suite holds them within 0.05°). The twin of record stops short in its LuGre friction band; the real board settles near 2°. Its PID in the deployment record is kp 200 / kd 4 N·m/rev (conf get 2026-10-29), while bam_ws memory says CAN 11 has run kp 400 / kd 8 since 11-15. Not checked live; if true, the twin runs stale gains.</p>
<figure>{step}</figure>

<h2>Real run</h2>
{real}

<h2>Config objects</h2>
<p class="mut">The config's type picks the backend (<code>hardware(config, description)</code>); a config is how the run happens, the description what is simulated.</p>
<div class="scroll"><table><tr><th>case</th><th>config</th><th>description · digest</th></tr>{configs}
<tr><td>6. Remote</td><td><code>RemoteConfig(address='unix:…/pendulum_can11.sock', fingerprint=robot_fingerprint(), robot='pendulum_can11', lease_s=0.25, timeout_s=2.0)</code></td><td>the core's</td></tr></table></div>
<h3>The two parameter sets the MuJoCo and analytical backends run</h3>
<div class="scroll"><table><tr><th>parameter</th><th>nominal (frictionless)</th><th>twin of record</th><th>unit</th></tr>{params}</table></div>

<h2>Fingerprint refusal</h2>
<p>The fingerprint is the robot's (its name and each joint with its servo key), not the parameter set: a client planning on the nominal sim may drive a core running the twin or the real board; it may not drive another robot.</p>
<pre>core:   {core}
client: {client}
→ {refusal}</pre>
<p>The same refusal through the LAN hairpin (client here → camera box → core here):</p>
<pre>{lan_refusal}</pre>

<h2>Reproduce</h2>
<pre>bash /home/bam/like_keras/examples/hw_backends/run.sh test
bash /home/bam/like_keras/examples/hw_backends/run.sh measure
bash /home/bam/like_keras/examples/hw_backends/run.sh build_report</pre>
</main></body></html>
"""

if __name__ == "__main__":
    raise SystemExit(main())
