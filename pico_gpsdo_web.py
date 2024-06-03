"""
Flask dashboard for GPSDO over USB CDC — used by: ./pico.py flash --webserver

Requires: pip install flask pyserial

The host TTY is the Pico's USB serial (CDC), not the UART to the GNSS. UBX binary from
the receiver is parsed on the Pico; only printf text (telemetry lines, CLI, heartbeats)
crosses USB — so splitting on newlines here is correct.

Plot retention: up to 12 h of samples in RAM by default (PICO_PLOT_RETAIN_S, default 43200).
Chart window: user selects 5 / 10 / 30 / 60 / 240 / 720 minutes via ?window_min= on /api/series.
UBX hex lines (firmware): enable with ubxhex 1 on USB — UI toggle sends that command.
Host log file: all CDC lines append to ./pico_gpsdo.log by default (PICO_GPSDO_LOGFILE to change or disable).
UI log panel shows only the last PICO_UI_LOG_LINES (default 400) lines; RAM still holds up to MAX_LOG_LINES.
"""

from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from collections import deque
from typing import Any

# Flask and pyserial are imported inside run_dashboard / _serial_reader so a missing
# dependency does not prevent the HTTP server from starting when the other is present.

MAX_LOG_LINES = 12000
DAC_SETTLE_MASK_S = 2.0


def _ui_log_line_cap() -> int:
    """How many recent lines /api/logs returns for the browser (full history stays in RAM + optional file)."""
    raw = os.environ.get("PICO_UI_LOG_LINES", "400").strip()
    try:
        n = int(raw)
    except ValueError:
        n = 400
    return max(50, min(n, MAX_LOG_LINES))


def _host_logfile_path() -> str | None:
    """Append-only host log of CDC lines. Env PICO_GPSDO_LOGFILE: path, or empty/0/none to disable. Default: pico_gpsdo.log."""
    if "PICO_GPSDO_LOGFILE" in os.environ:
        raw = os.environ["PICO_GPSDO_LOGFILE"].strip()
        if not raw or raw.lower() in ("-", "0", "none", "off"):
            return None
        return os.path.abspath(os.path.expanduser(raw))
    return os.path.abspath(os.path.join(os.getcwd(), "pico_gpsdo.log"))


# Hard cap on series length (after time-based trim). Override: PICO_PLOT_MAX_POINTS.
def _max_points_cap() -> int:
    raw = os.environ.get("PICO_PLOT_MAX_POINTS", "200000").strip()
    try:
        n = int(raw)
    except ValueError:
        return 200000
    return max(5000, min(n, 2_000_000))


def _plot_retain_s() -> float:
    """Keep plot samples for this many seconds (default 12 h). Env: PICO_PLOT_RETAIN_S."""
    raw = os.environ.get("PICO_PLOT_RETAIN_S", "43200").strip()
    try:
        w = float(raw)
    except ValueError:
        return 3600.0
    return max(60.0, min(w, 86400.0))


ALLOWED_WINDOW_MIN = (5, 10, 30, 60, 240, 720)


def _json_safe_series(xs: list[float]) -> list[float | None]:
    """JSON has no NaN; Flask would emit invalid JSON and the browser would fail to parse."""
    out: list[float | None] = []
    for v in xs:
        if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
            out.append(None)
        else:
            out.append(v)
    return out


def _slice_last_seconds(
    t: list[float],
    clkB: list[float],
    qErr: list[float],
    dac: list[float],
    window_s: float,
) -> tuple[list[float], list[float], list[float], list[float]]:
    """Keep rows whose t is within [max(t) - window_s, max(t)]."""
    if not t or window_s <= 0:
        return t, clkB, qErr, dac
    tmax = max(t)
    cutoff = tmax - window_s
    lo = 0
    for i, ti in enumerate(t):
        if ti >= cutoff:
            lo = i
            break
    return t[lo:], clkB[lo:], qErr[lo:], dac[lo:]


def _trim_series_to_retention(state: "GpsdoState") -> None:
    """Drop samples older than PICO_PLOT_RETAIN_S, then enforce max point cap."""
    retain = _plot_retain_s()
    if len(state.t) < 2:
        return
    tmax = state.t[-1]
    cutoff = tmax - retain
    lo = 0
    for i, ti in enumerate(state.t):
        if ti >= cutoff:
            lo = i
            break
    if lo > 0:
        state.t = state.t[lo:]
        state.clkB = state.clkB[lo:]
        state.qErr = state.qErr[lo:]
        state.dac = state.dac[lo:]
    cap = _max_points_cap()
    if len(state.t) > cap:
        n = len(state.t) - cap
        state.t = state.t[n:]
        state.clkB = state.clkB[n:]
        state.qErr = state.qErr[n:]
        state.dac = state.dac[n:]


_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>picoExtRef GPSDO</title>
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css" rel="stylesheet" crossorigin="anonymous"/>
  <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/js/bootstrap.bundle.min.js" crossorigin="anonymous"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
  <style>
    :root {
      --warm-bg: #faf7f2;
      --warm-card: #fffefb;
      --warm-border: #f0e4d4;
      --warm-muted: #6b5d4f;
      --warm-ink: #3d3428;
      --bs-primary: #c45c3e;
      --bs-primary-rgb: 196, 92, 62;
      --bs-secondary: #b08968;
      --bs-secondary-rgb: 176, 137, 104;
    }
    body { font-family: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; background: var(--warm-bg); color: var(--warm-ink); min-height: 100vh; }
    .canvas-wrap { background: var(--warm-card); border-radius: 0.75rem; border: 1px solid var(--warm-border); min-height: 220px; }
    .canvas-wrap canvas { max-height: 280px; }
    #plotTabs .nav-link { color: var(--warm-muted); border: none; border-bottom: 2px solid transparent; }
    #plotTabs .nav-link:hover { color: var(--warm-ink); border-color: var(--warm-border); }
    #plotTabs .nav-link.active { color: var(--bs-primary); border-color: var(--bs-primary); background: transparent; font-weight: 600; }
    .tab-chart-wrap { position: relative; height: min(42vh, 440px); min-height: 300px; }
    .tab-chart-wrap canvas { max-height: none !important; }
    #log {
      background: var(--warm-card); color: #2d4a3e; font: 12px/1.45 ui-monospace, monospace;
      min-height: 12rem; height: min(28vh, 360px); max-height: 40vh; overflow: auto; padding: 0.75rem 1rem;
      border-radius: 0.75rem; border: 1px solid var(--warm-border); white-space: pre-wrap; word-break: break-all;
    }
    .form-label-sm { font-size: 0.8rem; color: var(--warm-muted); }
    .btn-pill { border-radius: 50rem !important; padding-left: 1.1rem; padding-right: 1.1rem; }
    .btn-warm-outline { border-color: #d4a574; color: #8b5a2b; }
    .btn-warm-outline:hover { background: #fff5e8; border-color: #c9956c; color: #6b4423; }
    .btn-amber { background: #d97706; border-color: #d97706; color: #fff; }
    .btn-amber:hover { background: #b45309; border-color: #b45309; color: #fff; }
    .card-warm { background: var(--warm-card); border: 1px solid var(--warm-border); border-radius: 0.85rem; }
    .telemetry-status { color: #2d4a3e; line-height: 1.55; }
    .gps-fix-strip {
      border-radius: 0.85rem;
      border: 1px solid var(--warm-border);
      border-left-width: 6px;
      background: var(--warm-card);
      box-shadow: 0 0.125rem 0.35rem rgba(61, 52, 40, 0.06);
    }
    .gps-fix-strip--muted { border-left-color: #a8a29e; }
    .gps-fix-strip--bad { border-left-color: #dc2626; background: linear-gradient(90deg, rgba(220, 38, 38, 0.06) 0%, var(--warm-card) 48%); }
    .gps-fix-strip--warn { border-left-color: #d97706; background: linear-gradient(90deg, rgba(217, 119, 6, 0.07) 0%, var(--warm-card) 48%); }
    .gps-fix-strip--ok { border-left-color: #15803d; background: linear-gradient(90deg, rgba(21, 128, 61, 0.08) 0%, var(--warm-card) 45%); }
    .loop-strip--coarse { border-left-color: #d97706; background: linear-gradient(90deg, rgba(217, 119, 6, 0.07) 0%, var(--warm-card) 45%); }
    .loop-strip--fine { border-left-color: #15803d; background: linear-gradient(90deg, rgba(21, 128, 61, 0.08) 0%, var(--warm-card) 45%); }
    .loop-strip--inactive { border-left-color: #78716c; }
    .track-strip--ok { border-left-color: #15803d; background: linear-gradient(90deg, rgba(21, 128, 61, 0.08) 0%, var(--warm-card) 45%); }
    .track-strip--warn { border-left-color: #d97706; background: linear-gradient(90deg, rgba(217, 119, 6, 0.07) 0%, var(--warm-card) 45%); }
    .gps-fix-label { font-size: 0.7rem; letter-spacing: 0.06em; color: var(--warm-muted); text-transform: uppercase; }
    #gpsFixText { color: var(--warm-ink); line-height: 1.15; }
  </style>
</head>
<body>
  <div class="container py-3 py-md-4" style="max-width: 1140px;">
    <div id="errBanner" class="alert alert-danger border-0 shadow-sm mb-3 d-none" role="alert" style="border-radius: 0.75rem;"></div>
    <header class="mb-3">
      <div class="d-flex flex-wrap align-items-baseline gap-2 mb-1">
        <h1 class="h4 fw-semibold mb-0">GPSDO <span class="text-secondary fw-normal small" id="tty"></span></h1>
        <span class="badge rounded-pill px-3 py-2 fs-6 fw-semibold border" style="background: var(--warm-card); border-color: var(--warm-border) !important; color: var(--warm-ink);" id="fwVersion" title="Firmware build id from USB line [ver] …">—</span>
      </div>
      <p class="small text-muted mb-0">USB CLI: … · telemetry <code>lp</code> <code>tr</code> (bitmask) · thresholds include <strong>min_sv</strong> <strong>facc</strong> …</p>
    </header>

    <div id="gpsFixStrip" class="gps-fix-strip gps-fix-strip--muted mb-3 px-3 py-3 py-md-4">
      <div class="row align-items-center g-3">
        <div class="col-md-8">
          <div class="gps-fix-label mb-1">GNSS fix</div>
          <div id="gpsFixText" class="display-6 fw-bold">—</div>
          <div id="gpsFixSub" class="small text-muted mt-1"></div>
        </div>
        <div class="col-md-4 text-md-end">
          <div class="gps-fix-label mb-1">Satellites</div>
          <div id="gpsFixSv" class="fs-2 fw-semibold" style="color: var(--warm-ink);">—</div>
        </div>
      </div>
    </div>

    <div id="loopStageStrip" class="gps-fix-strip gps-fix-strip--muted mb-3 px-3 py-3 py-md-3">
      <div class="gps-fix-label mb-1">Servo loop stage</div>
      <div id="loopStageText" class="display-6 fw-bold" style="font-size: 1.75rem;">—</div>
      <div id="loopStageSub" class="small text-muted mt-1">Firmware JSON field <code>lp</code>: 0 = coarse (NAV-CLOCK bias), 1 = fine (TIM-TP qErr), 2 = inactive (mode free)</div>
    </div>

    <div id="trackStrip" class="gps-fix-strip gps-fix-strip--muted mb-3 px-3 py-3 py-md-3">
      <div class="gps-fix-label mb-1">Tracking quality</div>
      <div id="trackText" class="display-6 fw-bold" style="font-size: 1.75rem;">—</div>
      <div id="trackSub" class="small text-muted mt-1">Firmware <code>tr</code>: 0 = OK · +1 = no fix · +2 = low SV · +4 = fAcc &gt; facc gate</div>
    </div>

    <div class="card card-warm shadow-sm border-0 mb-3">
      <div class="card-body py-2 px-3 d-flex flex-wrap align-items-center gap-2 gap-md-3">
        <label for="plotWindowMin" class="small text-muted mb-0">Plot window</label>
        <select id="plotWindowMin" class="form-select form-select-sm" style="width: auto; min-width: 12rem;" title="Chart time span (server keeps up to retention window)">
          <option value="5">Last 5 min</option>
          <option value="10" selected>Last 10 min</option>
          <option value="30">Last 30 min</option>
          <option value="60">Last 60 min</option>
          <option value="240">Last 4 hours</option>
          <option value="720">Last 12 hours</option>
        </select>
        <div class="d-flex flex-wrap align-items-center gap-3 ms-md-auto">
          <button type="button" class="btn btn-outline-secondary btn-sm btn-pill" id="clearHistoryBtn" title="Clear in-memory plot history (t/clkB/qErr/dac)">
            Clear history
          </button>
          <div class="form-check form-switch mb-0">
            <input class="form-check-input" type="checkbox" id="jsonTelemetryToggle" checked title="Firmware prints compact `{…}` telemetry lines on USB"/>
            <label class="form-check-label small" for="jsonTelemetryToggle">JSON telemetry</label>
          </div>
          <div class="form-check form-switch mb-0">
            <input class="form-check-input" type="checkbox" id="ubxHexToggle" title="Firmware prints UBX:0xb5,0x62,... per packet on USB"/>
            <label class="form-check-label small" for="ubxHexToggle">UBX hex (unit tests)</label>
          </div>
        </div>
      </div>
    </div>

    <div class="card card-warm shadow-sm border-0 mb-3">
      <div class="card-header bg-transparent border-bottom pt-3 pb-0 px-2 px-sm-3" style="border-color: var(--warm-border) !important;">
        <ul class="nav nav-tabs border-0" id="plotTabs" role="tablist">
          <li class="nav-item" role="presentation">
            <button class="nav-link active rounded-0" id="tab-bias-btn" data-bs-toggle="tab" data-bs-target="#tab-bias" type="button" role="tab" aria-controls="tab-bias" aria-selected="true">clkB + qErr</button>
          </li>
          <li class="nav-item" role="presentation">
            <button class="nav-link rounded-0" id="tab-qhist-btn" data-bs-toggle="tab" data-bs-target="#tab-qhist" type="button" role="tab" aria-controls="tab-qhist" aria-selected="false">qErr hist</button>
          </li>
          <li class="nav-item" role="presentation">
            <button class="nav-link rounded-0" id="tab-dac-btn" data-bs-toggle="tab" data-bs-target="#tab-dac" type="button" role="tab" aria-controls="tab-dac" aria-selected="false">DAC</button>
          </li>
        </ul>
      </div>
      <div class="card-body p-2 p-sm-3">
        <div class="tab-content" id="plotTabContent">
          <div class="tab-pane fade show active" id="tab-bias" role="tabpanel" aria-labelledby="tab-bias-btn">
            <div class="canvas-wrap tab-chart-wrap"><canvas id="chBias"></canvas></div>
          </div>
          <div class="tab-pane fade" id="tab-qhist" role="tabpanel" aria-labelledby="tab-qhist-btn">
            <div class="canvas-wrap tab-chart-wrap"><canvas id="chQhist"></canvas></div>
          </div>
          <div class="tab-pane fade" id="tab-dac" role="tabpanel" aria-labelledby="tab-dac-btn">
            <div class="canvas-wrap tab-chart-wrap"><canvas id="chDac"></canvas></div>
          </div>
        </div>
      </div>
    </div>

    <div class="card card-warm shadow-sm border-0 mb-3">
      <div class="card-body py-2 px-3">
        <div class="d-flex align-items-baseline justify-content-between gap-2 mb-1 flex-wrap">
          <span class="h6 fw-semibold text-secondary mb-0">Telemetry</span>
          <span class="small text-muted">last line fields (live)</span>
        </div>
        <div id="telemetryLine" class="telemetry-status font-monospace small text-break"></div>
      </div>
    </div>

    <div class="card card-warm shadow-sm border-0 mb-3">
      <div class="card-body">
        <h2 class="h6 fw-semibold text-secondary mb-2">P-only loop tuning</h2>
        <form id="tuneForm">
          <p class="small text-muted mb-2"><strong>Coarse</strong> = NAV-CLOCK <code>clkD_bias</code>; <strong>fine</strong> = TIM-TP <code>qErr</code> (proportional only) when finelock. Telemetry <code>lp</code> shows stage. Thresholds are in the next card.</p>
          <div class="fw-semibold small text-secondary mb-1 mt-2">Coarse stage</div>
          <div class="row row-cols-1 row-cols-sm-2 row-cols-lg-3 g-2 mb-2">
            <div><label class="form-label form-label-sm mb-0">kp_coarse</label><input class="form-control form-control-sm" name="kp_coarse" type="text" placeholder="0.01" title="Coarse loop: gain on clkD_bias when clkB steps"/></div>
          </div>
          <div class="fw-semibold small text-secondary mb-1 mt-2">Fine stage</div>
          <div class="row row-cols-1 row-cols-sm-2 row-cols-lg-3 g-2 mb-2">
            <div><label class="form-label form-label-sm mb-0">kp</label><input class="form-control form-control-sm" name="kp" type="text" placeholder="0.08" title="Fine loop: gain on qErr (P-only)"/></div>
            <div><label class="form-label form-label-sm mb-0">int</label><input class="form-control form-control-sm" name="int" type="text" placeholder="integrator" title="Initial DAC / manual integrator (PWM counts)"/></div>
          </div>
          <div class="d-flex flex-wrap gap-2 pt-1">
            <button type="submit" class="btn btn-primary btn-pill shadow-sm">Send set</button>
            <button type="button" class="btn btn-warm-outline btn-pill" id="dumpBtn">dump</button>
            <button type="button" class="btn btn-amber btn-pill" id="servoBtn">mode servo</button>
            <button type="button" class="btn btn-outline-secondary btn-pill" id="freeBtn">mode free</button>
          </div>
        </form>
      </div>
    </div>

    <div class="card card-warm shadow-sm border-0 mb-3">
      <div class="card-body">
        <h2 class="h6 fw-semibold text-secondary mb-2">Thresholds &amp; finelock</h2>
        <form id="gateForm">
          <p class="small text-muted mb-2"><strong>min_sv</strong> sets telemetry <code>tr</code> bit1 when visible SV count is below it (0 disables). <strong>facc</strong> caps coarse corrections and sets <code>tr</code> bit2 when fAcc exceeds gate (0 disables gate).</p>
          <div class="row row-cols-1 row-cols-sm-2 row-cols-lg-3 g-2 mb-2">
            <div><label class="form-label form-label-sm mb-0">min_sv</label><input class="form-control form-control-sm" name="min_sv" type="text" placeholder="4" title="tr bit1 if satellites_visible &lt; this; 0 = off"/></div>
            <div><label class="form-label form-label-sm mb-0">facc</label><input class="form-control form-control-sm" name="facc" type="text" placeholder="3000" title="Skip coarse DAC when fAcc exceeds this (ps/s); 0 disables"/></div>
            <div><label class="form-label form-label-sm mb-0">fine_enter</label><input class="form-control form-control-sm" name="fine_enter" type="text" placeholder="20" title="|clkB| must stay below this (ns) to count toward finelock"/></div>
            <div><label class="form-label form-label-sm mb-0">fine_exit</label><input class="form-control form-control-sm" name="fine_exit" type="text" placeholder="30" title="Leave finelock if |clkB| above (ns)"/></div>
            <div><label class="form-label form-label-sm mb-0">fine_enter_hold</label><input class="form-control form-control-sm" name="fine_enter_hold" type="text" placeholder="30" title="Seconds |clkB| must stay within fine_enter before finelock"/></div>
            <div><label class="form-label form-label-sm mb-0">fine_qmax</label><input class="form-control form-control-sm" name="fine_qmax" type="text" placeholder="5000" title="|qErr| must stay below this (ps) to enter finelock; exit when |qErr| &gt;= this; 0 disables"/></div>
          </div>
          <div class="d-flex flex-wrap gap-2 pt-1">
            <button type="submit" class="btn btn-primary btn-pill shadow-sm">Send set</button>
          </div>
        </form>
      </div>
    </div>

    <div class="card card-warm shadow-sm border-0 mb-3">
      <div class="card-body">
        <h2 class="h6 fw-semibold text-secondary mb-2">Raw command</h2>
        <div class="input-group input-group-sm">
          <input class="form-control" id="rawLine" type="text" placeholder="Kp 0.08" style="border-radius: 50rem 0 0 50rem;"/>
          <button type="button" class="btn btn-primary btn-pill px-3" id="rawSend" style="border-radius: 0 50rem 50rem 0;">Send</button>
        </div>
      </div>
    </div>

    <div class="d-flex flex-wrap align-items-baseline justify-content-between gap-2 mb-2">
      <h2 class="h6 fw-semibold text-secondary mb-0">Log</h2>
      <span class="small text-muted" id="logMeta" title="Browser shows a tail; full stream is appended to the host log file when enabled"></span>
    </div>
    <div id="log"></div>
  </div>
  <script>
    document.getElementById('tty').textContent = __TTY__;

    Chart.defaults.color = '#5c5346';
    Chart.defaults.borderColor = 'rgba(200, 180, 150, 0.35)';

    const biasCtx = document.getElementById('chBias').getContext('2d');
    const qhistCtx = document.getElementById('chQhist').getContext('2d');
    const dacCtx = document.getElementById('chDac').getContext('2d');
    const baseScales = {
      x: { title: { display: true, text: 't (s)', color: '#5c5346' }, ticks: { maxTicksLimit: 8, color: '#6b5d4f' }, grid: { color: 'rgba(200, 180, 150, 0.35)' } },
    };
    const chartBias = new Chart(biasCtx, {
      type: 'scatter',
      data: {
        datasets: [
          { label: 'clkB (ns)', data: [], borderColor: '#c45c3e', backgroundColor: '#c45c3e', pointRadius: 2, pointHoverRadius: 3, showLine: false },
          { label: 'qErr (ns)', data: [], borderColor: '#0d9488', backgroundColor: '#0d9488', pointRadius: 2, pointHoverRadius: 3, showLine: false },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        plugins: {
          title: { display: true, text: 'clkB + qErr (dots)', color: '#3d3428', font: { size: 14, weight: '600' } },
          legend: { labels: { color: '#5c5346' } },
        },
        scales: {
          x: { type: 'linear', ...baseScales.x },
          y: {
            type: 'linear',
            title: { display: true, text: 'time error (ns)', color: '#5c5346' },
            grace: '12%',
            ticks: { maxTicksLimit: 10, color: '#6b5d4f' },
            grid: { color: 'rgba(200, 180, 150, 0.35)' },
          },
        },
      },
    });
    const chartQhist = new Chart(qhistCtx, {
      type: 'bar',
      data: { datasets: [{ label: 'qErr count', data: [], borderColor: '#0d9488', backgroundColor: 'rgba(13, 148, 136, 0.5)', borderWidth: 0, barPercentage: 1.0, categoryPercentage: 1.0, barThickness: 'flex', parsing: false }] },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        plugins: {
          title: { display: true, text: 'qErr histogram', color: '#3d3428', font: { size: 14, weight: '600' } },
          legend: { labels: { color: '#5c5346' } },
        },
        scales: {
          x: {
            type: 'linear',
            min: -2,
            max: 2,
            title: { display: true, text: 'qErr (ns)', color: '#5c5346' },
            ticks: {
              color: '#6b5d4f',
              callback: (v) => {
                const n = Number(v);
                if (!Number.isFinite(n)) return '';
                if (Math.abs(n + 1) < 1e-9) return '-1';
                if (Math.abs(n) < 1e-9) return '0';
                if (Math.abs(n - 1) < 1e-9) return '+1';
                return '';
              },
            },
            grid: { color: 'rgba(200, 180, 150, 0.35)' },
          },
          y: {
            type: 'linear',
            title: { display: true, text: 'count', color: '#5c5346' },
            grace: '10%',
            ticks: { maxTicksLimit: 10, color: '#6b5d4f' },
            grid: { color: 'rgba(200, 180, 150, 0.35)' },
          },
        },
      },
    });
    const chartDac = new Chart(dacCtx, {
      type: 'line',
      data: { datasets: [{ label: 'DAC (counts)', data: [], borderColor: '#7c3aed', backgroundColor: 'rgba(124, 58, 237, 0.07)', pointRadius: 0, tension: 0.12, spanGaps: true, borderWidth: 2 }] },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        plugins: {
          title: { display: true, text: 'DAC', color: '#3d3428', font: { size: 14, weight: '600' } },
          legend: { display: false },
        },
        scales: {
          x: { type: 'linear', ...baseScales.x },
          y: {
            type: 'linear',
            title: { display: true, text: 'PWM counts', color: '#5c5346' },
            grace: '8%',
            ticks: { maxTicksLimit: 10, color: '#6b5d4f' },
            grid: { color: 'rgba(200, 180, 150, 0.35)' },
          },
        },
      },
    });

    const plotCharts = { '#tab-bias': chartBias, '#tab-qhist': chartQhist, '#tab-dac': chartDac };
    document.querySelectorAll('#plotTabs [data-bs-toggle="tab"]').forEach((btn) => {
      btn.addEventListener('shown.bs.tab', () => {
        const ch = plotCharts[btn.getAttribute('data-bs-target')];
        if (ch) requestAnimationFrame(() => ch.resize());
      });
    });
    window.addEventListener('resize', () => {
      Object.values(plotCharts).forEach((ch) => ch.resize());
    });
    requestAnimationFrame(() => chartBias.resize());

    function yOrNull(v) {
      if (v == null) return null;
      const n = Number(v);
      return Number.isFinite(n) ? n : null;
    }
    function applyQerrHistogram(tb, qe, da) {
      const HIST_PS_LIM = 2000; // +/- 2 ns
      const settleS = 2.0;
      const vals = [];
      let lastDac = null;
      let stableAfter = -1e30;
      const n = Math.min(tb.length || 0, qe.length || 0, da.length || 0);
      for (let i = 0; i < n; i++) {
        const t = Number(tb[i]);
        const q = Number(qe[i]);
        const d = Number(da[i]);
        if (Number.isFinite(d)) {
          if (lastDac !== null && Math.abs(d - lastDac) > 0.5) {
            stableAfter = t + settleS;
          }
          lastDac = d;
        }
        if (!Number.isFinite(t) || !Number.isFinite(q)) continue;
        if (t < stableAfter) continue; // Use only settled OCXO samples for estimation histogram.
        if (Math.abs(q) > HIST_PS_LIM) continue;
        vals.push(q);
      }
      if (!vals.length) {
        chartQhist.data.datasets[0].data = [];
        chartQhist.update('none');
        return;
      }
      const vmin = -HIST_PS_LIM, vmax = HIST_PS_LIM;
      const bins = 40;
      const width = (vmax - vmin) > 0 ? (vmax - vmin) / bins : 1;
      const counts = new Array(bins).fill(0);
      for (const v of vals) {
        let i = Math.floor((v - vmin) / width);
        if (i < 0) i = 0;
        if (i >= bins) i = bins - 1;
        counts[i] += 1;
      }
      chartQhist.data.datasets[0].data = counts.map((c, i) => {
        const center_ps = vmin + (i + 0.5) * width;
        return { x: center_ps / 1000.0, y: c };
      });
      chartQhist.update('none');
    }
    function applySeries(d) {
      const tb = d.t || [], cb = d.clkB || [], qe = d.qErr || [], da = d.dac || [];
      const tm = d.t_min, tx = d.t_max;
      const fixX = typeof tm === 'number' && typeof tx === 'number' && Number.isFinite(tm) && Number.isFinite(tx) && tx > tm;
      for (const ch of [chartBias, chartDac]) {
        if (fixX) {
          ch.options.scales.x.min = tm;
          ch.options.scales.x.max = tx;
        } else {
          delete ch.options.scales.x.min;
          delete ch.options.scales.x.max;
        }
      }
      chartBias.data.datasets[0].data = tb.map((t, i) => ({ x: t, y: yOrNull(cb[i]) }));
      chartBias.data.datasets[1].data = tb.map((t, i) => {
        const q = yOrNull(qe[i]);
        return { x: t, y: (q == null ? null : q / 1000.0) };
      });
      chartBias.update('none');
      applyQerrHistogram(tb, qe, da);
      chartDac.data.datasets[0].data = tb.map((t, i) => ({ x: t, y: yOrNull(da[i]) }));
      chartDac.update('none');
    }

    const TELEM_KEY_ORDER = ['lp','tr','fx','sv','iT','tMS','tSub','q','cB','cDb','id','cD','tA','fA','dac','tk'];
    function fmtTelemVal(v) {
      if (v === null || v === undefined) return '';
      if (typeof v === 'number' && Number.isFinite(v)) {
        if (Number.isInteger(v)) return String(v);
        const a = Math.abs(v);
        if (a >= 1e7 || (a > 0 && a < 1e-4)) return v.toExponential(4);
        const s = v.toPrecision(6);
        return s.includes('e') || s.includes('E') ? v.toExponential(4) : s;
      }
      return String(v);
    }
    function formatTelemetryLine(t) {
      const keys = Object.keys(t || {});
      if (!keys.length) return '— waiting for `{…}` telemetry —';
      const ordered = [];
      const seen = new Set();
      for (const k of TELEM_KEY_ORDER) {
        if (k in t) { ordered.push(k); seen.add(k); }
      }
      for (const k of keys.sort()) {
        if (!seen.has(k)) { ordered.push(k); seen.add(k); }
      }
      return ordered.map((k) => k + '=' + fmtTelemVal(t[k])).join('   ');
    }

    /** u-blox NAV-PVT fixType (firmware `fx` field). */
    function describeGnssFix(fx) {
      const n = Number(fx);
      if (!Number.isFinite(n)) return { text: 'No telemetry', sub: 'Enable JSON telemetry and wait for data', variant: 'muted' };
      const rows = [
        ['No fix', 'No position / time solution'],
        ['Dead reckoning', 'Propagation only'],
        ['2D fix', 'Horizontal position'],
        ['3D fix', 'Position + altitude'],
        ['GPS + dead reckoning', 'Combined solution'],
        ['Time only', 'Clock valid, no position'],
      ];
      if (n < 0 || n >= rows.length) return { text: 'Fix type ' + n, sub: 'Unexpected code from receiver', variant: 'muted' };
      return { text: rows[n][0], sub: rows[n][1], variant: n <= 1 ? 'bad' : (n === 2 ? 'warn' : 'ok') };
    }

    /** Firmware JSON `lp`: 0 coarse, 1 fine, 2 inactive (mode free). */
    function describeLoopStage(lp) {
      const n = Number(lp);
      if (lp === undefined || lp === null || !Number.isFinite(n)) {
        return { text: '—', sub: 'Waiting for telemetry field lp', variant: 'muted' };
      }
      if (n === 0) return { text: 'Coarse', sub: 'NAV-CLOCK clkD_bias (not finelock)', variant: 'coarse' };
      if (n === 1) return { text: 'Fine', sub: 'TIM-TP qErr P-only (finelock)', variant: 'fine' };
      if (n === 2) return { text: 'Inactive', sub: 'mode free — DAC not servoed', variant: 'inactive' };
      return { text: 'lp=' + n, sub: 'Unexpected value', variant: 'muted' };
    }
    /** Firmware `tr`: add bits 1 (no fix), 2 (low SV), 4 (fAcc &gt; facc gate). */
    function describeTrackFlags(tr) {
      const n = Number(tr);
      if (tr === undefined || tr === null || !Number.isFinite(n)) {
        return { text: '—', sub: 'Waiting for telemetry field tr', variant: 'muted' };
      }
      if (n === 0) return { text: 'OK', sub: 'No reported GNSS / gate issues', variant: 'ok' };
      const parts = [];
      if (n & 1) parts.push('no fix');
      if (n & 2) parts.push('low satellites (below min_sv)');
      if (n & 4) parts.push('fAcc above gate (coarse DAC blocked)');
      return { text: 'Limited', sub: parts.join(' · '), variant: 'warn' };
    }
    function updateTrackStrip(t) {
      const strip = document.getElementById('trackStrip');
      const titleEl = document.getElementById('trackText');
      const subEl = document.getElementById('trackSub');
      if (!strip || !titleEl || !subEl) return;
      const d = describeTrackFlags(t && t.tr);
      titleEl.textContent = d.text;
      const base = 'tr: 0=OK · 1=no fix · 2=low SV · 4=fAcc gate';
      subEl.textContent = d.sub + ' — ' + base;
      strip.classList.remove('gps-fix-strip--muted', 'gps-fix-strip--bad', 'gps-fix-strip--warn', 'gps-fix-strip--ok');
      strip.classList.remove('track-strip--ok', 'track-strip--warn');
      if (d.variant === 'ok') strip.classList.add('track-strip--ok');
      else if (d.variant === 'warn') strip.classList.add('track-strip--warn');
      else strip.classList.add('gps-fix-strip--muted');
    }

    function updateLoopStageBar(t) {
      const strip = document.getElementById('loopStageStrip');
      const titleEl = document.getElementById('loopStageText');
      const subEl = document.getElementById('loopStageSub');
      if (!strip || !titleEl || !subEl) return;
      const d = describeLoopStage(t && t.lp);
      titleEl.textContent = d.text;
      const base = 'Firmware JSON field lp: 0 = coarse (NAV-CLOCK bias), 1 = fine (TIM-TP qErr), 2 = inactive (mode free)';
      subEl.textContent = d.sub + ' — ' + base;
      strip.classList.remove('gps-fix-strip--muted', 'gps-fix-strip--bad', 'gps-fix-strip--warn', 'gps-fix-strip--ok');
      strip.classList.remove('loop-strip--coarse', 'loop-strip--fine', 'loop-strip--inactive');
      if (d.variant === 'coarse') {
        strip.classList.add('loop-strip--coarse');
      } else if (d.variant === 'fine') {
        strip.classList.add('loop-strip--fine');
      } else if (d.variant === 'inactive') {
        strip.classList.add('loop-strip--inactive');
      } else {
        strip.classList.add('gps-fix-strip--muted');
      }
    }

    function updateGpsFixBar(t) {
      const strip = document.getElementById('gpsFixStrip');
      const titleEl = document.getElementById('gpsFixText');
      const subEl = document.getElementById('gpsFixSub');
      const svEl = document.getElementById('gpsFixSv');
      if (!strip || !titleEl || !subEl || !svEl) return;
      const d = describeGnssFix(t && t.fx);
      titleEl.textContent = d.text;
      subEl.textContent = d.sub;
      strip.classList.remove('gps-fix-strip--muted', 'gps-fix-strip--bad', 'gps-fix-strip--warn', 'gps-fix-strip--ok');
      strip.classList.add('gps-fix-strip--' + d.variant);
      const sv = t && t.sv;
      const sn = Number(sv);
      if (sv !== undefined && sv !== null && Number.isFinite(sn)) {
        svEl.textContent = String(sn);
      } else {
        svEl.textContent = '—';
      }
    }

    async function pollStatus() {
      try {
        const r = await fetch('/api/status');
        const j = await r.json();
        const b = document.getElementById('errBanner');
        if (j.serial_error) {
          b.classList.remove('d-none');
          b.textContent = j.serial_error;
        } else {
          b.classList.add('d-none');
        }
        const telem = j.telemetry || {};
        const tl = document.getElementById('telemetryLine');
        if (tl) tl.textContent = formatTelemetryLine(telem);
        updateGpsFixBar(telem);
        updateLoopStageBar(telem);
        updateTrackStrip(telem);
        const hx = document.getElementById('ubxHexToggle');
        if (hx && typeof j.ubx_hex_forward === 'boolean') {
          hx.checked = j.ubx_hex_forward;
          try { localStorage.setItem('pico.ubxHex', j.ubx_hex_forward ? '1' : '0'); } catch (e) {}
        }
        const jt = document.getElementById('jsonTelemetryToggle');
        if (jt && typeof j.json_telemetry_enabled === 'boolean') {
          jt.checked = j.json_telemetry_enabled;
          try { localStorage.setItem('pico.jsonTelemetry', j.json_telemetry_enabled ? '1' : '0'); } catch (e) {}
        }
        const tf = j.tune_fields || {};
        for (const formId of ['tuneForm', 'gateForm']) {
          const form = document.getElementById(formId);
          if (!form) continue;
          for (const [name, val] of Object.entries(tf)) {
            if (typeof val !== 'string') continue;
            const inp = form.elements.namedItem(name);
            if (!inp || inp === document.activeElement || !('value' in inp)) continue;
            if (inp.value !== val) inp.value = val;
          }
        }
      } catch (e) {}
    }
    function plotWindowMinutes() {
      const el = document.getElementById('plotWindowMin');
      const v = parseInt(el && el.value, 10);
      return [5, 10, 30, 60].includes(v) ? v : 10;
    }
    async function poll() {
      try {
        const w = plotWindowMinutes();
        const r = await fetch('/api/series?window_min=' + encodeURIComponent(String(w)));
        applySeries(await r.json());
      } catch (e) {}
      try {
        const r2 = await fetch('/api/logs');
        const j = await r2.json();
        const el = document.getElementById('log');
        el.textContent = (j.lines || []).join('\\n');
        el.scrollTop = el.scrollHeight;
        const meta = document.getElementById('logMeta');
        if (meta) {
          const tot = j.lines_total, shown = j.lines_shown;
          const lf = j.logfile;
          if (typeof tot === 'number' && typeof shown === 'number') {
            let s = 'Showing last ' + shown + ' of ' + tot + ' lines in RAM';
            if (lf) s += ' · full log: ' + lf;
            meta.textContent = s;
          } else meta.textContent = '';
        }
      } catch (e) {}
      await pollStatus();
    }
    setInterval(poll, 500);
    (function initPlotControls() {
      const sel = document.getElementById('plotWindowMin');
      const LS_W = 'pico.plotWindowMin';
      const saved = localStorage.getItem(LS_W);
      if (saved && ['5', '10', '30', '60', '240', '720'].includes(saved)) sel.value = saved;
      sel.addEventListener('change', () => {
        localStorage.setItem(LS_W, sel.value);
        poll();
      });
      async function postSettings(body) {
        try {
          await fetch('/api/settings', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
          });
        } catch (e) {}
      }
      const jt = document.getElementById('jsonTelemetryToggle');
      const LS_J = 'pico.jsonTelemetry';
      jt.addEventListener('change', async () => {
        const on = jt.checked;
        localStorage.setItem(LS_J, on ? '1' : '0');
        await postSettings({ json_telemetry: on });
      });
      const hx = document.getElementById('ubxHexToggle');
      const LS_H = 'pico.ubxHex';
      hx.addEventListener('change', async () => {
        const on = hx.checked;
        localStorage.setItem(LS_H, on ? '1' : '0');
        await postSettings({ ubx_hex: on });
      });
    })();
    poll();

    async function sendLine(line) {
      await fetch('/api/cmd', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ line }),
      });
    }

    function wireTuneFormSubmit(id) {
      const el = document.getElementById(id);
      if (!el) return;
      el.onsubmit = async (e) => {
        e.preventDefault();
        const fd = new FormData(e.target);
        for (const [k, v] of fd.entries()) {
          if (v && String(v).trim()) await sendLine(k + ' ' + String(v).trim());
        }
      };
    }
    wireTuneFormSubmit('tuneForm');
    wireTuneFormSubmit('gateForm');
    document.getElementById('clearHistoryBtn').onclick = async () => {
      try {
        await fetch('/api/series/clear', { method: 'POST' });
      } catch (e) {}
      poll();
    };
    document.getElementById('dumpBtn').onclick = () => sendLine('dump');
    document.getElementById('servoBtn').onclick = () => sendLine('mode servo');
    document.getElementById('freeBtn').onclick = () => sendLine('mode free');
    document.getElementById('rawSend').onclick = async () => {
      const v = document.getElementById('rawLine').value;
      if (v) await sendLine(v);
    };
  </script>
</body>
</html>
"""


def _parse_line(line: str) -> dict[str, Any]:
    """Extract clkB / qErr from compact `{cB:…,q:…}` lines or legacy `{'clkB':…}` JSON-ish text."""
    d: dict[str, Any] = {}
    m = re.search(r"\bcB:\s*(-?\d+)", line)
    if m:
        d["clkB"] = int(m.group(1))
    m = re.search(r"\bq:\s*(-?\d+)", line)
    if m:
        d["qErr"] = int(m.group(1))
    if "clkB" not in d:
        m = re.search(r"'clkB'\s*:\s*(-?\d+)", line)
        if m:
            d["clkB"] = int(m.group(1))
    if "qErr" not in d:
        m = re.search(r"'qErr'\s*:\s*(-?\d+)", line)
        if m:
            d["qErr"] = int(m.group(1))
    m = re.search(r"\bdac:\s*(\d+)", line)
    if m:
        d["dac"] = int(m.group(1))
    if "dac" not in d:
        m = re.search(r"'dac'\s*:\s*(\d+)", line)
        if m:
            d["dac"] = int(m.group(1))
    return d


def _parse_telemetry_snapshot(line: str) -> dict[str, Any]:
    """Parse compact `{iT:…,cB:…}` lines from firmware. Empty if not a brace telemetry line."""
    s = line.strip()
    if not (s.startswith("{") and s.endswith("}")):
        return {}
    inner = s[1:-1].strip()
    if not inner:
        return {}
    out: dict[str, Any] = {}
    for part in inner.split(","):
        part = part.strip()
        if ":" not in part:
            continue
        k, _, rest = part.partition(":")
        k = k.strip()
        rest = rest.strip()
        if not k:
            continue
        try:
            if any(ch in rest for ch in ".eE"):
                out[k] = float(rest)
            else:
                out[k] = int(rest)
        except ValueError:
            out[k] = rest
    return out


class GpsdoState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.logs: deque[str] = deque(maxlen=MAX_LOG_LINES)
        self.t0 = time.time()
        self.t: list[float] = []
        self.clkB: list[float] = []
        self.qErr: list[float] = []
        self.dac: list[float] = []
        self.telemetry: dict[str, Any] = {}
        self.ser: Any = None
        self.serial_error: str | None = None
        self.ubx_hex_forward: bool = False
        self.json_telemetry_enabled: bool = True
        self.firmware_version: str | None = None
        # Last-known tune / gate values from firmware dump / [t] ack lines (form field name -> str).
        self.tune_fields: dict[str, str] = {}


def _apply_tune_serial_line_locked(text: str, state: GpsdoState) -> None:
    """Update tune_fields and CLI flag mirrors from one CDC line. Caller must hold state.lock."""
    s = text.strip()
    if not s:
        return
    m = re.match(r"^\[t\]\s+json\s+([01])\s*$", s)
    if m:
        state.json_telemetry_enabled = m.group(1) == "1"
        return
    m = re.match(r"^\[t\]\s+ubxhex\s+([01])\s*$", s)
    if m:
        state.ubx_hex_forward = m.group(1) == "1"
        return
    ack_map: list[tuple[re.Pattern[str], str]] = [
        (re.compile(r"^\[t\]\s+kp\s+(\S+)"), "kp"),
        (re.compile(r"^\[t\]\s+kc\s+(\S+)"), "kp_coarse"),
        (re.compile(r"^\[t\]\s+facc\s+(\S+)"), "facc"),
        (re.compile(r"^\[t\]\s+min_sv\s+(\S+)"), "min_sv"),
        (re.compile(r"^\[t\]\s+fe\s+(\S+)"), "fine_enter"),
        (re.compile(r"^\[t\]\s+fx\s+(\S+)"), "fine_exit"),
        (re.compile(r"^\[t\]\s+fe_hold\s+(\S+)"), "fine_enter_hold"),
        (re.compile(r"^\[t\]\s+qmax\s+(\S+)"), "fine_qmax"),
        (re.compile(r"^\[t\]\s+int\s+(\S+)"), "int"),
    ]
    for pat, key in ack_map:
        mm = pat.match(s)
        if mm:
            state.tune_fields[key] = mm.group(1)
            return
    m = re.search(r"\bkp=([\d.eE+-]+)\s+kc=([\d.eE+-]+)\s+int=([\d.eE+-]+)", s)
    if m:
        state.tune_fields["kp"] = m.group(1)
        state.tune_fields["kp_coarse"] = m.group(2)
        state.tune_fields["int"] = m.group(3)
        return
    m = re.search(
        r"\bfacc=(\d+)\s+min_sv=(\d+)\s+fe=(\d+)\s+fx=(\d+)\s+fe_hold=([\d.]+)s",
        s,
    )
    if m:
        state.tune_fields["facc"] = m.group(1)
        state.tune_fields["min_sv"] = m.group(2)
        state.tune_fields["fine_enter"] = m.group(3)
        state.tune_fields["fine_exit"] = m.group(4)
        state.tune_fields["fine_enter_hold"] = m.group(5)
        return
    m = re.search(r"\bqmax=(\d+)", s)
    if m:
        state.tune_fields["fine_qmax"] = m.group(1)
        return
    return


def _serial_reader(tty: str, baud: int, state: GpsdoState, stop: threading.Event) -> None:
    """Read text from USB CDC: drain in multi-kB chunks, split on newlines (UBX is not on this port)."""
    ser = None
    log_fp = None
    try:
        import serial
    except ImportError:
        with state.lock:
            state.serial_error = "pyserial not installed — run: pip install pyserial"
        return
    log_path = _host_logfile_path()
    if log_path:
        try:
            log_fp = open(log_path, "a", encoding="utf-8", buffering=1)
        except OSError:
            log_fp = None
    try:
        ser = serial.Serial(tty, baud, timeout=0.15)
    except Exception as e:
        if log_fp is not None:
            try:
                log_fp.close()
            except OSError:
                pass
        with state.lock:
            state.serial_error = f"cannot open serial {tty}: {e}"
        return
    with state.lock:
        state.ser = ser
    try:
        ser.write(b"dump\n")
    except Exception:
        pass
    buf = bytearray()
    _MAX_SLIP = 256 * 1024
    try:
        while not stop.is_set():
            try:
                chunk = ser.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            buf.extend(chunk)
            if len(buf) > _MAX_SLIP:
                del buf[: len(buf) - _MAX_SLIP // 2]
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw_line = buf[:nl]
                del buf[: nl + 1]
                if raw_line.endswith(b"\r"):
                    raw_line = raw_line[:-1]
                text = raw_line.decode("utf-8", errors="replace")
                if log_fp is not None:
                    try:
                        log_fp.write(text + "\n")
                    except OSError:
                        pass
                with state.lock:
                    state.logs.append(text)
                    _apply_tune_serial_line_locked(text, state)
                    ver_m = re.match(r"^\[ver\]\s+(.+)$", text.strip())
                    if ver_m:
                        state.firmware_version = ver_m.group(1).strip()
                    snap = _parse_telemetry_snapshot(text)
                    if snap:
                        state.telemetry.update(snap)
                    p = _parse_line(text)
                    if not snap and p:
                        if "clkB" in p:
                            state.telemetry["cB"] = p["clkB"]
                        if "qErr" in p:
                            state.telemetry["q"] = p["qErr"]
                    if not p:
                        continue
                    t = time.time() - state.t0
                    state.t.append(t)
                    if snap and "dac" in snap:
                        dac_v = float(snap["dac"])
                    else:
                        dac_v = float(p.get("dac", float("nan")))
                    # NaN = key absent this line.
                    clkb_v = float(p.get("clkB", float("nan")))
                    qerr_v = float(p.get("qErr", float("nan")))
                    state.clkB.append(clkb_v)
                    state.qErr.append(qerr_v)
                    state.dac.append(dac_v)
                    _trim_series_to_retention(state)
    finally:
        with state.lock:
            state.ser = None
        if log_fp is not None:
            try:
                log_fp.close()
            except OSError:
                pass
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def run_dashboard(tty: str, *, host: str = "0.0.0.0", port: int = 8765, baud: int = 9600) -> None:
    try:
        from flask import Flask, Response, jsonify, request
    except ImportError as e:
        raise SystemExit(
            "Flask dashboard requires: pip install -r requirements.txt  (flask, pyserial)"
        ) from e

    state = GpsdoState()
    stop = threading.Event()
    th = threading.Thread(target=_serial_reader, args=(tty, baud, state, stop), daemon=True)
    th.start()

    app = Flask(__name__)

    @app.get("/")
    def index():
        html = _HTML.replace("__TTY__", json.dumps(tty))
        return Response(html, mimetype="text/html")

    @app.get("/api/status")
    def api_status():
        with state.lock:
            return jsonify(
                {
                    "tty": tty,
                    "serial_error": state.serial_error,
                    "telemetry": state.telemetry,
                    "ubx_hex_forward": state.ubx_hex_forward,
                    "json_telemetry_enabled": state.json_telemetry_enabled,
                    "firmware_version": state.firmware_version,
                    "tune_fields": dict(state.tune_fields),
                }
            )

    @app.get("/api/settings")
    def api_settings_get():
        return jsonify(
            {
                "retain_s": _plot_retain_s(),
                "window_minutes_choices": list(ALLOWED_WINDOW_MIN),
                "default_window_min": 10,
                "max_points_cap": _max_points_cap(),
            }
        )

    @app.post("/api/settings")
    def api_settings_post():
        data = request.get_json(silent=True) or {}
        ser = None
        with state.lock:
            ser = state.ser
        if "ubx_hex" in data:
            en = bool(data["ubx_hex"])
            with state.lock:
                state.ubx_hex_forward = en
            if ser is not None:
                try:
                    ser.write(f"ubxhex {1 if en else 0}\n".encode("utf-8"))
                except Exception:
                    pass
        if "json_telemetry" in data:
            jn = bool(data["json_telemetry"])
            with state.lock:
                state.json_telemetry_enabled = jn
            if ser is not None:
                try:
                    ser.write(f"json {1 if jn else 0}\n".encode("utf-8"))
                except Exception:
                    pass
        with state.lock:
            ubx = state.ubx_hex_forward
            jtel = state.json_telemetry_enabled
        return jsonify({"ok": True, "ubx_hex_forward": ubx, "json_telemetry_enabled": jtel})

    @app.get("/api/series")
    def api_series():
        wmin = request.args.get("window_min", default=10, type=int)
        if wmin not in ALLOWED_WINDOW_MIN:
            wmin = 10
        win = float(wmin * 60)
        with state.lock:
            t, cb, qe, da = _slice_last_seconds(
                list(state.t),
                list(state.clkB),
                list(state.qErr),
                list(state.dac),
                win,
            )
        # Fixed [t_min, t_max] so Chart.js x-axis matches the chosen window (avoids jumps when the
        # first/last sample times do not sit exactly on the slice boundaries).
        t_max: float | None = max(t) if t else None
        t_min: float | None = (t_max - win) if t_max is not None else None
        return jsonify(
            {
                "t": t,
                "window_s": win,
                "window_min": wmin,
                "t_min": t_min,
                "t_max": t_max,
                "clkB": _json_safe_series(cb),
                "qErr": _json_safe_series(qe),
                "dac": _json_safe_series(da),
            }
        )

    @app.post("/api/series/clear")
    def api_series_clear():
        with state.lock:
            state.t.clear()
            state.clkB.clear()
            state.qErr.clear()
            state.dac.clear()
            state.t0 = time.time()
        return jsonify({"ok": True})

    @app.get("/api/logs")
    def api_logs():
        cap = _ui_log_line_cap()
        logfile = _host_logfile_path()
        with state.lock:
            full = list(state.logs)
        n = len(full)
        tail = full[-cap:] if n > cap else full
        return jsonify(
            {
                "lines": tail,
                "lines_total": n,
                "lines_shown": len(tail),
                "logfile": logfile,
            }
        )

    @app.post("/api/cmd")
    def api_cmd():
        data = request.get_json(silent=True) or {}
        line = (data.get("line") or "").strip()
        if not line:
            return jsonify({"ok": False, "error": "empty line"}), 400
        with state.lock:
            ser = state.ser
        if ser is None:
            return jsonify({"ok": False, "error": "serial not ready"}), 503
        if not line.endswith("\n"):
            line += "\n"
        ser.write(line.encode("utf-8"))
        return jsonify({"ok": True})

    try:
        lp = _host_logfile_path()
        if lp:
            print(f"[web] Host log file (append): {lp}", flush=True)
        else:
            print("[web] Host log file disabled (set PICO_GPSDO_LOGFILE to a path to enable)", flush=True)
        print(f"[web] UI log tail: last {_ui_log_line_cap()} lines (PICO_UI_LOG_LINES)", flush=True)
        print(f"[web] Listening on http://{host}:{port}/  (Ctrl+C to stop)", flush=True)
        if host in ("0.0.0.0", "::", ""):
            print(f"[web] On this machine open: http://127.0.0.1:{port}/", flush=True)
        app.run(host=host, port=port, threaded=True, use_reloader=False)
    finally:
        stop.set()
        th.join(timeout=2.0)
