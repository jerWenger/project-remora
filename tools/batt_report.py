#!/usr/bin/env python3
"""Energy use from the BMS history (battery_litime.py CSVs), alone or matched to a dataflash log.

    scp boat@highfieldboat.tailcacf9.ts.net:battlogs/batt_$(date +%Y%m%d).csv logs/batt/
    .venv/bin/python tools/batt_report.py logs/batt/batt_20261115.csv                       # the day
    .venv/bin/python tools/batt_report.py logs/batt/batt_20261115.csv --log logs/ap/..._14.BIN
    .venv/bin/python tools/batt_report.py batt_*.csv --from 15:10 --to 15:40               # UTC window

Alone: duration, SoC start -> end, Ah and Wh drawn (integrated from the readings), peak
current and power, lowest pack and cell voltage, widest cell spread.

With --log: the dataflash GPS week/ms gives every log sample a UTC time, the CSV has the
Jetson's (NTP) UTC, so each BMS reading gets the boat's speed and commanded speed at that
moment. Output: pack power while idle (the hotel load), then one row per speed: mean W and A,
Wh per km, and the range and endurance on --usable-wh (default 4096 = 80 % of 5.12 kWh, the
plan's "abort at 20 % SoC"). Rows come from commanded-speed plateaus in Auto/Guided when the
log has them (the tune mission: each speed is held on opposite legs, so wind and current
cancel), otherwise from 0.5 m/s bins of measured speed. A reading counts only if the speed
stayed within --band m/s for the --settle s before it.

The BMS current sign is taken from the data (the side with the bigger currents is discharge).
Requires: numpy; pymavlink for --log.
"""
import argparse
import csv
import datetime as dt
import os
import sys

import numpy as np

GPS_EPOCH, LEAP = 315964800, 18  # GPS-UTC = 18 s since 2017


def read_csv(paths):
    rows = []
    for p in paths:
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    if int(r["online"]) != 1:
                        continue
                    cmin = float(r["cell_min"]) if r.get("cell_min") else np.nan
                    spread = float(r["cell_spread_mv"]) if r.get("cell_spread_mv") else np.nan
                    temp = float(r["temp"]) if r.get("temp") not in ("", "None", None) else np.nan
                    rows.append((float(r["unix"]), float(r["voltage"]), float(r["current"]), float(r["soc"]), cmin, spread, temp))
                except (KeyError, ValueError):
                    continue
    if not rows:
        raise SystemExit("no online BMS readings in those files")
    a = np.array(sorted(rows))
    return a[np.concatenate([[True], np.diff(a[:, 0]) > 0])]


def discharge(a):
    """Current with discharge positive, whatever the BMS convention is."""
    cur = a[:, 2]
    neg, pos = np.abs(cur[cur < -1]).sum(), cur[cur > 1].sum()
    sign = -1.0 if neg > pos else 1.0
    return sign * cur, ("negative" if sign < 0 else "positive")


def integrate(t, y, max_gap=10.0):
    """Trapezoid over time in hours, skipping gaps (service restarts, BMS offline)."""
    d = np.diff(t)
    ok = d <= max_gap
    return float((0.5 * (y[1:] + y[:-1]) * d)[ok].sum() / 3600.0), float(d[ok].sum())


def utc(s):
    return dt.datetime.fromtimestamp(s, dt.timezone.utc).strftime("%H:%M:%S")


def summary(a, dis, title):
    t = a[:, 0]
    ah, live = integrate(t, np.clip(dis, 0, None))
    wh, _ = integrate(t, np.clip(dis, 0, None) * a[:, 1])
    chg, _ = integrate(t, np.clip(-dis, 0, None))
    print(f"{title}: {utc(t[0])}-{utc(t[-1])} UTC, {live / 60:.1f} min of readings ({len(t)}, median every {np.median(np.diff(t)):.1f} s)")
    print(f"  SoC {a[0, 3]:.0f} % -> {a[-1, 3]:.0f} %   drawn {ah:.2f} Ah, {wh:.0f} Wh" + (f"   charged {chg:.2f} Ah" if chg > 0.05 else ""))
    i = int(np.argmax(dis))
    print(f"  peak {dis[i]:.1f} A / {dis[i] * a[i, 1]:.0f} W at {utc(t[i])}; mean while drawing > 2 A: "
          + (f"{dis[dis > 2].mean():.1f} A" if (dis > 2).any() else "never"))
    print(f"  pack {a[:, 1].min():.2f}-{a[:, 1].max():.2f} V; lowest cell {np.nanmin(a[:, 4]):.3f} V; "
          f"widest cell spread {np.nanmax(a[:, 5]):.0f} mV; temperature {np.nanmin(a[:, 6]):.0f}-{np.nanmax(a[:, 6]):.0f} C")
    return wh


def read_log(path):
    from pymavlink import mavutil
    cache = path + ".batt.npz"
    if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(path):
        z = np.load(cache)
        return {k: z[k] for k in z.files}
    print(f"reading {path} (once; cached) ...", flush=True)
    m = mavutil.mavlink_connection(path)
    g, th, mo, ar = [], [], [], []
    while True:
        x = m.recv_match(type=["GPS", "THR", "MODE", "ARM"])
        if x is None:
            break
        t = x.get_type()
        if t == "GPS":
            if getattr(x, "I", 0) == 0 and x.Status >= 3 and x.GWk > 0:
                g.append((x.TimeUS / 1e6, GPS_EPOCH + x.GWk * 604800 + x.GMS / 1000.0 - LEAP, x.Spd))
        elif t == "THR":
            th.append((x.TimeUS / 1e6, x.DesSpeed, x.ThrOut))
        elif t == "MODE":
            mo.append((x.TimeUS / 1e6, x.ModeNum))
        else:
            ar.append((x.TimeUS / 1e6, x.ArmState))
    d = dict(GPS=np.array(g, float), THR=np.array(th, float), MODE=np.array(mo, float), ARM=np.array(ar, float))
    np.savez(cache, **d)
    return d


def step_at(ev, t, default=0.0):
    """Value of a step signal [(t, v)] at times t."""
    if not len(ev):
        return np.full(len(t), default)
    i = np.searchsorted(ev[:, 0], t, side="right") - 1
    return np.where(i >= 0, ev[np.clip(i, 0, None), 1], default)


def by_speed(a, dis, d, usable, settle, band):
    g = d["GPS"]
    if len(g) < 10:
        raise SystemExit("no GPS fix with a week number in the log: cannot place it in time")
    off = float(np.median(g[:, 1] - g[:, 0]))          # log seconds -> unix
    lt = a[:, 0] - off                                 # BMS readings on the log's clock
    inlog = (lt >= g[0, 0]) & (lt <= g[-1, 0])
    print(f"\nlog spans {utc(g[0, 1])}-{utc(g[-1, 1])} UTC; {int(inlog.sum())} BMS readings fall inside it")
    if inlog.sum() < 5:
        raise SystemExit("the BMS file and the log do not overlap (wrong day? Jetson clock?)")
    lt, v, i_d = lt[inlog], a[inlog, 1], dis[inlog]
    w = v * i_d
    summary(a[inlog], i_d, "during the log")
    spd = np.interp(lt, g[:, 0], g[:, 2])
    th = d["THR"]
    des = np.interp(lt, th[:, 0], th[:, 1])
    thr = np.interp(lt, th[:, 0], th[:, 2])
    armed = step_at(d["ARM"], lt) >= 1
    auto = np.isin(step_at(d["MODE"], lt, -1), (10, 11, 12, 15))

    def held(sig_t, sig, tol):
        """True where sig stayed within tol of its value at lt for the settle window before it."""
        out = np.zeros(len(lt), bool)
        for k, t in enumerate(lt):
            m = (sig_t >= t - settle) & (sig_t <= t)
            out[k] = m.sum() > 3 and np.ptp(sig[m]) <= 2 * tol
        return out

    idle = (~armed | (np.abs(thr) < 1)) & (spd < 0.3)
    if idle.sum() >= 3:
        print(f"\nidle (not driving): {np.median(w[idle]):.0f} W, {np.median(i_d[idle]):.2f} A  ({int(idle.sum())} readings): the hotel load")
    steady_spd = held(g[:, 0], g[:, 2], band)
    plateau = auto & armed & held(th[:, 0], th[:, 1], 0.1) & steady_spd & (des > 0.2) & (np.abs(spd - des) < 2 * band)
    if plateau.sum() >= 10:
        key, label, use = np.round(des * 4) / 4, "commanded-speed plateaus (Auto/Guided)", plateau
    else:
        key, label, use = np.round(spd * 2) / 2, "0.5 m/s bins of measured speed", armed & steady_spd & (spd > 0.3)
    print(f"\npower by speed, from {label}; usable {usable:.0f} Wh:")
    print(f"{'m/s':>5} {'kn':>5} {'n':>4} {'thr %':>6} {'A':>6} {'W':>6} {'+-':>5} {'Wh/km':>6} {'range km':>9} {'hours':>6}")
    rows = []
    for k in sorted(set(key[use])):
        m = use & (key == k)
        if m.sum() < 15:      # a few readings are a corner or a speed change, not a plateau
            continue
        s, p = float(spd[m].mean()), float(w[m].mean())
        whkm = p / (s * 3.6)
        rows.append((s, p))
        print(f"{s:5.2f} {s * 1.944:5.1f} {int(m.sum()):4d} {thr[m].mean():6.0f} {i_d[m].mean():6.1f} {p:6.0f} {w[m].std():5.0f} "
              f"{whkm:6.1f} {usable / whkm:9.1f} {usable / p:6.1f}")
    if not rows:
        print("  (no speed was held long enough; lower --settle or drive steadier legs)")
    elif len(rows) >= 2:
        best = min((r for r in rows if r[0] >= 0.8), key=lambda r: r[1] / r[0], default=rows[-1])
        print(f"most distance per Wh at {best[0]:.1f} m/s. Range = still water, this wind, this load; the pack's own SoC is the check.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="battlogs/batt_YYYYMMDD.csv file(s)")
    ap.add_argument("--log", help="dataflash .BIN to match against")
    ap.add_argument("--from", dest="t0", help="UTC HH:MM[:SS] window start (first file's date)")
    ap.add_argument("--to", dest="t1", help="UTC HH:MM[:SS] window end")
    ap.add_argument("--usable-wh", type=float, default=4096.0, help="energy to plan range on (default 4096 = 80 %% of 5.12 kWh)")
    ap.add_argument("--settle", type=float, default=8.0, help="s the speed must have been steady before a reading counts (default 8)")
    ap.add_argument("--band", type=float, default=0.15, help="m/s that counts as steady (default 0.15)")
    a = ap.parse_args()
    data = read_csv(a.csv)
    day = dt.datetime.fromtimestamp(data[0, 0], dt.timezone.utc).date()
    for bound, lo in ((a.t0, True), (a.t1, False)):
        if bound:
            hms = [int(x) for x in bound.split(":")] + [0]
            ts = dt.datetime(day.year, day.month, day.day, hms[0], hms[1], hms[2], tzinfo=dt.timezone.utc).timestamp()
            data = data[data[:, 0] >= ts] if lo else data[data[:, 0] <= ts]
    if len(data) < 2:
        raise SystemExit("no readings in that window")
    dis, sign = discharge(data)
    print(f"BMS current: discharge is {sign} in this file")
    wh = summary(data, dis, "BMS")
    soc_drop = data[0, 3] - data[-1, 3]
    if soc_drop >= 3:
        print(f"  cross-check: {wh:.0f} Wh for {soc_drop:.0f} % SoC = {100 * wh / soc_drop:.0f} Wh per 100 % (nameplate 5120)")
    if a.log:
        by_speed(data, dis, read_log(a.log), a.usable_wh, a.settle, a.band)


if __name__ == "__main__":
    sys.exit(main())
