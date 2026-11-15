#!/usr/bin/env python3
"""Score a Rover Auto (or Guided) run from its dataflash log: is the steering weaving, how
far behind is the turn rate, does the speed loop hold each commanded speed, and what the
yaw plant looks like from this run. The closed-loop companion of ap_tune_report.py.

    .venv/bin/python tools/ap_auto_report.py logs/ap/2026-11-12_21-27-25_13.BIN
    .venv/bin/python tools/ap_auto_report.py LOG.BIN --out logs/tune/

For every armed stretch in Auto/Guided longer than --min s it prints
  steering   share of time the desired turn rate sits at ATC_STR_RAT_MAX and the steering
             output is saturated, the dominant period and size of the yaw-rate oscillation,
             the lag from desired to achieved rate, motor direction changes per minute
  yaw plant  rate = K * steering through a first-order lag tau and a delay, fitted to this
             run; 1/K is the ATC_STR_RAT_FF this run asks for
  path       cross-track error
  speed      one row per commanded speed held >= 8 s: achieved speed, throttle
and a verdict line per loop. Plot: <out>/<log>_auto<k>.png.

Model behind the verdict (checked against log 13, 2026-11-12): in Auto the position
controller turns cross-track velocity error into lateral acceleration, PSC_VEL_P, and that
into a turn rate; around a straight line it is a heading loop of gain PSC_VEL_P [1/s] closed
around the yaw lag. With tau 1.3 s + 0.7 s delay it goes unstable near PSC_VEL_P ~0.7 and
weaves with a 6-8 s period; ~0.3-0.4 leaves margin.
Requires: pymavlink, numpy, matplotlib (all in .venv).
"""
import argparse
import math
import os
import sys

import numpy as np

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit("pip install pymavlink")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TYPES = ["PIDS", "PIDA", "STER", "THR", "NTUN", "RCOU", "MODE", "ARM", "PARM", "POS", "CMD"]
FIELDS = {
    "PIDS": ("Tar", "Act", "P", "I", "FF"), "PIDA": ("Tar", "Act", "P", "I", "FF"),
    "STER": ("SteerOut", "DesTurnRate", "TurnRate"), "THR": ("ThrOut", "DesSpeed", "Speed"),
    "NTUN": ("XTrack",), "MODE": ("ModeNum",), "ARM": ("ArmState",),
    "POS": ("Lat", "Lng"), "CMD": ("CNum", "CId", "Lat", "Lng"),
}
MODES = {10: "AUTO", 15: "GUIDED", 11: "RTL", 12: "SMARTRTL"}


def extract(path):
    cache = path + ".auto2.npz"
    if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(path):
        z = np.load(cache, allow_pickle=True)
        return {k: z[k] for k in z.files}
    print(f"reading {path} (once; cached to {os.path.basename(cache)}) ...", flush=True)
    m = mavutil.mavlink_connection(path)
    rows, params = {t: [] for t in TYPES if t != "PARM"}, {}
    while True:
        x = m.recv_match(type=TYPES)
        if x is None:
            break
        t = x.get_type()
        if t == "PARM":
            params[x.Name] = x.Value
        elif t == "RCOU":
            rows[t].append((x.TimeUS,) + tuple(getattr(x, f"C{i}", 0) for i in range(1, 15)))
        else:
            rows[t].append((x.TimeUS,) + tuple(getattr(x, f) for f in FIELDS[t]))
    d = {t: np.array(r, float) for t, r in rows.items()}
    d["PARAMS"] = np.array(sorted(params.items()), dtype=object)
    np.savez(cache, **d)
    return d


def intervals(d, min_len):
    """Armed stretches in an autonomous mode: [(t0, t1, mode)]."""
    t_end = d["RCOU"][-1, 0] / 1e6
    ev = sorted([(t / 1e6, "arm", v) for t, v in d["ARM"]] + [(t / 1e6, "mode", v) for t, v in d["MODE"]])
    out, armed, mode, start = [], False, -1, None
    for t, kind, v in ev + [(t_end, "end", 0)]:
        if start is not None:
            if t - start >= min_len:
                out.append((start, t, mode))
            start = None
        if kind == "arm":
            armed = v >= 1
        elif kind == "mode":
            mode = int(v)
        if kind != "end" and armed and mode in MODES:
            start = t
    return out


def fit_plant(t, u, y):
    """y = K * lag(tau){u delayed by d}; grid over d and tau, K by least squares."""
    dt = float(np.median(np.diff(t)))
    best = None
    for d in np.arange(0.0, 2.01, 0.1):
        ud = np.interp(t - d, t, u)
        for tau in np.geomspace(0.3, 8, 40):
            a = math.exp(-dt / tau)
            x = np.zeros(len(t))
            for i in range(1, len(t)):
                x[i] = a * x[i - 1] + (1 - a) * ud[i - 1]
            den = float(np.dot(x, x))
            if den <= 0:
                continue
            k = float(np.dot(x, y)) / den
            e = float(((y - k * x) ** 2).mean())
            if best is None or e < best[0]:
                best = (e, d, tau, k)
    e, d, tau, k = best
    return dict(K=k, tau=tau, delay=d, r2=1 - e / max(float(np.var(y)), 1e-12))


def analyse(d, P, t0, t1, skip):
    def seg(k):
        a = d[k]
        t = a[:, 0] / 1e6
        m = (t >= t0 + skip) & (t < t1)
        return t[m], a[m]

    r = {}
    t, p = seg("PIDS")
    if len(t) < 200:
        return None
    dt = float(np.median(np.diff(t)))
    tar, act = p[:, 1], p[:, 2]
    rmax = math.radians(P.get("ATC_STR_RAT_MAX", 0)) or np.inf
    r["at_rate_max"] = float((np.abs(tar) >= 0.98 * rmax).mean())
    r["pid_sat"] = float((np.abs(p[:, 3] + p[:, 4] + p[:, 5]) >= 1).mean())
    r["tar_p99"], r["act_p99"] = (float(np.degrees(np.percentile(np.abs(v), 99))) for v in (tar, act))
    y = act - act.mean()
    f, Y = np.fft.rfftfreq(len(y), dt), np.abs(np.fft.rfft(y))
    band = (f > 1 / 30) & (f < 1.0)
    k = np.flatnonzero(band)[Y[band].argmax()]
    r["period"] = float(1 / f[k])
    r["osc_share"] = float((Y[max(k - 2, 1):k + 3] ** 2).sum() / max((Y[1:] ** 2).sum(), 1e-12))
    r["act_rms"] = float(np.degrees(act.std()))
    a = tar - tar.mean()
    lags = np.arange(0, int(6 / dt))
    r["lag"] = float(lags[int(np.argmax([np.dot(a[:len(a) - l], y[l:]) for l in lags]))] * dt)

    fn = {int(P.get(f"SERVO{i}_FUNCTION", 0)): i for i in range(1, 15)}
    tr, rc = seg("RCOU")
    L, R = rc[:, fn[73]], rc[:, fn[74]]
    trim, span = P.get(f"SERVO{fn[73]}_TRIM", 1500), P.get(f"SERVO{fn[73]}_MAX", 2000) - P.get(f"SERVO{fn[73]}_TRIM", 1500)
    for name, s in (("rev_l", L), ("rev_r", R)):
        sg = np.sign(np.where(np.abs(s - trim) < 25, 0, s - trim))
        sg = sg[sg != 0]
        r[name] = 60.0 * float((np.diff(sg) != 0).sum()) / (t1 - t0 - skip)
    r["full_opposed"] = float(((np.abs(L - trim) > 0.95 * span) & (np.abs(R - trim) > 0.95 * span) & ((L - trim) * (R - trim) < 0)).mean())
    u = np.interp(t, tr, (L - R) / 2 / span)
    tt, th = seg("THR")
    spd = np.interp(t, tt, th[:, 3])
    r["speed_mean"] = float(spd.mean())
    r["plant"] = fit_plant(t, u, act) if np.std(u) > 0.05 else None

    tn, nt = seg("NTUN")
    r["xt_rms"], r["xt_max"] = float(np.sqrt((nt[:, 1] ** 2).mean())), float(np.abs(nt[:, 1]).max())

    # commanded-speed plateaus: desired speed within 0.05 m/s for >= 8 s; skip the first 5 s of each
    rows, i, n = [], 0, len(tt)
    des, out, sp = th[:, 2], th[:, 1], th[:, 3]
    while i < n:
        j = i + 1
        while j < n and abs(des[j] - des[i]) <= 0.05:
            j += 1
        if tt[j - 1] - tt[i] >= 8 and des[i] > 0.2:
            m = tt[i:j] >= tt[i] + 5
            if m.sum() > 5:
                rows.append((round(float(des[i:j][m].mean()), 2), tt[j - 1] - tt[i] - 5, float(sp[i:j][m].mean()),
                             float(sp[i:j][m].std()), float(out[i:j][m].mean())))
        i = j
    levels = {}
    for dv, dur, s, sd, o in rows:
        levels.setdefault(round(dv * 4) / 4, []).append((dur, s, sd, o))
    r["levels"] = {k: tuple(float(np.average([x[c] for x in v], weights=[x[0] for x in v])) if c else float(sum(x[0] for x in v))
                            for c in range(4)) for k, v in sorted(levels.items())}
    ta, pa = seg("PIDA")
    r["spd_err_rms"] = float(np.sqrt(((pa[:, 1] - pa[:, 2]) ** 2).mean()))
    tp, po = seg("POS")
    r["track"] = po[:, 1:3]
    r["series"] = dict(t=t - t0, tar=np.degrees(tar), act=np.degrees(act), ff=p[:, 5], pp=p[:, 3], ii=p[:, 4],
                       tt=tt - t0, des=th[:, 2], spd=th[:, 3], thr=th[:, 1], tr=tr - t0, L=L, R=R, tn=tn - t0, xt=nt[:, 1])
    return r


def verdict(r, P):
    out = []
    vel_p = P.get("PSC_VEL_P", float("nan"))
    weave = 3 < r["period"] < 15 and r["osc_share"] > 0.3 and r["act_rms"] > 4
    if weave and (r["at_rate_max"] > 0.2 or r["pid_sat"] > 0.2):
        out.append(f"STEERING: limit cycle, period {r['period']:.1f} s, saturated {100 * r['pid_sat']:.0f} % of the time. "
                   f"Lower PSC_VEL_P (now {vel_p:g}) by a third to a half; do not touch the rate gains first.")
    elif weave:
        out.append(f"STEERING: weaving, period {r['period']:.1f} s, {r['act_rms']:.1f} deg/s rms, not saturated. "
                   f"Lower PSC_VEL_P (now {vel_p:g}) by ~25 %.")
    else:
        out.append(f"STEERING: no sustained weave ({r['act_rms']:.1f} deg/s rms yaw rate). "
                   f"If corners are cut wide and xtrack settles slowly, raise PSC_VEL_P (now {vel_p:g}) by 0.1.")
    pl = r["plant"]
    if pl and pl["r2"] <= 0.85:
        out.append(f"RATE FF: plant fit too loose to act on (R2 {pl['r2']:.2f}; a run that mixes speeds or mostly drives "
                   f"straight does that). Keep ATC_STR_RAT_FF {P.get('ATC_STR_RAT_FF', float('nan')):g}.")
    elif pl:
        ff, now = 1 / pl["K"], P.get("ATC_STR_RAT_FF", float("nan"))
        note = "keep" if abs(ff - now) / now < 0.15 else f"set {ff:.1f}"
        out.append(f"RATE FF: this run fits {ff:.2f} at {r['speed_mean']:.1f} m/s (now {now:g}): {note}. "
                   f"Full steering gives {math.degrees(pl['K']):.0f} deg/s here; keep ATC_STR_RAT_MAX at or under ~80 % of that.")
    worst = max(r["levels"].items(), key=lambda kv: abs(kv[1][1] - kv[0]), default=None)
    if worst:
        e = worst[1][1] - worst[0]
        out.append(f"SPEED: worst plateau {worst[0]:.2f} m/s -> {worst[1][1]:.2f} ({e:+.2f}); "
                   + ("within 0.2 m/s: keep ATC_SPEED_*." if abs(e) <= 0.2 else
                      "steady error: raise ATC_SPEED_I 0.2 -> 0.3; if it is the same sign at every speed, fix CRUISE_SPEED/THROTTLE from the table instead."))
    return out


def plot(r, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    s = r["series"]
    fig, ax = plt.subplots(5, 1, figsize=(13, 12), sharex=True)
    ax[0].plot(s["tt"], s["des"], label="desired"); ax[0].plot(s["tt"], s["spd"], label="actual"); ax[0].set_ylabel("speed m/s")
    ax[1].plot(s["t"], s["tar"], label="desired"); ax[1].plot(s["t"], s["act"], label="actual"); ax[1].set_ylabel("turn rate deg/s")
    ax[2].plot(s["t"], s["ff"], label="FF"); ax[2].plot(s["t"], s["pp"], label="P"); ax[2].plot(s["t"], s["ii"], label="I")
    ax[2].axhline(1, ls=":", c="k"); ax[2].axhline(-1, ls=":", c="k"); ax[2].set_ylabel("steering terms (+-1 = full)")
    ax[3].plot(s["tr"], s["L"], label="left"); ax[3].plot(s["tr"], s["R"], label="right"); ax[3].set_ylabel("us")
    ax[4].plot(s["tn"], s["xt"], label="cross-track"); ax[4].set_ylabel("m"); ax[4].set_xlabel("s since the mode was entered")
    for a in ax:
        a.legend(loc="upper right"); a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(path, dpi=100); plt.close(fig)


def plot_track(r, d, t0, t1, path, title):
    """Track of this stretch with the mission's waypoints (as logged in CMD), numbered."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tr = r["track"]
    if len(tr) < 10:
        return False
    lat0, lon0 = tr[0]
    kx = 111320 * math.cos(math.radians(lat0))
    fig, ax = plt.subplots(figsize=(11, 9))
    ax.plot((tr[:, 1] - lon0) * kx, (tr[:, 0] - lat0) * 111320, lw=1, label="track")
    ax.plot(0, 0, "g^", ms=9, label="start")
    c = d.get("CMD", np.zeros((0, 5)))
    wps = {}
    for row in c:                                  # last upload wins; nav waypoints with a position only
        if row[0] / 1e6 > t1:
            break                                  # a mission uploaded after this stretch
        if int(row[2]) == 16 and int(row[1]) > 0 and abs(row[3]) > 1:
            wps[int(row[1])] = (row[3], row[4])
        else:
            wps.pop(int(row[1]), None)             # that slot now holds a DO command, not a waypoint
    if wps:
        ks = sorted(wps)
        e = [(wps[k][1] - lon0) * kx for k in ks]
        n = [(wps[k][0] - lat0) * 111320 for k in ks]
        ax.plot(e, n, "r.--", lw=1, ms=7, label="mission")
        seen = set()
        for k, x, y in zip(ks, e, n):
            key = (round(x), round(y))
            if key not in seen:                    # repeated laps: label each spot once
                seen.add(key)
                ax.annotate(str(k), (x, y), fontsize=8, color="r", xytext=(3, 3), textcoords="offset points")
    ax.set_aspect(1); ax.grid(alpha=.3); ax.legend(loc="best")
    ax.set_xlabel("m east of start"); ax.set_ylabel("m north of start"); ax.set_title(title)
    fig.tight_layout(); fig.savefig(path, dpi=100); plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--min", type=float, default=40.0, help="shortest autonomous stretch to report, s (default 40)")
    ap.add_argument("--skip", type=float, default=10.0, help="s ignored at the start of each stretch (default 10)")
    ap.add_argument("--out", default=os.path.join(REPO, "logs", "tune"))
    a = ap.parse_args()
    d = extract(a.log)
    P = {k: float(v) for k, v in d["PARAMS"]}
    ints = intervals(d, a.min)
    if not ints:
        raise SystemExit("no armed Auto/Guided stretch of that length in this log")
    os.makedirs(a.out, exist_ok=True)
    print(f"\n== {os.path.basename(a.log)}")
    print("Logged: " + ", ".join(f"{k} {P[k]:g}" for k in (
        "PSC_POS_P", "PSC_VEL_P", "PSC_VEL_I", "ATC_STR_RAT_FF", "ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_MAX",
        "ATC_STR_ACC_MAX", "ATC_TURN_MAX_G", "ATC_SPEED_P", "ATC_SPEED_I", "CRUISE_SPEED", "CRUISE_THROTTLE",
        "WP_SPEED", "WP_RADIUS", "WP_PIVOT_ANGLE", "WP_PIVOT_RATE") if k in P))
    for k, (t0, t1, mode) in enumerate(ints):
        r = analyse(d, P, t0, t1, a.skip)
        if r is None:
            continue
        print(f"\n-- {MODES[mode]} {t0:.0f}-{t1:.0f} s ({t1 - t0:.0f} s), mean {r['speed_mean']:.2f} m/s")
        print(f"steering  desired rate at ATC_STR_RAT_MAX {100 * r['at_rate_max']:.0f} % of the time, FF+P+I beyond full steering "
              f"{100 * r['pid_sat']:.0f} %, motors full and opposed {100 * r['full_opposed']:.0f} %")
        print(f"          yaw rate {r['act_rms']:.1f} deg/s rms (99th pct desired {r['tar_p99']:.0f}, achieved {r['act_p99']:.0f}); "
              f"dominant period {r['period']:.1f} s carrying {100 * r['osc_share']:.0f} % of it; desired -> achieved lag {r['lag']:.1f} s")
        print(f"          direction changes per minute: left {r['rev_l']:.1f}, right {r['rev_r']:.1f}")
        pl = r["plant"]
        if pl:
            print(f"yaw plant K {pl['K']:.3f} rad/s ({math.degrees(pl['K']):.1f} deg/s) per full steering, tau {pl['tau']:.2f} s, "
                  f"delay {pl['delay']:.1f} s, R2 {pl['r2']:.2f} -> FF {1 / pl['K']:.2f}")
        print(f"path      cross-track rms {r['xt_rms']:.2f} m, max {r['xt_max']:.2f} m")
        print(f"speed     error rms {r['spd_err_rms']:.2f} m/s")
        for lv, (dur, s, sd, o) in r["levels"].items():
            print(f"          {lv:4.2f} m/s commanded for {dur:4.0f} s: achieved {s:.2f} +- {sd:.2f}, throttle {o:.0f} %")
        for line in verdict(r, P):
            print("  > " + line)
        png = os.path.join(a.out, f"{os.path.splitext(os.path.basename(a.log))[0]}_auto{k}.png")
        plot(r, png)
        print(f"plot: {png}")
        trk = png.replace(f"_auto{k}.png", f"_track{k}.png")
        if plot_track(r, d, t0, t1, trk, f"{os.path.basename(a.log)}  {MODES[mode]} {t0:.0f}-{t1:.0f} s"):
            print(f"track: {trk}")


if __name__ == "__main__":
    sys.exit(main())
