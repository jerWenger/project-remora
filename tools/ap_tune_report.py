#!/usr/bin/env python3
"""Turn a Rover dataflash log into tuning numbers: steady runs, step responses, starting gains.

    .venv/bin/python tools/ap_tune_report.py logs/ap/2026-11-08_16-46-12_9.BIN
    .venv/bin/python tools/ap_tune_report.py LOG.BIN --vcb-limit 600 --out logs/tune/

No markers needed: it reads the motor outputs (SERVO9/10, found by function 73/74 in the
log's PARM) and finds
  * steady segments: both outputs held for >= --min-steady s. For each, speed over ground
    and yaw rate in the second half (after the boat has settled), and the GPS course.
  * steps: a jump in mean thrust (speed step) or in differential thrust (yaw step) followed
    by a steady hold. Each is fitted with a first-order response y0 + dy*(1-exp(-(t-d)/tau)).

Outputs are converted to VCB thrust in permille with the 0.3.x map: 25 us deadband, then
(|us-1500|-25)*1000/475, clamped at the VCB limit. The limit defaults to the map at
SERVO9_MAX (1620 -> 200, 1810 -> 600), which is right whenever the SERVO range ends at
the clamp; pass --vcb-limit otherwise.

Sign conventions: T = (left+right)/2 (positive = forward), D = (left-right)/2 (positive =
left pushes harder = bow to starboard = heading increasing). Yaw rate in deg/s from the
unwrapped EKF yaw, positive clockwise.

Rover's own units, used for the suggestions: throttle -100..100 % of the SERVO range,
steering -1..1, ATC_STR_RAT_FF in (steering fraction) per (rad/s), ATC_STR_RAT_MAX in deg/s.
Suggestions are STARTING values from open-loop data, to be checked on the water.
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
TYPES = ["THR", "STER", "RCOU", "GPS", "ATT", "ARM", "MODE", "PARM"]
DT = 0.1  # resample grid, s


def vcb_permille(us, limit):
    d = np.asarray(us, float) - 1500.0
    mag = np.clip((np.abs(d) - 25.0) * 1000.0 / 475.0, 0.0, limit)
    return np.sign(d) * mag


# ----------------------------------------------------------------------------- extract
def extract(path):
    """Read the message types we need once; cache as <log>.npz beside the log."""
    cache = path + ".tune.npz"
    if os.path.exists(cache) and os.path.getmtime(cache) >= os.path.getmtime(path):
        z = np.load(cache, allow_pickle=True)
        return {k: z[k] for k in z.files}
    print(f"reading {path} (once; cached to {os.path.basename(cache)}) ...", flush=True)
    m = mavutil.mavlink_connection(path)
    rows = {t: [] for t in TYPES}
    params = {}
    while True:
        x = m.recv_match(type=TYPES)
        if x is None:
            break
        t = x.get_type()
        if t == "PARM":
            params[x.Name] = x.Value
        elif t == "THR":
            rows[t].append((x.TimeUS, x.ThrIn, x.ThrOut, x.DesSpeed, x.Speed))
        elif t == "STER":
            rows[t].append((x.TimeUS, x.SteerIn, x.SteerOut, x.DesTurnRate, x.TurnRate))
        elif t == "RCOU":
            rows[t].append((x.TimeUS,) + tuple(getattr(x, f"C{i}", 0) for i in range(1, 15)))
        elif t == "GPS":
            if getattr(x, "I", 0) == 0:
                rows[t].append((x.TimeUS, x.Spd, x.GCrs, x.Lat, x.Lng, x.Status))
        elif t == "ATT":
            rows[t].append((x.TimeUS, x.Yaw, x.DesYaw))
        elif t == "ARM":
            rows[t].append((x.TimeUS, x.ArmState))
        elif t == "MODE":
            rows[t].append((x.TimeUS, x.ModeNum))
    d = {t: np.array(r, float) for t, r in rows.items() if t != "PARM"}
    d["PARAMS"] = np.array(sorted(params.items()), dtype=object)
    np.savez(cache, **d)
    return d


# ----------------------------------------------------------------------------- series
def armed_intervals(arm, t_end):
    out, start = [], None
    for t, s in arm:
        if s >= 1 and start is None:
            start = t
        elif s < 1 and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, t_end))
    return out


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def smooth(y, n):
    if n <= 1:
        return y
    k = np.ones(n) / n
    return np.convolve(np.pad(y, (n // 2, n - 1 - n // 2), mode="edge"), k, mode="valid")


def build(d, limit_arg):
    P = {k: float(v) for k, v in d["PARAMS"]}
    fn = {int(P.get(f"SERVO{i}_FUNCTION", 0)): i for i in range(1, 15)}
    if 73 not in fn or 74 not in fn:
        raise SystemExit("no SERVOn_FUNCTION 73/74 (ThrottleLeft/Right) in this log's PARM")
    ch_l, ch_r = fn[73], fn[74]
    smax = P.get(f"SERVO{ch_r}_MAX", 2000.0)
    limit = limit_arg if limit_arg else float(vcb_permille([smax], 1000)[0])

    rc = d["RCOU"]
    t_all = rc[:, 0] / 1e6
    ints = armed_intervals(d["ARM"][:, :2] / [1e6, 1] if len(d["ARM"]) else [], t_all[-1])
    grid = np.concatenate([np.arange(a, b, DT) for a, b in ints]) if ints else np.array([])
    seg_id = np.concatenate([np.full(len(np.arange(a, b, DT)), i) for i, (a, b) in enumerate(ints)]) if ints else np.array([])

    def interp(arr, col):
        return np.interp(grid, arr[:, 0] / 1e6, arr[:, col])

    s = {"t": grid, "seg": seg_id}
    s["us_l"] = interp(rc, ch_l)
    s["us_r"] = interp(rc, ch_r)
    s["pl"] = vcb_permille(s["us_l"], limit)
    s["pr"] = vcb_permille(s["us_r"], limit)
    s["T"] = (s["pl"] + s["pr"]) / 2
    s["D"] = (s["pl"] - s["pr"]) / 2
    g = d["GPS"]
    s["spd"] = interp(g, 1)
    s["crs"] = interp(np.column_stack([g[:, 0], unwrap_deg(g[:, 2])]), 1) % 360
    s["lat"], s["lng"] = interp(g, 3), interp(g, 4)
    a = d["ATT"]
    yaw_u = unwrap_deg(a[:, 1])
    s["yaw"] = np.interp(grid, a[:, 0] / 1e6, yaw_u)
    s["rate"] = smooth(np.gradient(s["yaw"], DT), 5)
    for seg in np.unique(seg_id):  # no gradient across disarmed gaps
        idx = np.where(seg_id == seg)[0]
        if len(idx) > 2:
            s["rate"][idx] = smooth(np.gradient(s["yaw"][idx], DT), 5)
    th, st = d["THR"], d["STER"]
    s["thr_out"], s["des_spd"], s["thr_spd"] = interp(th, 2), interp(th, 3), interp(th, 4)
    s["str_out"], s["des_rate"], s["ster_rate"] = interp(st, 2), interp(st, 3), interp(st, 4)
    mo = d["MODE"]
    s["mode"] = np.array([mo[mo[:, 0] / 1e6 <= t, 1][-1] if np.any(mo[:, 0] / 1e6 <= t) else -1 for t in grid]) if len(mo) else np.full(len(grid), -1)
    return s, P, limit, ints, (ch_l, ch_r)


# ----------------------------------------------------------------------------- analysis
def steady_segments(s, min_len, tol):
    """Greedy: longest runs where both outputs stay within tol permille of the run's start."""
    out, n, i = [], len(s["t"]), 0
    while i < n:
        j = i + 1
        while (j < n and s["seg"][j] == s["seg"][i]
               and abs(s["pl"][j] - s["pl"][i]) <= tol and abs(s["pr"][j] - s["pr"][i]) <= tol):
            j += 1
        if (j - i) * DT >= min_len and (abs(s["pl"][i]) > 0 or abs(s["pr"][i]) > 0):
            h = i + (j - i) // 2  # second half: settled
            crs = s["crs"][h:j]
            out.append(dict(
                t0=s["t"][i], dur=(j - i) * DT, T=s["T"][i:j].mean(), D=s["D"][i:j].mean(),
                us_l=s["us_l"][i:j].mean(), us_r=s["us_r"][i:j].mean(),
                spd=s["spd"][h:j].mean(), spd_sd=s["spd"][h:j].std(),
                rate=s["rate"][h:j].mean(), rate_sd=s["rate"][h:j].std(),
                crs=float(np.degrees(np.arctan2(np.sin(np.radians(crs)).mean(), np.cos(np.radians(crs)).mean())) % 360),
                mode=int(s["mode"][i])))
            i = j
        else:
            i += 1
    return out


def fit_first_order(t, y, y0):
    """y = y0 + dy*(1-exp(-(t-d)/tau)) for t>d. Grid over d, tau; dy by least squares."""
    best = None
    for d in np.arange(0.0, 1.6, 0.1):
        for tau in np.geomspace(0.2, 30, 80):
            x = np.where(t > d, 1 - np.exp(-(t - d) / tau), 0.0)
            den = (x * x).sum()
            if den == 0:
                continue
            dy = ((y - y0) * x).sum() / den
            r = ((y - y0 - dy * x) ** 2).mean()
            if best is None or r < best[0]:
                best = (r, d, tau, dy)
    r, d, tau, dy = best
    var = np.var(y) if np.var(y) > 0 else 1.0
    return dict(delay=d, tau=tau, dy=dy, r2=1 - r / var)


def steps(s, jump, hold, pre=2.0):
    """Jumps in T (speed) or D (yaw) of >= jump permille within 1 s, steady for >= hold s after."""
    out, n, k = [], len(s["t"]), int(1.0 / DT)
    i = int(pre / DT)
    while i < n - k:
        same_seg = s["seg"][i - int(pre / DT)] == s["seg"][min(i + k + int(hold / DT), n - 1)]
        for kind, key, other in (("speed", "T", "D"), ("yaw", "D", "T")):
            dj = s[key][i + k] - s[key][i]
            if not same_seg or abs(dj) < jump:
                continue
            a = i + k
            b = a
            while b < n - 1 and abs(s[key][b + 1] - s[key][a]) <= 15 and abs(s[other][b + 1] - s[other][a]) <= 15:
                b += 1
            pre_sl = slice(i - int(pre / DT), i)
            if (b - a) * DT < hold or np.ptp(s[key][pre_sl]) > 15 or np.ptp(s[other][pre_sl]) > 15:
                continue
            ysig = s["spd"] if kind == "speed" else s["rate"]
            y0 = ysig[pre_sl].mean()
            tt = s["t"][i:b] - s["t"][i]
            f = fit_first_order(tt, ysig[i:b], y0)
            f.update(kind=kind, t0=s["t"][i], u0=s[key][i], u1=s[key][a], other=s[other][a],
                     y0=y0, K=f["dy"] / (s[key][a] - s[key][i]), hold=(b - a) * DT)
            out.append(f)
            i = b
            break
        i += 1
    return out


# ----------------------------------------------------------------------------- report
MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 10: "AUTO", 11: "RTL", 15: "GUIDED"}


def report(path, s, P, limit, ints, chans, segs, stp, out_dir):
    print(f"\n== {os.path.basename(path)}")
    print(f"VCB limit {limit:.0f} permille (SERVO{chans[1]}_MAX {P.get(f'SERVO{chans[1]}_MAX', 0):.0f}); "
          f"left = SERVO{chans[0]}, right = SERVO{chans[1]}")
    print("Logged settings: " + ", ".join(f"{k} {P[k]:g}" for k in (
        "CRUISE_SPEED", "CRUISE_THROTTLE", "MOT_THR_MIN", "ATC_SPEED_P", "ATC_SPEED_I",
        "ATC_STR_RAT_FF", "ATC_STR_RAT_P", "ATC_STR_RAT_I", "ATC_STR_RAT_MAX", "ATC_STR_ANG_P") if k in P))
    tot = sum(b - a for a, b in ints)
    print(f"Armed {len(ints)}x, {tot:.0f} s total: " + ", ".join(f"{a:.0f}-{b:.0f}" for a, b in ints if b - a > 20))
    modes = sorted(set(int(m) for m in s["mode"]))
    print("Modes while armed: " + ", ".join(MODES.get(m, str(m)) for m in modes))

    print(f"\nSteady segments ({len(segs)}): outputs held, values from the settled second half")
    print(f"{'t (s)':>7} {'dur':>5} {'L us':>5} {'R us':>5} {'T ‰':>5} {'D ‰':>5} {'m/s':>5} {'±':>4} {'°/s':>6} {'±':>4} {'course':>6}  mode")
    for g in segs:
        print(f"{g['t0']:7.0f} {g['dur']:5.1f} {g['us_l']:5.0f} {g['us_r']:5.0f} {g['T']:5.0f} {g['D']:5.0f} "
              f"{g['spd']:5.2f} {g['spd_sd']:4.2f} {g['rate']:6.1f} {g['rate_sd']:4.1f} {g['crs']:6.0f}  {MODES.get(g['mode'], g['mode'])}")

    print(f"\nSteps ({len(stp)}): first-order fit y0 + K*du*(1-exp(-(t-delay)/tau))")
    print(f"{'t (s)':>7} {'kind':>5} {'from':>5} {'to':>5} {'other':>5} {'hold':>5} {'K':>8} {'tau s':>6} {'dly':>4} {'R²':>5}")
    for f in stp:
        unit = "m/s/‰" if f["kind"] == "speed" else "°/s/‰"
        print(f"{f['t0']:7.0f} {f['kind']:>5} {f['u0']:5.0f} {f['u1']:5.0f} {f['other']:5.0f} {f['hold']:5.1f} "
              f"{f['K']:8.4f} {f['tau']:6.2f} {f['delay']:4.1f} {f['r2']:5.2f}  {unit}")

    # --- starting values
    print("\nStarting values (open-loop data; check on the water):")
    span = P.get(f"SERVO{chans[1]}_MAX", 2000.0) - 1500.0
    rover_thr = lambda g: 100 * ((g["us_l"] + g["us_r"]) / 2 - 1500) / span   # Rover throttle %, pre-MOT_THR_MIN
    rover_str = lambda g: ((g["us_l"] - g["us_r"]) / 2) / span                # Rover steering fraction
    fwd = [g for g in segs if g["T"] > 20 and abs(g["D"]) < 0.25 * g["T"] and g["spd_sd"] < 0.3]
    if fwd:
        # pair opposite courses when possible to cancel wind/current
        T = np.array([g["T"] for g in fwd]); V = np.array([g["spd"] for g in fwd])
        k = (T * V).sum() / (T * T).sum()
        mid = min(fwd, key=lambda g: abs(g["T"] - 0.5 * limit))
        print(f"  speed ≈ {k * 100:.3f} m/s per 100 ‰ mean thrust (through origin, {len(fwd)} straight runs; "
              f"at the limit ≈ {k * limit:.2f} m/s)")
        print(f"  CRUISE_THROTTLE {rover_thr(mid):.0f}  CRUISE_SPEED {mid['spd']:.2f}   "
              f"(the straight run nearest half the band, t={mid['t0']:.0f} s, course {mid['crs']:.0f}°; "
              f"average both directions before writing it)")
    else:
        print("  no straight steady runs: drive card item A (straight, 20 s per stick setting, both ways)")
    turns = [g for g in segs if abs(g["D"]) > 20 and abs(g["rate"]) > 2]
    if turns:
        ff = [abs(rover_str(g)) / math.radians(abs(g["rate"])) for g in turns]
        rmax = max(abs(g["rate"]) for g in turns)
        print(f"  ATC_STR_RAT_FF ≈ {np.median(ff):.2f} (median of {len(turns)} steady turns, "
              f"range {min(ff):.2f}-{max(ff):.2f}; steering fraction per rad/s)")
        print(f"  ATC_STR_RAT_MAX / ACRO_TURN_RATE ≲ {rmax:.0f} °/s (fastest steady turn seen"
              f"{'' if any(abs(rover_str(g)) > 0.9 for g in turns) else ', not at full steering: true max is higher'})")
    else:
        print("  no steady turns: drive card items C/D (full steering, held, both ways)")
    sp = [f for f in stp if f["kind"] == "speed" and f["r2"] > 0.6]
    yw = [f for f in stp if f["kind"] == "yaw" and f["r2"] > 0.6]
    if sp:
        print(f"  speed time constant ≈ {np.median([f['tau'] for f in sp]):.1f} s ({len(sp)} good steps)")
    if yw:
        print(f"  yaw-rate time constant ≈ {np.median([f['tau'] for f in yw]):.1f} s ({len(yw)} good steps)")
    if not sp and not yw:
        print("  no clean steps: drive card items E/F (snap the stick, hold 10-20 s)")

    plots(path, s, ints, segs, stp, limit, out_dir)


def plots(path, s, ints, segs, stp, limit, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    base = os.path.join(out_dir, os.path.splitext(os.path.basename(path))[0])
    for k, (a, b) in enumerate(ints):
        if b - a < 30:
            continue
        m = (s["t"] >= a) & (s["t"] < b)
        t = s["t"][m] - a
        fig, ax = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
        ax[0].plot(t, s["pl"][m], label="left ‰"); ax[0].plot(t, s["pr"][m], label="right ‰")
        ax[0].axhline(limit, ls=":", c="k"); ax[0].axhline(-limit, ls=":", c="k")
        ax[0].set_ylabel("VCB thrust ‰"); ax[0].legend(loc="upper right")
        ax[1].plot(t, s["spd"][m], label="GPS speed"); ax[1].plot(t, s["des_spd"][m], label="Rover desired", alpha=.7)
        ax[1].set_ylabel("m/s"); ax[1].legend(loc="upper right")
        ax[2].plot(t, s["rate"][m], label="yaw rate (EKF yaw)"); ax[2].plot(t, s["des_rate"][m], label="Rover desired", alpha=.7)
        ax[2].set_ylabel("°/s"); ax[2].legend(loc="upper right")
        ax[3].plot(t, s["yaw"][m] % 360, ".", ms=1, label="heading"); ax[3].plot(t, s["crs"][m], ".", ms=1, label="GPS course")
        ax[3].set_ylabel("°"); ax[3].set_xlabel(f"s since arm at {a:.0f} s"); ax[3].legend(loc="upper right")
        for g in segs:
            if a <= g["t0"] < b:
                for x in ax:
                    x.axvspan(g["t0"] - a, g["t0"] - a + g["dur"], color="g", alpha=.08)
        for f in stp:
            if a <= f["t0"] < b:
                for x in ax:
                    x.axvline(f["t0"] - a, color="r", alpha=.4)
        ax[0].set_title(f"{os.path.basename(path)}  armed {a:.0f}-{b:.0f} s  (green = steady, red = step)")
        fig.tight_layout(); fig.savefig(f"{base}_armed{k}.png", dpi=110); plt.close(fig)
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.plot(s["lng"][m], s["lat"][m], lw=.8); ax.set_aspect(1 / math.cos(math.radians(s["lat"][m].mean())))
        ax.set_title(f"track, armed {a:.0f}-{b:.0f} s"); fig.tight_layout()
        fig.savefig(f"{base}_armed{k}_track.png", dpi=110); plt.close(fig)
    if segs:
        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        ax[0].scatter([g["T"] for g in segs], [g["spd"] for g in segs], c=[abs(g["D"]) for g in segs], cmap="viridis")
        ax[0].set_xlabel("mean thrust T ‰"); ax[0].set_ylabel("speed m/s"); ax[0].set_title("speed vs thrust (colour = |D|)")
        ax[1].scatter([g["D"] for g in segs], [g["rate"] for g in segs], c=[g["T"] for g in segs], cmap="plasma")
        ax[1].set_xlabel("differential D ‰"); ax[1].set_ylabel("yaw rate °/s"); ax[1].set_title("turn rate vs differential (colour = T)")
        fig.tight_layout(); fig.savefig(f"{base}_steady.png", dpi=110); plt.close(fig)
    print(f"\nplots: {base}_*.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--vcb-limit", type=float, default=None, help="VCB clamp in permille (default: from SERVO MAX)")
    ap.add_argument("--min-steady", type=float, default=6.0, help="s held to count as steady (default 6)")
    ap.add_argument("--tol", type=float, default=15.0, help="permille wobble allowed in a steady run (default 15)")
    ap.add_argument("--step", type=float, default=40.0, help="permille jump that counts as a step (default 40)")
    ap.add_argument("--out", default=os.path.join(REPO, "logs", "tune"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    d = extract(a.log)
    s, P, limit, ints, chans = build(d, a.vcb_limit)
    if not len(s["t"]):
        raise SystemExit("never armed in this log")
    segs = steady_segments(s, a.min_steady, a.tol)
    stp = steps(s, a.step, hold=5.0)
    report(a.log, s, P, limit, ints, chans, segs, stp, a.out)


if __name__ == "__main__":
    sys.exit(main())
