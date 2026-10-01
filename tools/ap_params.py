#!/usr/bin/env python3
"""Compare / query ArduPilot .parm dumps produced by ap_inventory.py (or QGC / Mission Planner).

    ./ap_params.py diff params/2026-10-01.parm params/2026-10-08.parm
    ./ap_params.py get  params/2026-10-01.parm SERVO MOT_        # prefix match

Accepts "NAME,value" (QGC/MP) and "NAME value" (MAVProxy) lines; '#' lines ignored.
"""
import sys


def load(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.replace("\t", " ").replace(",", " ").split()
            if len(parts) >= 2:
                try:
                    out[parts[0]] = float(parts[1])
                except ValueError:
                    pass
    return out


def fmt(v):
    return f"{v:g}"


def cmd_diff(a_path, b_path):
    a, b = load(a_path), load(b_path)
    changed = [(k, a[k], b[k]) for k in sorted(a) if k in b and a[k] != b[k]]
    only_a = sorted(set(a) - set(b))
    only_b = sorted(set(b) - set(a))
    print(f"{len(a)} params in A, {len(b)} in B, {len(changed)} changed")
    for k, va, vb in changed:
        print(f"  {k:18s} {fmt(va):>12s} -> {fmt(vb)}")
    if only_a:
        print(f"\nonly in A ({len(only_a)}): " + " ".join(only_a))
    if only_b:
        print(f"\nonly in B ({len(only_b)}): " + " ".join(only_b))
    return 1 if (changed or only_a or only_b) else 0


def cmd_get(path, prefixes):
    p = load(path)
    for k in sorted(p):
        if not prefixes or any(k.startswith(x) for x in prefixes):
            print(f"  {k:18s} {fmt(p[k])}")
    return 0


def main(argv):
    if len(argv) >= 4 and argv[1] == "diff":
        return cmd_diff(argv[2], argv[3])
    if len(argv) >= 3 and argv[1] == "get":
        return cmd_get(argv[2], argv[3:])
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
