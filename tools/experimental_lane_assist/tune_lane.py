#!/usr/bin/env python3
"""Tune lane-controller gains with short closed-loop CARLA trials (hill climbing), live in the terminal.

Keep seg_lane_node.py running in its own terminal (.venv-ml), then in .venv-carla:
    python tune_lane.py --controllers pure-pursuit stanley pid mpc --trials 12 --trial-s 30 --spawn 6 --speed 3
Each trial runs lane_drive.py for --trial-s seconds with one gain set. Trial 1 is the defaults; later
trials perturb the best set so far, with a shrinking step. Unknown flags (e.g. --map Town03) are passed
through to lane_drive.py. Results: <out>/results.csv, <out>/best.json, one folder per trial.

Score (lower is better) = RMS lateral error vs the map lane centre [m] + 0.05 * departure % (>1.5 m)
+ 0.02 * lost-lane stop % + 3 * fraction of the trial cut short (kart left the lane).
"""
import argparse
import csv
import json
import math
import random
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from compare_lane_drives import metrics  # noqa: E402

# flag -> (default, low, high, is_int)
SPACE = {
    "pure-pursuit": {"--kp": (1.0, 0.4, 1.6, False), "--ld-time": (1.5, 0.7, 3.0, False)},
    "stanley": {"--stanley-k": (1.0, 0.1, 3.0, False), "--stanley-softening": (0.5, 0.2, 3.0, False)},
    "pid": {"--pid-kp": (0.7, 0.1, 1.5, False), "--pid-ki": (0.03, 0.0, 0.1, False),
            "--pid-kd": (0.15, 0.0, 0.4, False)},
    "mpc": {"--mpc-horizon": (8, 4, 16, True), "--mpc-step": (0.2, 0.1, 0.4, False)},
}


def fmt(v):
    return str(int(v)) if float(v).is_integer() else f"{v:.3f}"


def perturb(best, space, scale, rng):
    out = {}
    for flag, (_, lo, hi, is_int) in space.items():
        v = min(hi, max(lo, best[flag] + rng.gauss(0, scale * (hi - lo))))
        out[flag] = int(round(v)) if is_int else round(v, 4)
    return out


def score(folder, trial_s):
    try:
        m = metrics(folder)
    except (OSError, ValueError, KeyError):
        return 1e3, None
    with (folder / "drive.csv").open(newline="") as f:
        last_t = max(float(r["t"]) for r in csv.DictReader(f))
    cut = max(0.0, 1.0 - last_t / trial_s)
    cut = cut if cut > 0.05 else 0.0
    return m["rms"] + 0.05 * m["departures"] + 0.02 * m["stopped"] + 3.0 * cut, m


def live(proc, csv_path, label):
    """Show progress while the driver runs by reading its drive.csv."""
    while proc.poll() is None:
        time.sleep(1.0)
        try:
            with csv_path.open(newline="") as f:
                rows = [r for r in csv.DictReader(f) if r.get("true_offset")]
        except OSError:
            rows = []
        if rows:
            err = [float(r["true_offset"]) for r in rows[-100:]]
            rms = math.sqrt(sum(e * e for e in err) / len(err))
            print(f"\r  {label} t={float(rows[-1]['t']):5.1f}s  err={err[-1]:+.2f} m  recent rms={rms:.2f} m   ",
                  end="", flush=True)
    print("\r" + " " * 70 + "\r", end="")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--controllers", nargs="+", choices=list(SPACE), default=list(SPACE))
    ap.add_argument("--trials", type=int, default=12, help="trials per controller")
    ap.add_argument("--trial-s", type=float, default=30.0, help="seconds per trial")
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--spawn", type=int, default=0)
    ap.add_argument("--out", default="runs/tune")
    ap.add_argument("--driver", default=str(HERE / "lane_drive.py"))
    ap.add_argument("--pause", type=float, default=3.0, help="s between trials")
    ap.add_argument("--seed", type=int, default=0)
    a, passthrough = ap.parse_known_args()

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(a.seed)
    new = not (out / "results.csv").exists()
    res = (out / "results.csv").open("a", newline="")
    w = csv.writer(res)
    if new:
        w.writerow(["controller", "trial", "score", "rms", "max", "departures_pct", "stopped_pct", "params"])
    best_all = {}

    for name in a.controllers:
        space = SPACE[name]
        best, best_score = {f: d for f, (d, *_rest) in space.items()}, math.inf
        print(f"\n=== {name}: {a.trials} trials x {a.trial_s:.0f} s ===")
        for i in range(1, a.trials + 1):
            params = dict(best) if i == 1 else perturb(best, space, 0.3 * 0.85 ** (i - 2), rng)
            folder = out / f"{name}-{i:02d}"
            cmd = [sys.executable, a.driver, "--controller", name, "--duration", str(a.trial_s),
                   "--speed", str(a.speed), "--spawn", str(a.spawn), "--out", str(folder)]
            for flag, v in params.items():
                cmd += [flag, fmt(v)]
            cmd += passthrough
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
            live(proc, folder / "drive.csv", f"{name} {i}/{a.trials}")
            err = proc.stderr.read()
            s, m = score(folder, a.trial_s)
            if m is None:
                print(f"  trial {i:2d} FAILED (no data). {err.strip().splitlines()[-1] if err.strip() else ''}")
            ptxt = " ".join(f"{k.lstrip('-')}={fmt(v)}" for k, v in params.items())
            if m:
                improved = s < best_score
                if improved:
                    best, best_score = params, s
                print(f"  trial {i:2d}  score {s:6.3f}  rms {m['rms']:.2f}  max {m['max']:.2f}  "
                      f"depart {m['departures']:4.1f}%  stop {m['stopped']:4.1f}%  {ptxt}"
                      f"{'  <- best' if improved else ''}")
                w.writerow([name, i, f"{s:.4f}", f"{m['rms']:.4f}", f"{m['max']:.4f}",
                            f"{m['departures']:.2f}", f"{m['stopped']:.2f}", ptxt])
                res.flush()
            time.sleep(a.pause)
        best_all[name] = {"score": best_score, "params": best}
        print(f"  best {name}: score {best_score:.3f}  " + " ".join(f"{k} {fmt(v)}" for k, v in best.items()))
        (out / "best.json").write_text(json.dumps(best_all, indent=2))

    print("\n=== ranking (lower is better) ===")
    for name, r in sorted(best_all.items(), key=lambda kv: kv[1]["score"]):
        print(f"{name:<14} {r['score']:7.3f}  " + " ".join(f"{k} {fmt(v)}" for k, v in r["params"].items()))
    print("Final check: re-run the winners with compare_lane_drives.py over a longer --duration (30 s trials can overfit).")


if __name__ == "__main__":
    main()
