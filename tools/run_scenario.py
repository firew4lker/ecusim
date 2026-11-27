#!/usr/bin/env python3
"""Run a headless scenario, write a CSV of every signal and a multi-panel PNG plot.

Examples:
    python3 tools/run_scenario.py cold-start --ambient 15 --seconds 960
    python3 tools/run_scenario.py free-rev
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ecusim.scenarios import Recording, cold_start_idle, warm_free_rev  # noqa: E402

PANELS = [
    ("Engine speed (rpm)", ["rpm", "rpm_target"]),
    ("Temperatures (°C)", ["ect_c", "oil_c", "iat_c"]),
    ("Catalyst (°C)", ["cat_c", "cat_est_c"]),
    ("Air", ["map_kpa", "maf_gps", "throttle_pct"]),
    ("Lambda", ["lambda_cmd", "lambda_wb", "lambda_comb"]),
    ("Fuel trims (%)", ["stft_pct", "ltft_pct"]),
    ("Spark (° BTDC)", ["spark_deg", "mbt_deg"]),
    ("Injector PW (ms)", ["pw_ms"]),
    ("Downstream O2 (V)", ["nb_volts"]),
    ("Fuel status / flags", ["fuel_status", "cat_heat", "dfco"]),
]


def write_csv(rec: Recording, path: Path) -> None:
    keys = list(rec.rows[0].keys())
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rec.rows:
            w.writerow({k: f"{v:.4f}" for k, v in r.items()})


def plot(rec: Recording, path: Path, title: str, t0: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [(r["t"] - t0) for r in rec.rows]
    fig, axes = plt.subplots(len(PANELS), 1, figsize=(13, 2.1 * len(PANELS)), sharex=True)
    for ax, (label, keys) in zip(axes, PANELS):
        for k in keys:
            ax.plot(t, rec.column(k), label=k, linewidth=0.9)
        ax.set_ylabel(label, fontsize=8)
        ax.legend(loc="upper right", fontsize=7, ncol=len(keys))
        ax.grid(alpha=0.3)
    axes[0].set_title(title)
    axes[-1].set_xlabel("time since start (s)")
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", choices=["cold-start", "free-rev"])
    ap.add_argument("--ambient", type=float, default=15.0, help="soak/ambient temperature °C (cold-start)")
    ap.add_argument("--seconds", type=float, default=960.0, help="duration after start (cold-start)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", type=Path, default=Path("out"))
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    if args.scenario == "cold-start":
        rec, crank, t0 = cold_start_idle(args.ambient, args.seconds, args.seed)
        name = f"cold_start_{args.ambient:g}C"
        title = f"Cold start at {args.ambient:g} °C (crank {crank:.2f} s)"
    else:
        rec, _ = warm_free_rev(args.seed)
        t0 = rec.rows[0]["t"]
        name = "free_rev"
        title = "Warm free-rev"

    csv_path = args.out / f"{name}.csv"
    write_csv(rec, csv_path)
    print(f"wrote {csv_path} ({len(rec.rows)} rows)")
    if not args.no_plot:
        png = args.out / f"{name}.png"
        plot(rec, png, title, t0)
        print(f"wrote {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
