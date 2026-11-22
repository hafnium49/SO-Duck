"""Reproduce an ARM-ONLY Jacobian survey, not whole-body foot-contact IK."""
import argparse
import csv
from pathlib import Path
import numpy as np
from scipy.stats import qmc
from arm_model import LIMITS, frames, fk

CANDIDATES = {
    "stock_upright": (0, 0, 0, False),
    "retained_terminal_upright": (90, 0, 0, True),
    "retained_terminal_forward": (90, 0, 90, True),
}

def calculate(n=8192, seed=20261122, length=.15):
    if n < 1 or n & (n-1):
        raise ValueError("Sample size must be a positive power of two")
    unit = qmc.Sobol(6, scramble=True, seed=seed).random_base2(int(np.log2(n)))
    q = qmc.scale(unit, LIMITS[:, 0], LIMITS[:, 1])
    out = []
    for name, spec in CANDIDATES.items():
        _, j = fk(q, frames(*spec), length=length)
        sv = np.linalg.svd(j, compute_uv=False)
        low = sv[:, -1]
        high = sv[:, 0]
        out.append(dict(layout=name, sample_size=n, characteristic_length_m=length,
                        smin_p05=float(np.quantile(low, .05)),
                        smax_p95=float(np.quantile(high, .95)),
                        condition_p95=float(np.quantile(high / np.maximum(low, 1e-15), .95)),
                        fraction_rank6=float(np.mean(low > 1e-8))))
    return out

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--samples", type=int, default=8192)
    p.add_argument("--output", default="arm_only_results.csv")
    p.add_argument("--length", type=float, default=.15)
    args = p.parse_args()
    rows = calculate(args.samples, length=args.length)
    output = Path(args.output)
    with output.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    for row in rows:
        print(row)
