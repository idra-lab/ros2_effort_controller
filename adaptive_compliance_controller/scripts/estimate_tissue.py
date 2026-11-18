#!/usr/bin/env python3
"""Offline nonnegative Hunt-Crossley fit from already compensated contact data.

CSV: penetration_m, penetration_rate_m_s, contact_force_n (positive compression).
Fit each palpation location independently, then interpolate/export a tissue map
outside the control loop. No robot motion or online calibration is performed.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def estimate(penetration, rate, force, beta=1.35):
    p, v, f = (np.asarray(x, dtype=float) for x in (penetration, rate, force))
    if p.ndim != 1 or p.shape != v.shape or p.shape != f.shape or len(p) < 3:
        raise ValueError('Need at least three aligned one-dimensional samples')
    if not np.isfinite(beta) or beta < 1 or not all(np.isfinite(x).all() for x in (p, v, f)):
        raise ValueError('Nonfinite samples or invalid beta')
    if (p <= 0).any() or (f < 0).any():
        raise ValueError('Fit requires strictly positive penetration and nonnegative compressive force')
    a = np.column_stack((p**beta, p**beta * v))
    scale = np.linalg.norm(a, axis=0)
    if (scale <= 0).any() or np.linalg.matrix_rank(a / scale) < 2:
        raise ValueError('Insufficient dynamic excitation to identify stiffness and viscosity')
    unconstrained, *_ = np.linalg.lstsq(a / scale, f, rcond=None)
    candidates = [np.zeros(2)]
    coefficient = unconstrained / scale
    if (coefficient >= 0).all():
        candidates.append(coefficient)
    for i in range(2):
        boundary = np.zeros(2)
        boundary[i] = max(0., a[:, i] @ f / (a[:, i] @ a[:, i]))
        candidates.append(boundary)
    best = min(candidates, key=lambda x: np.linalg.norm(a @ x - f))
    if best[0] <= 0:
        raise ValueError('Samples do not identify a positive elastic coefficient')
    return dict(kappa=float(best[0]), viscosity=float(best[1]), beta=float(beta),
                samples=len(p), rmse_n=float(np.sqrt(np.mean((a @ best - f)**2))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--beta', type=float, default=1.35)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    with args.csv.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    result = estimate(*([float(r[key]) for r in rows] for key in
        ('penetration_m', 'penetration_rate_m_s', 'contact_force_n')), beta=args.beta)
    content = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(content)
    print(content, end='')


if __name__ == '__main__':
    main()
