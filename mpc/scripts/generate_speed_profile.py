#!/usr/bin/env python3
"""Replan CSV speeds while preserving every position and heading exactly."""
import argparse
from pathlib import Path

import numpy as np
from generate_waypoints import speed_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-speed', type=float, default=10.0)
    parser.add_argument('--lateral-accel', type=float, default=8.5)
    parser.add_argument('--accel', type=float, default=4.0)
    parser.add_argument('--brake', type=float, default=4.0)
    args = parser.parse_args()
    limits = [args.max_speed, args.lateral_accel, args.accel, args.brake]
    if not all(np.isfinite(value) and value > 0 for value in limits):
        parser.error('all limits must be finite and positive')
    data = np.loadtxt(args.input, delimiter=',')
    if data.ndim != 2 or data.shape[1] != 4 or not np.isfinite(data).all():
        parser.error('input must be a finite four-column waypoint CSV')
    data[:, 3] = speed_profile(data[:, :2], vmax=args.max_speed,
                              lateral=args.lateral_accel, accel=args.accel,
                              brake=args.brake)[:, 3]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, data, delimiter=',', fmt='%.8f')
    print(f'{len(data)} unchanged poses; speed {data[:, 3].min():.2f}-{data[:, 3].max():.2f} m/s')


if __name__ == '__main__':
    main()
