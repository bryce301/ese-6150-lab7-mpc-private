#!/usr/bin/env python3
"""Generate closed-loop MPC references from f1tenth_gym_ros maps (offline)."""
import argparse
from pathlib import Path

import cvxpy as cp
import numpy as np
import yaml
from PIL import Image
from scipy.interpolate import CubicSpline
from scipy.ndimage import map_coordinates
from scipy.sparse import diags


def resample(points, spacing):
    if np.linalg.norm(points[-1] - points[0]) < 1e-6:
        points = points[:-1]
    closed = np.vstack((points, points[0]))
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    spline = CubicSpline(s, closed, bc_type='periodic')
    dense_s = np.linspace(0, s[-1], int(np.ceil(s[-1] / 0.005)) + 1)
    dense = spline(dense_s)
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    uniform_s = np.linspace(0, arc[-1], int(np.ceil(arc[-1] / spacing)), endpoint=False)
    return spline(np.interp(uniform_s, arc, dense_s))


def geometry(points):
    closed = np.vstack((points, points[0]))
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    spline = CubicSpline(s, closed, bc_type='periodic')
    d1, d2 = spline(s[:-1], 1), spline(s[:-1], 2)
    yaw = np.arctan2(d1[:, 1], d1[:, 0])
    curvature = (d1[:, 0]*d2[:, 1] - d1[:, 1]*d2[:, 0]) / np.linalg.norm(d1, axis=1)**3
    return s, yaw, curvature


def optimize_path(points, map_yaml, margin=0.65):
    center = resample(points, 0.20)
    s, yaw, _ = geometry(center)
    normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
    meta = yaml.safe_load(map_yaml.read_text())
    pixels = np.array(Image.open(map_yaml.parent / meta['image']).convert('L'))[::-1]
    free = ((1 - pixels/255) <= meta['occupied_thresh']).astype(float)
    offsets = np.arange(0, 4, 0.02)
    widths = []
    for sign in (-1, 1):
        samples = center[:, None, :] + sign * normal[:, None, :] * offsets[None, :, None]
        grid = (samples - np.array(meta['origin'][:2])) / meta['resolution']
        values = map_coordinates(free, grid.reshape(-1, 2)[:, ::-1].T,
                                 order=1, mode='constant', cval=0)
        widths.append(offsets[np.argmax(values.reshape(len(center), -1) < 0.5, axis=1)])
    n = len(center)
    D2 = diags([np.ones(n-1), -2*np.ones(n), np.ones(n-1), [1], [1]],
               [-1, 0, 1, n-1, 1-n], shape=(n, n), format='csc') / (s[-1]/n)**2
    offset = cp.Variable(n)
    line = center + cp.multiply(normal, cp.reshape(offset, (n, 1), order='C'))
    cost = cp.sum_squares(D2 @ line) + 0.01 * cp.sum_squares(offset)
    problem = cp.Problem(cp.Minimize(cost), [offset >= -widths[0] + margin,
                                           offset <= widths[1] - margin])
    problem.solve(solver=cp.OSQP, eps_abs=1e-7, eps_rel=1e-7, max_iter=50000)
    assert problem.status == cp.OPTIMAL, problem.status
    return resample(line.value, 0.05)


def speed_profile(points, vmax=6.0, lateral=6.5, accel=1.8, brake=2.0):
    s, yaw, curvature = geometry(points)
    ds = np.diff(s)
    steer = np.arctan(0.33 * curvature)
    dsteer = np.abs(np.roll(steer, -1) - steer) / ds
    v = np.minimum(vmax, np.sqrt(lateral / np.maximum(np.abs(curvature), 1e-8)))
    v = np.minimum(v, 1.5 / np.maximum(dsteer, 1e-8))
    # Include the closing segment when propagating acceleration/braking limits.
    for _ in range(3):
        for i in range(len(v)):
            j = (i + 1) % len(v)
            v[j] = min(v[j], np.sqrt(v[i]**2 + 2 * accel * ds[i]))
        for i in range(len(v)-1, -1, -1):
            j = (i + 1) % len(v)
            v[i] = min(v[i], np.sqrt(v[j]**2 + 2 * brake * ds[i]))
    return np.column_stack((points, yaw, v))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps', type=Path, required=True, help='f1tenth_gym_ros/maps')
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'waypoints')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for track, filename, delimiter, columns, map_name in (
        ('levine', 'levine_centerline.csv', ',', (0, 1), 'levine_blocked'),
        ('spielberg', 'Spielberg_raceline.csv', ';', (1, 2), 'Spielberg'),
    ):
        source = np.loadtxt(args.maps / filename, delimiter=delimiter)
        points = optimize_path(source[:, columns], args.maps / f'{map_name}.yaml')
        waypoints = speed_profile(points)
        np.savetxt(args.output / f'{track}.csv', waypoints, delimiter=',', fmt='%.8f')
        print(f'{track}: {len(points)} points, {geometry(points)[0][-1]:.2f} m, '
              f'{waypoints[:, 3].min():.2f}-{waypoints[:, 3].max():.2f} m/s')


if __name__ == '__main__':
    main()
