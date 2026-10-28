# MPC reference paths

The node defaults to `<track>.csv`; the `waypoint_file` parameter can select
another installed CSV. The launches now select `levine_fast.csv` and
`spielberg_fast.csv`. The original CSVs remain available. Each file has four comma-separated
columns, without a header:

```text
x [m], y [m], yaw [rad], reference speed [m/s]
```

Positions describe the rear axle (`base_link`), matching the controller and
the simulator bridge's odometry. Both paths follow the supplied track direction;
Levine is counter-clockwise. The final point connects to the first without a
duplicate endpoint. Spacing is approximately 0.05 m, including that segment,
as required by the template's reference interpolation.

| Track | Points | Length | Reference speed | Peak geometric steering |
| --- | ---: | ---: | ---: | ---: |
| Levine | 1,250 | 62.45 m | 2.78-6.00 m/s | 0.271 rad |
| Levine fast (same poses) | 1,250 | 62.45 m | 3.45-11.00 m/s | 0.271 rad |
| Spielberg | 6,800 | 339.96 m | 3.45-6.00 m/s | 0.179 rad |
| Spielberg fast (revised path) | 6,845 | 342.22 m | 3.33-10.00 m/s | 0.248 rad |

## Original path and speed construction

Levine starts from the supplied centerline. Spielberg starts from the supplied
raceline, whose geometry is smoother than its centerline at the tight corners.
Neither source is used unchanged: local track widths are measured from the
actual occupancy maps, and a quadratic optimization adjusts lateral offsets
to minimize squared second spatial differences, with a small offset penalty.
A 0.65 m normal-direction wall margin leaves room for the vehicle footprint
and tracking error. It is a planning margin, not a guaranteed driven clearance.

Periodic cubic splines and arc-length resampling produce a smooth closed path.
Yaw is computed from its tangent. The speed ceiling combines:

- maximum speed: 6.0 m/s;
- curvature limit: `v <= sqrt(6.5 / abs(curvature))`, using 6.5 m/s² lateral acceleration;
- geometric steering-rate limit: 1.5 rad/s, below the controller's limit;
- forward acceleration: 1.8 m/s²;
- braking: 2.0 m/s².

Forward and backward passes enforce acceleration and braking limits across the
entire loop, including its seam. The car slows before the corner rather than
waiting until it reaches the maximum curvature. The node computes commanded
speed from measured speed and the first MPC acceleration; it does not directly
command the CSV speed at startup.

## Reproduce

Inputs are from
[`f1tenth/f1tenth_gym_ros`, commit `2c019810b4098ed82d44cad1d7d282a2dfc12421`](https://github.com/f1tenth/f1tenth_gym_ros/tree/2c019810b4098ed82d44cad1d7d282a2dfc12421/maps):

- `maps/levine_centerline.csv`, `maps/levine_blocked.yaml`, `maps/levine_blocked.png`;
- `maps/Spielberg_raceline.csv`, `maps/Spielberg.yaml`, `maps/Spielberg.png`.

Source datasets: Copyright (c) 2020 Hongrui Zheng, under the
[MIT license](../../LICENSE).

From the package's repository root, after checking out those simulator inputs:

```bash
python mpc/scripts/generate_waypoints.py --maps /path/to/f1tenth_gym_ros/maps
```

The offline generator uses NumPy, SciPy, CVXPY/OSQP, Pillow and PyYAML. It is not
run by the ROS node or the launch files; the generated CSVs are installed by
the existing `CMakeLists.txt`.

## Validation and limits

The offline results below apply to the original paths/speeds. Full ROS tests
and the new speed-only Levine profile are documented in
[`SPEED_TUNING.md`](../../SPEED_TUNING.md).
The revised Spielberg geometry and full ROS results are documented in
[`SPIELBERG_TUNING.md`](../../SPIELBERG_TUNING.md).

The saved CSVs were checked for uniform spacing, valid headings, closure,
steering feasibility, and lateral/longitudinal acceleration limits. Actual
MPC/reference functions were then exercised against the upstream single-track
dynamics, speed/steering actuators, and RK4 integration at 0.01 s, using the
stock starting poses and occupancy-map vehicle-footprint checks. Upstream
dynamics source: `f1tenth/f1tenth_gym`, commit
`ec3b1a4693ed20317e42e57a9454d94b824cec81`.

Seven runs per track checked control periods of 0.01, 0.05 and 0.10 s, reduced
friction (`mu=0.85`), 40 ms command delay, position/yaw noise (standard deviations
0.02 m / 0.01 rad), and their combination. Each Levine run covered three flying
laps after the initial finish-line crossing; each Spielberg run covered one.
All 14 runs finished without a footprint intersecting occupied map cells.
Minimum estimated footprint-to-wall clearances across these runs were 0.235 m
for Levine and 0.160 m for Spielberg.

These are offline model results, not a complete ROS/JAX simulator or real-car
validation. They do not establish a globally optimal racing line or lap time;
the supplied launch files can be used for the remaining simulator validation.
