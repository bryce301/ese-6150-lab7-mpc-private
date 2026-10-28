# Levine speed tuning without changing the path

The Levine launch now uses `levine_fast.csv`, with the same 1,250 position and
heading triples as `levine.csv`. Only the reference speed column differs.
No other team's trajectory was used. The original CSV and the default 6 m/s,
3 m/s² controller configuration remain available for a conservative run.
Spielberg now has its own optimized path CSV, using the same MPC settings;
see [SPIELBERG_TUNING.md](SPIELBERG_TUNING.md).

## Selected configuration

`ros2 launch mpc levine_launch.py` selects:

| Setting | Value |
|---|---:|
| Reference speed range | 3.450–11.000 m/s |
| MPC maximum speed | 11.0 m/s |
| MPC acceleration/deceleration input bound | 8.0 m/s² |
| Reference lateral acceleration ceiling | 10.0 m/s² |
| Reference forward acceleration ceiling | 5.5 m/s² |
| Reference braking ceiling | 6.5 m/s² |
| Reference geometric steering-rate ceiling | 1.5 rad/s |

The MPC input bound and reference-planning limits are separate. The simulator
accepts target speed and applies its own speed actuator, so the model's planned
acceleration is not a direct measurement of physical acceleration. Commands
still use measured speed plus the first optimized acceleration times the 0.1 s
model timestep. Local held-pose checks measured a 0.80 m/s command from rest,
below the documented 1.5 m/s startup check; steering correction checks passed.
These are local analogues, not the official autograder.

## Experiments

All tests used the actual ROS bridge, JAX single-track dynamics, LiDAR, and an
independent collision/lap observer, with a fresh simulator per run.

| Configuration | Best lap | Consecutive laps | Collision |
|---|---:|---:|---|
| Original | 14.08 s | 3 | No |
| Original speeds ×1.20, cap 7.2 m/s | 12.22 s | 3 | No |
| Original speeds ×1.40, cap 8.4 m/s | 10.94 s | 3 | No |
| Original speeds ×1.60, cap 9.6 m/s | No full lap | 0 | **Yes** |
| Replanned speeds, lateral 8.5, MPC acceleration bound 4 | 11.61 s | 3 | No |
| Previous profile: cap 10, lateral 9, MPC acceleration bound 8 | 10.32 s | 10 | No |
| Cap 10.5, lateral 9, reference accel/brake 4.5/5.5 | 10.11 s | 3 | No |
| Cap 11, lateral 9.5, reference accel/brake 5/6 | 9.84 s | 3 | No |
| Same profile, MPC acceleration bound increased to 9 | 9.73 s | 3 | No |
| Selected: cap 11, lateral 10, reference accel/brake 5.5/6.5 | 9.63 s | 10 | No |
| Selected, start y +0.10 m and yaw +0.05 rad | 9.63 s | 3 | No |
| Selected, start y −0.10 m and yaw −0.05 rad | 9.63 s | 3 | No |
| Selected through the installed final Levine launch | 9.63 s | 3 | No |

Uniform scaling increased corner speeds too much. The selected profile permits
higher straight-line speed while slowing before high-curvature segments.
Its three-lap test measured a maximum of 10.55 m/s and a minimum of 3.60 m/s
after lap 1. Sampled vehicle-footprint clearance was approximately 0.098 m,
less than the previous 10.32 s profile's 0.157 m. This is a more aggressive
simulation configuration. Clearance is estimated from recorded odometry and
map pixels, not a continuous safety guarantee.

The new ten-lap run recorded 9.65 s on its first flying lap and 9.63–9.64 s
thereafter, with zero collision messages. Both perturbed-start runs also
completed all three laps without a collision. This is 6.7% faster than the
previous 10.32 s profile. Local test artifacts and scripts live in the sibling
`../sim_ws/` workspace. Original summaries are in `artifacts/speed_tuning.json`;
new results are in `artifacts/speed_tuning_round2.json`.
Spielberg's unchanged configuration previously completed a clean lap in 58.27 s.
The Levine tuning above changed only its speed CSV and launch speed cap.
Subsequent Spielberg work uses the same MPC weights and settings, with a
different map-specific path CSV; see the separate Spielberg report.
The installed final launch independently reproduced 9.65, 9.64, and 9.63 s,
without collision. Geometry equality, speed-profile bounds including the seam,
and all five local held-pose check analogues passed with the final parameters.
These are local simulator times, not a result posted by the course autograder.

## Reproduce the speed CSV

From the repository root, in the simulation environment:

```bash
python mpc/scripts/generate_speed_profile.py \
  --input mpc/waypoints/levine.csv \
  --output mpc/waypoints/levine_fast.csv \
  --max-speed 11 --lateral-accel 10 --accel 5.5 --brake 6.5
```

The offline script reuses the existing periodic-spline curvature and
forward/backward speed-limit passes. It never optimizes or moves path points.
Rebuild the ROS package after regenerating the CSV.

To reproduce the previous, wider-margin 10.32 s profile, use
`--max-speed 10 --lateral-accel 9 --accel 4 --brake 5` instead and set the
Levine launch's `max_speed` back to `10.0`. Its exact CSV is also retained locally
at `../sim_ws/candidates/levine-v10-lat9-a4-b5.csv`.

For the original Levine behavior:

```bash
ros2 run mpc mpc_node.py --ros-args -p track:=levine \
  -p waypoint_file:=levine.csv -p max_speed:=6.0 -p max_accel:=3.0
```

Tested solver stack: CVXPY 1.6.5, OSQP 1.0.4, SciPy 1.15.3, NumPy 2.4.6.
The newer solver combination previously tested on this machine produced sparse
matrix update errors; see `../sim_ws/README_LOCAL.md`.

No real-car testing or course submission was performed. Clean local runs do not
guarantee collision-free behavior with different friction, localization error,
latency, or real hardware.
