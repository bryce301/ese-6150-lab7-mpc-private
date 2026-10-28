# Spielberg: about 40 seconds with the same shared MPC

Both track launches use the same `mpc_node.py` and identical MPC settings.
Only `track` and `waypoint_file` differ. The attempted tracking-weight parameter
was removed; no alternative weight settings are used by either final launch.

| Shared MPC setting | Both tracks |
|---|---|
| State costs Q and Qf | diag(13.5, 13.5, 5.5, 13.0) |
| Input costs R and Rd | diag(0.01, 100.0) |
| Prediction | 8 steps × 0.1 s |
| Wheelbase | 0.33 m |
| Speed constraint | 0–11 m/s |
| Acceleration input constraint | ±8 m/s² |
| Steering constraint | ±0.4189 rad |

`../sim_ws/verify_shared_mpc.py` instantiates both final launches' configurations,
compares them, and checks Q/Qf/R/Rd against the original values. This passed.
The machine-readable verification is included in `artifacts/spielberg_tuning.json`.

## Path changes

Spielberg now loads `mpc/waypoints/spielberg_fast.csv`; the original
`spielberg.csv` is retained. This is a geometry-and-speed update, unlike the
earlier speed-only Levine change. The supplied upstream raceline and occupancy
map are used; no other team's path data is used.

- Map-normal planning margin: 1.0 m, increased from 0.65 m, to leave room for the
  unchanged MPC's tracking error through fast bends.
- Periodic spline, approximately 0.05 m sampling: 6,845 points, 342.22 m.
- Reference speeds: approximately 3.33–10.00 m/s.
- Reference lateral acceleration: at most 8.5 m/s².
- Reference forward acceleration / braking: 4.0 / 5.0 m/s².
- Reference geometric steering-rate ceiling: 1.5 rad/s.

Simply increasing speeds on the old geometry caused collisions at several bend
exits. A new path with 0.95 m planning margin completed a 39.77 s lap, but its
sampled footprint clearance was only about 0.030 m. The selected 1.0 m-margin
path completed about 40.1 s with approximately 0.125 m sampled clearance.
Planning margin and actual clearance are different quantities. Clearance is
estimated from recorded odometry and occupancy pixels, not continuously proven.

## Full ROS validation

All runs use the actual JAX physics, LiDAR, map server and ROS MPC, with the
documented start pose and an independent collision/lap observer.

| Run | Laps | Flying lap times | Collision |
|---|---:|---|---|
| Original geometry and speeds | 1 | 58.27 s | No |
| New path, first test | 1 | 40.11 s | No |
| New path, endurance | 3 consecutive | 40.11, 40.10, 40.10 s | No |
| Start y +0.10 m, yaw +0.05 rad | 1 | 40.10 s | No |
| Start y −0.10 m, yaw −0.05 rad | 1 | 40.10 s | No |
| Installed final Spielberg launch | 1 | 40.11 s | No |
| Levine regression, unchanged path/controller settings | 3 | 9.65, 9.64, 9.63 s | No |

The endurance run's maximum measured longitudinal speed was 10.032 m/s.
The screenshot supplied by the user reports Team 5 at 39.15 s, mean 8.22 m/s,
top 10.87 m/s, and Team 6 at 39.38 s, mean 8.15 m/s, top 10.00 m/s. These support
targeting sustained speed through a suitable path rather than a much higher
top speed. Local sampled averages may use a different definition from the
leaderboard and should not be treated as exactly comparable.

## Reproduction and submission

Build the package and launch it as usual:

```bash
ros2 launch mpc spielberg_launch.py
```

No path-generation script runs on the car or is required by the autograder.
Submit the one `mpc` package, shared controller/utilities, both launch files,
and their installed CSVs. Do not create separate MPC implementations per track.
The final launch's maximum speed and acceleration parameters match Levine's;
Spielberg's 10 m/s reference ceiling lives in its CSV.

The local offline reproduction helper (not a runtime dependency) is:

```bash
cd '/home/bryce/ESE 6510/Lab 7/sim_ws'
source env.sh
python make_spielberg_path.py \
  --output src/mpc/waypoints/spielberg_fast.csv \
  --margin 1.0 --max-speed 10 --lateral-accel 8.5 --accel 4 --brake 5
```

It reuses the repository's existing `optimize_path` and `speed_profile` functions.
Rebuild after regenerating data. The selected installed CSV was checked to match
the tested candidate exactly. Geometry/heading finiteness, geometric steering,
and periodic speed/acceleration constraints also passed.
Re-evaluating curvature from eight-decimal CSV coordinates gives a maximum
lateral acceleration of 8.500826 m/s², within the 0.001 m/s² quantization
tolerance around the 8.5 planning target. Peak geometric steering is 0.2483 rad.

Evidence: `artifacts/spielberg_tuning.json`, `artifacts/spielberg_comparison.png`,
and the sibling `../sim_ws/results/spielberg-*` logs/trajectory CSVs.
These are local ROS simulation results, not an official submission or real-car
validation. Different friction, noise, or delay can change collision behavior.
