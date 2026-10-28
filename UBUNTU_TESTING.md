# Personal Ubuntu simulation workspace

This private repository is a snapshot of the current Lab 7 project, including
the completed Python MPC and both generated waypoint CSVs. It is separate from
the team's Classroom repository. Pushing here is not a course submission.

Full local ROS simulation was run on 2026-10-27 using an isolated RoboStack
ROS 2 Jazzy / Python 3.12 Conda environment on Ubuntu 22.04. Both original
track configurations passed their required laps. Subsequent speed-only Levine
tuning is documented in [SPEED_TUNING.md](SPEED_TUNING.md). The earlier offline
and Windows reports in `artifacts/` are historical. The original assignment
instructions remain in `README.md`.

## Prepare Ubuntu

Use Ubuntu 24.04, ROS 2 Jazzy and Python 3.12. First install the simulator using
the [dev-jazzy simulator instructions](https://github.com/f1tenth/f1tenth_gym_ros/tree/dev-jazzy#installation),
including its `--system-site-packages` venv and the `dev-jax` gym. Use that same
workspace/venv for this package. The commands below assume `~/sim_ws`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/sim_ws/.venv/bin/activate
cd ~/sim_ws/src
git clone https://github.com/bryce301/ese-6150-lab7-mpc-private.git
cd ~/sim_ws
python -m pip install cvxpy osqp numba
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

Authenticate with your GitHub account when cloning this private repository.
The waypoint files are already generated and included; no regeneration is needed.

## Create test configurations without changing source files

After sourcing ROS, the venv and the workspace:

```bash
python - <<'PY'
import copy
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory

package = Path(get_package_share_directory('f1tenth_gym_ros'))
base = yaml.safe_load((package / 'config/sim.yaml').read_text())
for track, map_name, x, y, yaw in (
    ('levine', 'levine_blocked', -12.0, 0.0, 0.0),
    ('spielberg', 'Spielberg', 14.59, 3.92, -2.877),
):
    config = copy.deepcopy(base)
    config['bridge']['ros__parameters'].update(
        map_path=f'maps/{map_name}', sx=x, sy=y, stheta=yaw,
        num_agents=1, kb_teleop=False,
    )
    Path(f'/tmp/mpc-{track}.yaml').write_text(yaml.safe_dump(config))
PY
```

## Run each map separately

In each terminal, activate the same venv and source both ROS and
`~/sim_ws/install/setup.bash` before running commands.

Levine, simulator terminal:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py config:=/tmp/mpc-levine.yaml open_foxglove:=false
```

Levine, controller terminal:

```bash
ros2 launch mpc levine_launch.py
```

Check three consecutive completed laps with no collision. Stop both processes
before launching Spielberg, which needs a fresh simulator run:

```bash
# Simulator terminal
ros2 launch f1tenth_gym_ros gym_bridge_launch.py config:=/tmp/mpc-spielberg.yaml open_foxglove:=false
# Controller terminal
ros2 launch mpc spielberg_launch.py
```

Check one completed lap with no collision. An independent observer terminal may
monitor `/ego_racecar/lap_count` and `/ego_racecar/collision`; the MPC node does
not subscribe to these topics. Record any collision, including transient `true`
messages. Lap completion and times also appear in the simulator log.

For visualization, connect Foxglove to `ws://localhost:8765` and import the
simulator's `config/foxglove/gym_bridge_foxglove.json` layout.
