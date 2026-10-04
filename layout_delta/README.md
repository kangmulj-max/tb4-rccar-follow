# layout_delta (0.1.0)

Repeated indoor capture on a **fixed reference map** and **round-to-round layout change reports**
(added / removed / moved) for factories and warehouses. ROS 2 Jazzy, ament_python, Apache-2.0.

```
round n:  localization gate → Nav2 waypoint patrol (stop-and-observe) → round map (occupancy)
compare:  round n vs n-1 → changes (kind, x, y, area) → report.md / changes.png / changes.json
```

## Install
```bash
cd ~/ws/src && git clone <this repo>
rosdep install --from-paths . -y --ignore-src
cd ~/ws && colcon build --packages-select layout_delta && colcon test --packages-select layout_delta
```

## Run one round (robot or simulation)
Prerequisites: localization (AMCL) and Nav2 on the **round-0 reference map**, same namespace.
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot1 map:=site01/round00/map.yaml
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot1
ros2 launch layout_delta round.launch.py namespace:=/robot1 ref_map:=site01/round00/map.yaml \
     waypoints:=config/waypoints_arena1004.yaml out_dir:=site01/round01 initial_pose:='[x, y, yaw_deg]'
ros2 run layout_delta detect --ref site01/round00/map.yaml --cur site01/round01/map.yaml --out site01/report01 \n     --roi-waypoints config/waypoints_arena1004.yaml
```

## Nodes
| Node | Subscribes | Provides | Key parameters |
|---|---|---|---|
| `scan_mapper` | `scan`, `odom`, TF `map → <scan frame>` | `~/save` (std_srvs/Trigger) → `map.yaml/pgm`, `map_stats.json`; saves again on shutdown | `ref_map`, `out_dir`, `max_range` 6.0, `only_when_still` true, `still_settle_s` 0.5, `max_tf_age` 0.3, `max_pose_std_m` 0.25 |
| `patrol` | `amcl_pose` | Nav2 goals; `mission.json` | `waypoints`, `initial_pose`, `localization_tol_m` 0.3, `localization_timeout_s` 60, `dwell_s` 4.0, `goal_timeout_s` 90, `retries` 1 |
| `detect` (CLI) | — | `report.md`, `changes.png`, `changes.json` (+ `eval.json` with `--truth`) | `--roi-waypoints` + `--roi-radius` 3.0 m, `--shift-margin` 0.15 m, `--min-area` 0.04 m² or `--min-extent` 0.45 m, `--wall-margin` 0.15 m |

## Behaviour that matters in the field
- **Localization gate**: the robot does not move until AMCL agrees with `initial_pose` within 0.3 m; otherwise the round is aborted and flagged (`mission.json: aborted`). Driving on a wrong pose makes recovery behaviours push the robot into walls.
- **Stop-and-observe**: only scans taken while the robot stands still (≥ 0.5 s) are integrated — moving scans carry pose error and timing skew.
- **Localization quality gate in the mapper**: scans are dropped while AMCL's position std exceeds 0.25 m, so a robot that got stuck and lost does not write a shifted copy of the room into the round map.
- **Misalignment is not change**: an added cell next to structure of the other round (≤ 0.15 m) is treated as a shifted wall, not an object. Objects touching walls are therefore missed (known limitation).
- **No change from unobserved cells**: a cell is compared only if both rounds observed it, so partial coverage does not create false "removed" objects.
- **Partial rounds are valid**: a failed waypoint is retried, then skipped and recorded.

## Known limitations (0.1.0)
- 2D lidar only: objects entirely below the lidar plane (TurtleBot 4: 0.193 m) are invisible. Depth layer planned (`layout_delta_depth`).
- Position tolerance for change matching is 0.5 m class; survey-grade accuracy (±30 mm) is out of scope without reference-point correction.
- Reference map must be fixed after round 0 (no re-mapping between rounds).
- Objects within ~0.15 m of walls or other persisting structure are not reported.
- A round whose route is blocked ends partial; Nav2 recovery near obstacles can still degrade AMCL (see simulation results).

## Tests
`colcon test` runs `test/test_diff.py` (synthetic grids: no change, added/removed/moved, unobserved cells, wall jitter, shifted wall, single-face line, ROI, evaluation rules, scan accumulation).
Simulation regression: `tb4_follow_sim` repo, `scripts/make_round_worlds.py` + session script (4 rounds on the real SLAM-map world with a ground-truth ledger).
