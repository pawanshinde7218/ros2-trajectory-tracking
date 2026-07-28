# trajectory_tracking

A from-scratch ROS 2 (Humble) pipeline for **waypoint collection → cubic-spline
path smoothing → constant-velocity time parameterization → pure-pursuit-style
trajectory tracking** on a differential-drive robot — **no Nav2 whatsoever**
(no Planner Server, Smac/NavFn, Controller Server, DWB/RPP/MPPI, BT Navigator,
recovery behaviors, or costmaps).

It assumes a working simulated robot already exists: URDF, Gazebo spawn,
`ros2_control` + diff-drive controller, `/odom`, `/cmd_vel`, and TF are all
in place. This package only implements the four algorithmic stages that turn
a handful of clicked points into smooth, tracked motion.

---

## 1. Project Overview

| Stage | Node | Input | Output | Algorithm |
|---|---|---|---|---|
| 1 | `waypoint_collector.py` | `/clicked_point` (RViz "Publish Point") | `/raw_waypoints`, `/waypoint_markers` | Collect exactly N (default 7) clicks |
| 2 | `path_smoother.py` | `/raw_waypoints` | `/smooth_path` | Parametric cubic spline over chord length |
| 3 | `trajectory_generator.py` | `/smooth_path` | `/trajectory` | Constant-velocity time parameterization |
| 4 | `trajectory_controller.py` | `/odom`, `/trajectory` | `/cmd_vel` | Time-indexed pure-pursuit-style controller |
| — | `rviz_visualizer.py` | `/odom` | `/robot_path` | Accumulates the actual travelled path (green) |

## 2. Architecture Diagram

```
                     RViz "Publish Point" tool
                              │
                              ▼
                    ┌───────────────────┐
                    │ waypoint_collector │  -> /waypoint_markers (red spheres)
                    └─────────┬─────────┘
                              │ /raw_waypoints (7 pts, nav_msgs/Path)
                              ▼
                    ┌───────────────────┐
                    │   path_smoother    │  cubic spline, 5 cm sampling
                    └─────────┬─────────┘
                              │ /smooth_path (nav_msgs/Path, blue in RViz)
                              ▼
                    ┌───────────────────┐
                    │ trajectory_generator│ constant-velocity time parameterization
                    └─────────┬─────────┘
                              │ /trajectory (trajectory_tracking/Trajectory)
                              ▼
                    ┌───────────────────┐
        /odom ────▶ │ trajectory_controller │──▶ /cmd_vel ──▶ diff-drive robot
                    └───────────────────┘
                              ▲
                              │ /odom
                    ┌───────────────────┐
                    │  rviz_visualizer   │──▶ /robot_path (green, actual path driven)
                    └───────────────────┘
```

## 3. ROS Graph / Topics

| Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/clicked_point` | `geometry_msgs/PointStamped` | RViz | `waypoint_collector` |
| `/raw_waypoints` | `nav_msgs/Path` | `waypoint_collector` | `path_smoother` |
| `/waypoint_markers` | `visualization_msgs/MarkerArray` | `waypoint_collector` | RViz |
| `/smooth_path` | `nav_msgs/Path` | `path_smoother` | `trajectory_generator`, RViz |
| `/trajectory` | `trajectory_tracking/Trajectory` | `trajectory_generator` | `trajectory_controller` |
| `/odom` | `nav_msgs/Odometry` | robot stack | `trajectory_controller`, `rviz_visualizer` |
| `/cmd_vel` | `geometry_msgs/Twist` | `trajectory_controller` | robot stack |
| `/robot_path` | `nav_msgs/Path` | `rviz_visualizer` | RViz |

Custom messages (`msg/`):

```
TrajectoryPoint.msg:
  float64 x
  float64 y
  float64 theta
  float64 velocity
  float64 time_from_start

Trajectory.msg:
  std_msgs/Header header
  TrajectoryPoint[] points
```

## 4. Algorithms & Math

### 4.1 Path Smoothing — Parametric Cubic Spline

Given waypoints `P0..Pn`, chord length is accumulated as
`s_0 = 0`, `s_i = s_{i-1} + ||P_i - P_{i-1}||`.

Two independent natural cubic splines are fit:

```
x(s) = CubicSpline(s_i, x_i),  bc_type = "natural"  (zero curvature at ends)
y(s) = CubicSpline(s_i, y_i)
```

The spline is then evaluated at `s = 0, Δs, 2Δs, ..., s_n` with `Δs = 0.05 m`
(parameter `sample_spacing`), producing a dense `nav_msgs/Path`.

### 4.2 Trajectory Generation — Constant Velocity Time Parameterization

For each smoothed point `i` with cumulative distance `d_i`:

```
t_i = d_i / v_cruise           (v_cruise = 0.5 m/s by default)
θ_i = atan2(y_{i+1} - y_i, x_{i+1} - x_i)     (heading from look-ahead point)
```

The final point's commanded velocity is forced to `0` so the controller can
detect the stopping condition.

### 4.3 Trajectory Tracking — Time-Indexed Pure Pursuit

At each control tick (20 Hz), elapsed time `Δt = now - t_start` is used to
find the two trajectory points bracketing `Δt` and linearly interpolate a
reference `(x_ref, y_ref, θ_ref, v_ref)` — **the reference is chosen by time,
never by nearest-waypoint search**, as required.

Errors relative to the current pose `(x, y, θ)`:

```
Δx = x_ref - x,  Δy = y_ref - y
distance_error = sqrt(Δx² + Δy²)
heading_error  = normalize(atan2(Δy, Δx) - θ)      (wrapped to (-π, π])
```

Control law (pure-pursuit curvature form, with the reference speed in place
of a fixed lookahead distance):

```
curvature = 2 · sin(heading_error) / max(distance_error, ε)
v = clamp((v_ref + k_d · distance_error) · cos(heading_error), 0, v_max)
ω = clamp(curvature · v, -ω_max, ω_max)
```

The `cos(heading_error)` taper slows the robot (and lets `curvature` dominate)
when it is pointed far away from the reference, avoiding overshoot on sharp
turns. The robot stops once elapsed time has passed the trajectory's total
duration **and** it is within `goal_position_tolerance` of the final point.

## 5. Package Layout

```
trajectory_tracking/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── trajectory_tracking.launch.py
├── config/
│   └── trajectory_tracking.rviz
├── msg/
│   ├── TrajectoryPoint.msg
│   └── Trajectory.msg
├── test/
│   └── test_utils.py
└── trajectory_tracking/
    ├── __init__.py
    ├── utils.py
    ├── waypoint_collector.py
    ├── path_smoother.py
    ├── trajectory_generator.py
    ├── trajectory_controller.py
    └── rviz_visualizer.py
```

## 6. Build & Run

```bash
# from the root of your ROS2 workspace, e.g. ~/ros2_ws
cp -r trajectory_tracking src/
colcon build --packages-select trajectory_tracking
source install/setup.bash
```

Terminal 1 — bring up your existing robot simulation (Gazebo + robot + TF),
exactly as you already do today (this package does not touch that stack):

```bash
ros2 launch <your_robot_bringup_package> <your_sim_launch_file>
```

Terminal 2 — start RViz2 with the provided configuration:

```bash
rviz2 -d install/trajectory_tracking/share/trajectory_tracking/config/trajectory_tracking.rviz
```

Terminal 3 — start the pipeline:

```bash
ros2 launch trajectory_tracking trajectory_tracking.launch.py
```

In RViz, select the **Publish Point** tool and click 7 points in the map.
After the 7th click, smoothing, trajectory generation, and tracking start
automatically — no service call, button, or keyboard input required.

### Useful launch arguments

```bash
ros2 launch trajectory_tracking trajectory_tracking.launch.py \
  num_waypoints:=7 sample_spacing:=0.05 cruise_velocity:=0.5 control_frequency:=20.0
```

## 7. Testing

`test/test_utils.py` unit-tests the pure geometry/math helpers
(angle normalization, distance, heading, cumulative arc length, quaternion
round-trips, interpolation) independently of any ROS2 runtime:

```bash
python3 -m pytest test/test_utils.py -v
```

Node-level integration testing (spinning up each node and checking published
topics with `ros2 topic echo` / a `launch_testing` harness) is a natural next
step and is called out under Future Work.

## 8. AI Tools Used

This package (nodes, message definitions, launch file, RViz config, unit
tests, and this README) was generated with AI assistance (Claude) based on
the assignment specification, then reviewed for correctness against the
constant-velocity time-parameterization and cubic-spline path-smoothing math
worked through by hand.

## 9. Extending to a Real Robot

- Replace the assumed `/odom` source with a fused estimate (wheel odometry +
  IMU, or an EKF via `robot_localization`) since raw wheel odometry drifts
  significantly over the duration of a real trajectory.
- Add acceleration/jerk limits to the trajectory generator (trapezoidal or
  S-curve velocity profile) — real motors cannot instantaneously reach
  `v_cruise`.
- Tune `distance_gain`, `max_linear_velocity`, and `max_angular_velocity` to
  the physical robot's actual dynamics and safety limits; add a watchdog
  that zeroes `/cmd_vel` if `/odom` goes stale.
- Add a low-pass filter or slew-rate limiter on the published `Twist` to
  protect gearboxes/motor drivers from step commands.

## 10. Extra Credit — Obstacle Handling (Future Work)

Only a **safe stop** is in scope here — no replanning, no Nav2 recovery
behaviors, no local planner:

- A `LaserScan` (or `PointCloud2`) subscriber would check for obstacles
  inside a forward safety corridor sized to the robot's footprint plus
  current speed's stopping distance.
- On detection, the controller publishes a zero `Twist` and latches a
  "blocked" state until the obstacle clears, then resumes tracking from the
  current elapsed time (not from the last waypoint, to avoid a discontinuous
  jump).
- True obstacle avoidance (local replanning around the obstacle) is
  explicitly **out of scope** and left as future work.

## 11. Future Work

- Unlimited waypoint collection (not fixed to 7)
- Service-based or GUI-based trigger to start smoothing, instead of a fixed count
- Optional Nav2 integration for global planning while keeping this custom local pipeline
- Dynamic obstacle avoidance (beyond the safe-stop behavior above)
- Trajectory replanning on execution deviation
- Trapezoidal / S-curve velocity profiles instead of constant velocity
- Model Predictive Control (MPC) tracking controller as an alternative to pure pursuit
- Node-level integration tests using `launch_testing`

## 12. Screenshots

_Add RViz screenshots/GIFs here showing: (1) the 7 red waypoint markers,
(2) the blue smoothed spline, and (3) the green travelled path overlapping
the blue path as the robot completes the run._
