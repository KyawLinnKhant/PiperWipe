# Section 1 — Kinematics & Reachability

Stack: ROS 2 Humble · MoveIt 2 · Python 3.10

## What's here

```
section1/
├── README.md                    ← this file
├── writeup.md                   ← short answer: where can/can't the arm reach?
├── code/
│   ├── ik_solver.py             ← IKSolver library (surface-aligned wrapper around /compute_ik)
│   ├── ik_service.py            ← stand-alone ROS node — exposes /solve_ik (moveit_msgs/GetPositionIK)
│   ├── planning_scene.py        ← pushes countertop / mirror / faucet as CollisionObjects to MoveIt
│   └── reachability_heatmap.py  ← sweeps 60×60 cm patch @ 2 cm, writes CSV + PNG
└── outputs/
    ├── reachability.csv         ← per-cell (x, y, reachable, best_yaw_deg, reason)
    └── reachability_heatmap.png ← heatmap visualization
```

The arm + scene visual markers live in the **existing** `piper_wiping` package
(`kitchen_scene.py`, `kitchen_full_launch.py`, `rviz/kitchen.rviz`, URDF,
controller configs). Section 1 deliverables only add the IK / scene-collision /
heatmap pieces; the kitchen visualization is reused unchanged.

## Arm + scene configuration

- **Arm base** (`world` → `base_link`):
  `xyz = (0.00, 0.00, 0.05)`, `rpy = (0, 0, π/2)` — base on the front edge of
  the countertop, facing +Y toward the mirror. Set in
  `src/piper_ros/src/piper_description/urdf/piper_description.xacro` and
  mirrored by `kitchen_full_launch.py` (`BASE_XYZ`, `BASE_RPY`).
- **Group / tip link** (from `piper_with_gripper_moveit/config/piper.srdf`):
  group `arm`, base `base_link`, tip `link6`.
- **Tool offset:** the gripper finger origins are `0.1358 m` along link6's +Z.
  When the tool points down, the contact point is `link6.z - 0.1358`. The
  heatmap accounts for this when computing the IK target Z.
- **Scene collision objects** (published by `planning_scene.py`, coords match
  `kitchen_scene.py`):

  | Object        | Frame | Pose (m)              | Size / r,h (m)       |
  |---------------|-------|-----------------------|----------------------|
  | countertop    | world | (0.00, 0.25, 0.025)   | 1.20 × 0.60 × 0.05   |
  | mirror        | world | (0.00, 0.555, 0.500)  | 0.60 × 0.01 × 0.90   |
  | faucet_base   | world | (0.00, 0.50, 0.10)    | cyl r=0.015, h=0.10  |
  | faucet_spout  | world | (0.00, 0.475, 0.157)  | 0.015 × 0.05 × 0.015 |

## Build / run

```bash
# 0. one-time build
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select piper_wiping --symlink-install

# 1. bring up Piper + MoveIt 2 + RViz + kitchen markers
source ~/ros2_ws/install/setup.bash
ros2 launch piper_wiping kitchen_full_launch.py

# 2. publish kitchen collision objects to MoveIt's planning scene
python3 ~/PiperWipe/section1/code/planning_scene.py

# 3. (optional) start the standalone IK service
python3 ~/PiperWipe/section1/code/ik_service.py
#   → service available at /solve_ik (type: moveit_msgs/srv/GetPositionIK)

# 4. generate the reachability CSV + heatmap PNG
python3 ~/PiperWipe/section1/code/reachability_heatmap.py
# outputs land in section1/outputs/
```

## Design notes

- **Surface-aligned IK.** `IKSolver.solve(x, y, z, surface, tcp_yaw, tilt_rad)`
  composes the tool orientation from `surface`:
  - `countertop` → tool Z points down (world −Z), RPY = (π, 0, yaw).
  - `mirror`     → tool Z points into the mirror face (world +Y), RPY = (π/2, 0, yaw).
  `tcp_yaw` rotates the tool about its own approach axis (gripper spin), and
  `tilt_rad` tilts the tool off the surface normal by a few degrees about a
  local axis — the heatmap uses this to model "lean around the faucet" and
  "spin the gripper to clear the mirror" within the ±10° normal tolerance
  the assignment allows.
- **Why call `/compute_ik` instead of writing our own IK?** The Piper arm has
  a 6-DOF wrist; MoveIt ships a numerical solver (KDL by default; LMA is
  enabled in `kitchen_full_launch.py`) that already understands joint limits,
  scene collisions, and self-collisions. Reimplementing analytic IK would not
  add reachability information — it would only re-derive what MoveIt already
  reports.
- **No custom .srv files.** Section 1 uses `moveit_msgs/srv/GetPositionIK` for
  the `/solve_ik` service so the deliverable is a single set of pure-Python
  scripts that need no `colcon build` of message interfaces.
- **Collision-aware sweep.** The heatmap calls `/compute_ik` with
  `avoid_collisions=True` against the scene published by `planning_scene.py`,
  so cells where the gripper would intersect the countertop, mirror or faucet
  are rejected.

See `writeup.md` for the interpretation of the resulting heatmap.
