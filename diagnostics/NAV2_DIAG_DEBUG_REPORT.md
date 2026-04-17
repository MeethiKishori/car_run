# Nav2 + Gazebo + RViz Debug Report

Date: 2026-04-16
Workspace: car_run

## Update Policy

This report is the single source of truth for debugging status.

For every new issue cycle, update this file with:

1. Latest finding (what failed)
2. Root cause evidence (log-based)
3. Solution applied (what changed)
4. Validation result (what passed/what still fails)
5. Append updates immediately after every code/config/script change in the same work session.

Keep explanations short and practical.

## File Removal Clarification

No whole project files were removed.

What was removed were redundant items inside existing files:

1. Redundant package installs in `f1tenth_gym_ros/Dockerfile`
2. Redundant repeated manual steps in `dev.sh`

In short:

1. No repo source file was deleted.
2. Existing files were cleaned and simplified.

## Change Log (What and How)

This section records every important code/config change made during debugging.

### 1. Nav2 BT plugin fix

What changed:

1. Added an explicit Foxy-compatible `plugin_lib_names` list in `my_robot/config/nav2/nav2_params.yaml`.

How:

1. Compared runtime missing-library errors with actual installed libraries in `/opt/ros/foxy/lib`.
2. Kept only plugin names compatible with the Foxy environment.

Why:

1. `bt_navigator` failed during startup because it tried to load unavailable BT plugin libraries.

### 2. BT XML path fix

What changed:

1. Replaced relative BT XML lookup with an absolute path in `my_robot/config/nav2/nav2_params.yaml`.
2. Added launch-time BT XML override in `my_robot/launch/diag_launch.py`.

How:

1. Verified available BT XML files in `/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees`.
2. Pointed configuration directly to a known existing XML file.

Why:

1. `bt_navigator` failed when the XML filename was resolved only relatively.

### 3. Removed duplicate static TF ownership

What changed:

1. Removed redundant static TF publishers from `my_robot/launch/diag_launch.py`.

How:

1. Kept `robot_state_publisher` as the main source of URDF-derived TF.
2. Removed extra publishers for links already described by the robot model.

Why:

1. Duplicate TF publishers can confuse consumers like SLAM and Nav2.

### 4. Nav2 controller stabilization

What changed:

1. Replaced custom regulated-pure-pursuit controller tuning with Foxy-stable DWB settings in `my_robot/config/nav2/nav2_params.yaml`.

How:

1. Compared custom controller config against Foxy reference params.
2. Switched to a known stable controller profile.

Why:

1. Navigation goals were aborting or crashing mid-execution.

### 5. Simpler BT XML for repeated-goal stability

What changed:

1. Switched BT XML to `navigate_w_replanning_distance.xml`.

How:

1. Compared available Foxy behavior trees.
2. Selected a simpler replanning tree with fewer recovery interactions.

Why:

1. Repeated goals were unstable with the earlier recovery-oriented tree.

### 6. Explicit bt_navigator frame parameters

What changed:

1. Added explicit `robot_base_frame`, `global_frame`, and `odom_topic` for `bt_navigator`.
2. Added explicit parameters for `bt_navigator_rclcpp_node`.

How:

1. Used log evidence showing unexpected fallback to `base_link`.
2. Forced runtime parameters to match the robot namespace and frame tree.

Why:

1. Navigation was failing because bt_navigator pose lookup did not match the actual robot frames.

### 7. Added TF alias for base_link compatibility

What changed:

1. Added static transform alias from `car_1_base_link` to `base_link` in `my_robot/launch/diag_launch.py`.

How:

1. Added a `static_transform_publisher` node for zero-offset aliasing.

Why:

1. Foxy navigation internals still attempted `base_link` lookups even when runtime parameters were corrected.

### 8. dev.sh launch workflow update

What changed:

1. Added `launch` command to `dev.sh`.
2. Later updated `launch` to do cleanup + build + source + launch in one flow.

How:

1. Added container-side sourcing of:
   - `/opt/ros/foxy/setup.bash`
   - `/sim_ws/install/setup.bash`
   - `/usr/share/gazebo/setup.bash`
2. Added cleanup for stale Nav2, SLAM, Gazebo, RViz, and old launch processes.
3. Added package rebuild before launch.

Why:

1. User should not need to manually source or rebuild before each launch.
2. Duplicate nodes were being caused by old launch trees still running.

### 9. Dockerfile cleanup

What changed:

1. No Dockerfile file was removed.
2. Redundant package installs were removed from `f1tenth_gym_ros/Dockerfile`.
3. Clone/install steps were combined into fewer layers.
4. `dist-upgrade` was removed.

How:

1. Removed explicit redundant installs:
   - `ros-foxy-nav2-behavior-tree`
   - `ros-foxy-nav2-bt-navigator`
2. Kept broader packages already providing the needed Nav2 stack:
   - `ros-foxy-navigation2`
   - `ros-foxy-nav2-bringup`
3. Combined `git clone` + `pip install -e .` for `f1tenth_gym` into one layer.

Why:

1. This reduces duplication, image size, and maintenance overhead.

1. Running `./dev.sh launch` after previous attempts could still leave multiple ROS2/Nav2 nodes.
2. User needed launch to include build so manual source/build flow is not required each time.
3. Duplicate-node state caused unstable navigation behavior and intermittent errors.
4. User wants `./dev.sh launch` to keep full launch output visible in the same terminal, not hide it behind background execution.
5. `./dev.sh launch` could exit immediately after cleanup and return to host prompt before build/launch logs.
6. Even after earlier guard changes, `set -e` could still exit on the `docker exec` cleanup call itself.
7. Nav2 was running, but runtime logs still showed frequent warnings:
   - `Control loop missed its desired rate of 20.0000Hz`
   - intermittent `send_goal failed` aborts
   - repeated `Message Filter dropping message` from RViz/SLAM
8. User asked whether cleanup actually verifies duplicate critical nodes, and whether container restart is always required.
9. In latest failing run, duplicate-node health was OK, but `/navigate_to_pose` had zero action servers after startup because `bt_navigator` dropped while planner/controller stayed alive.
10. Runtime process table showed controller/planner consuming very high CPU while `bt_navigator` was absent; this correlated with `send_goal failed` and control-loop miss warnings.
11. New run showed planner-level failure on some goals:
   - `Failed to create a plan from potential when a legal potential was found`
   - followed by `send_goal failed`
   - then intermittent `bt_navigator` crash (`exit code -11`) on subsequent goal.
12. RViz also showed intermittent TF/OpenGL warnings (empty frame warnings, GLSL sampler error), which can add UI noise but are separate from planner failure.

### Solutions applied (latest cycle)

1. Updated `dev.sh` `launch` command to do all steps in one run:
   - cleanup stale ROS/Gazebo/Nav2 processes
   - build required packages
   - source workspace
   - launch stack in the same visible terminal
2. Added process cleanup in `launch` for:
   - old `ros2 launch my_robot diag_launch.py`
   - Nav2 nodes
   - SLAM node
   - spawn_entity
   - RViz/Gazebo processes
3. Cleaned `f1tenth_gym_ros/Dockerfile` by removing redundant package installs and simplifying layers.
4. Kept launch handling simple so the user can directly watch sourcing, build output, and live ROS launch logs.
5. Hardened `cleanup_runtime()` in `dev.sh` to always return success so non-critical cleanup exit codes do not abort launch under `set -e`.
6. Added `|| true` to the cleanup `docker exec` command so `set -e` cannot terminate launch on cleanup return codes.
7. Applied performance/timing tuning in Nav2 and SLAM configs to reduce compute spikes and TF timing sensitivity:
   - lowered `controller_frequency` from 20.0 to 10.0
   - reduced DWB sample counts and horizon (`vx/vy/vtheta samples`, `sim_time`)
   - increased DWB `transform_tolerance`
   - lowered planner expected frequency
   - lowered recoveries and waypoint loop rates
   - lowered local/global costmap update/publish rates
   - relaxed SLAM transform timing (`transform_publish_period`, `transform_timeout`, `tf_buffer_duration`)
8. Added explicit duplicate-node checking in `dev.sh`:
   - launch now checks for duplicate critical nodes immediately after cleanup
   - added `./dev.sh health` command for manual duplicate-node verification
   - launch exits with restart guidance if duplicates still exist
9. Reduced local planner computational load by switching controller plugin from DWB to Regulated Pure Pursuit with conservative speed/tolerance settings.
10. Lowered controller/planner pressure further (`controller_frequency` and `expected_planner_frequency`) and increased `bt_navigator default_server_timeout`.
11. Increased Navfn planner robustness for partial/edge map goals:
   - `GridBased.tolerance`: `0.5 -> 1.0`
   - `GridBased.use_astar`: `false -> true`
12. Added `bt_navigator` auto-respawn in launch:
   - `respawn=True`
   - `respawn_delay=2.0`
9. Added automatic post-start health check inside `./dev.sh launch`:
   - after launch starts, a delayed check runs automatically
   - prints `HEALTH: OK` or duplicate-node details in the same launch terminal

### Why this helps

1. Prevents multiple active launch trees (main source of duplicate nodes).
2. Ensures launch always uses freshly built packages.
3. Reduces manual steps and lowers chance of environment drift.
4. Preserves direct visibility into the launch process, which is more useful for debugging runtime failures.
5. Prevents early script exit right after "Cleaning stale ROS/Gazebo processes".
6. Makes cleanup step robust even when process-kill commands return non-zero exit statuses.
7. Reduces CPU pressure and frame-timing jitter, which helps prevent control loop deadline misses and intermittent action aborts.
8. Clarifies restart policy: restart is not required every run; use restart only if duplicate critical nodes survive cleanup.
9. Distinguishes two failure classes clearly:
   - duplicate nodes (cleanup/restart issue)
   - navigator action-server dropout under load (controller/planner tuning issue)
10. Lower compute load and higher time tolerance should reduce control-loop misses and intermittent `send_goal failed` aborts.
11. Planner tolerance + A* improves path generation success near uncertain map boundaries.
12. Auto-respawn prevents complete loss of `/navigate_to_pose` server after rare Foxy `bt_navigator` crashes.
9. Gives immediate runtime visibility into node-health state without requiring a separate manual health command.

### New findings

1. Nav2 goals were accepted but then aborted/stuck on repeated commands.
2. Logs showed bt_navigator trying to resolve `base_link`, while the robot TF tree uses `car_1_base_link`.
3. This mismatch triggered pose lookup failure and intermittent abort/crash behavior.

### Evidence from logs

1. `Invalid frame ID "base_link"`
2. `No Transform available ... source_frame does not exist`
3. `Navigation failed` during `/navigate_to_pose`

### Solutions applied

1. Updated Nav2 controller settings to a Foxy-stable DWB configuration in `my_robot/config/nav2/nav2_params.yaml`.
2. Switched BT XML to a simpler replanning tree:
   - `navigate_w_replanning_distance.xml`
3. Added explicit frame params for:
   - `bt_navigator`
   - `bt_navigator_rclcpp_node`
4. Added static TF alias in launch:
   - `car_1_base_link -> base_link`
   so legacy/base_link lookups do not fail.

### Validation status

1. Lifecycle nodes still come up.
2. Frame mismatch error was removed from previous failing path.
3. Navigation behavior improved, but goal handling can still be inconsistent depending on startup timing and RViz/OpenGL runtime instability.
4. Recommended practice: wait for `bt_navigator` to be active before sending the first goal.

## What was failing at the beginning

The startup flow had multiple failures across build and runtime:

1. Host shell launch failed with:
   - ros2: command not found
   Reason: ROS was not sourced on the macOS host shell.

2. Nav2 bringup failed in bt_navigator with:
   - Could not load library: libnav2_change_goal_node_bt_node.so
   Reason: Plugin mismatch between config and available Foxy BT plugins.

3. bt_navigator also failed to load behavior tree file with:
   - Couldn't open input XML file: navigate_w_replanning_and_recovery.xml
   Reason: Relative XML path was not reliably resolved in runtime context.

4. Lifecycle manager failed with:
   - Failed to bring up all requested nodes
   Reason: bt_navigator failed during configure, which blocked full Nav2 activation.

5. Runtime was unstable due to overlapping launch sessions:

## Latest Operator Update (2026-04-16, Evening)

### What was added/changed

1. Map saving now works directly to project maps folder:
   - `./dev.sh save_map <name>`
   - saved files: `my_robot/maps/<name>.pgm` and `my_robot/maps/<name>.yaml`
2. Added manual headless mode command:
   - `./dev.sh headless`
   - this closes only `gzclient` and keeps `gzserver` running
3. Kept GUI control manual by request:
   - `cleanup_runtime()` no longer kills `gzclient` automatically
   - user decides when to close GUI
4. `cancel_goal` behavior validated:
   - `return_code=0` with `goals_canceling=[]` means cancel succeeded but no active goal existed at call time

### Why users still see aborts without sending any goal

Observed lifecycle state sometimes becomes partial at startup:

1. `/controller_server -> unavailable` or `inactive`
2. `/planner_server -> unconfigured`
3. `/recoveries_server -> unconfigured`
4. `/bt_navigator -> active` or `unconfigured` depending on timing

This can happen before any goal click when CPU is overloaded and lifecycle transitions do not complete cleanly.

### Evidence pattern (latest)

1. CPU often > 700% in container snapshots
2. Top load from `gzserver`, `gzclient`, `rviz2`
3. Goal attempt then fails with:
   - `Send goal call failed`
   - `Action server failed while executing action callback: "send_goal failed"`
   - `[navigate_to_pose] ... Aborting handle`

### Recommended operation (first goal and next goal)

Use this sequence every run.

1. Launch stack:
   - `./dev.sh launch`
2. Optional lower-load mode:
   - `./dev.sh headless`
3. Wait 20-30 seconds.
4. Run readiness check:
   - `./dev.sh perf`
5. Send first goal only if all lifecycle nodes are `active [3]`:
   - `/controller_server`
   - `/planner_server`
   - `/recoveries_server`
   - `/bt_navigator`
   - `/waypoint_follower`

For next goals:

1. Send only one new goal at a time.
2. Wait for result state before sending next goal.
3. If navigation looks stuck:
   - run `./dev.sh cancel_goal`
   - wait 2-3s
   - send one fresh goal
4. If repeated failure (2-3 times) or any lifecycle node is not active:
   - `./dev.sh stop`
   - `docker restart f1tenth_gym_ros-sim-1`
   - `./dev.sh launch`

### Practical interpretation notes

1. `goals_canceling=[]` after cancel call is valid when goal already ended or no active goal exists.
2. `Message Filter dropping message ... reason 'Unknown'` is often timing/TF pressure under load and is not by itself proof of Nav2 hard failure.
3. Partial lifecycle state is a startup/runtime stability issue, not a user goal-click mistake.

### Commands summary (current)

1. Bringup: `./dev.sh launch`
2. Health check: `./dev.sh health`
3. Perf + lifecycle + recommendation: `./dev.sh perf`
4. Cancel active nav goals: `./dev.sh cancel_goal`
5. Headless mode (manual): `./dev.sh headless`
6. Save map: `./dev.sh save_map <map_name>`
7. Capture diagnostics snapshot: `./dev.sh capture`

## Latest Update (2026-04-16, Late Evening)

### What changed

1. Added `goal_status` command to `dev.sh`.
2. This command runs inside the container and prints:
   - `ros2 action info /navigate_to_pose`
   - lifecycle state of core Nav2 nodes

### Why it was needed

1. Running `ros2 action info /navigate_to_pose` directly on host returned `127` (`ros2` not found) because ROS is inside container runtime.
2. Users needed a one-command check to know if previous goal ended and whether it is safe to send next goal.

### How to use

1. `./dev.sh goal_status`
2. Send next goal only when:
   - action server `/bt_navigator` is present
   - all lifecycle nodes are `active [3]`

### Updated quick decision rule

1. If `goal_status` shows partial lifecycle (`inactive`, `unconfigured`, `unavailable`): do not send goal.
2. If repeated issues continue:
   - `./dev.sh stop`
   - `docker restart f1tenth_gym_ros-sim-1`
   - `./dev.sh launch`

## Latest Update (2026-04-16, Night: Red-Path-No-Motion Case)

### New finding

1. In some runs, RViz shows a valid global path (red line) and `bt_navigator` starts navigation, but the car does not move.
2. This occasionally ends with `Failed to make progress` and action abort.

### Log evidence pattern

1. `Begin navigating from current location ...`
2. intermittent `Message Filter dropping message ... reason 'Unknown'`
3. `Control loop missed its desired rate ...`
4. then `Failed to make progress` -> `Navigation failed`

### Root cause interpretation

1. Planner is producing a path, but controller/progress checks are too strict for current timing/load jitter.
2. Under high CPU and TF timing pressure, the progress checker can abort before enough movement is observed.

### Solution applied

1. Tuned `progress_checker` in `my_robot/config/nav2/nav2_params.yaml`:
   - `required_movement_radius: 0.5 -> 0.2`
   - `movement_time_allowance: 10.0 -> 20.0`
2. Relaxed goal checker tolerances:
   - `xy_goal_tolerance: 0.25 -> 0.30`
   - `yaw_goal_tolerance: 0.25 -> 0.30`
3. Reduced RPP aggressiveness and increased TF tolerance:
   - `desired_linear_vel: 0.35 -> 0.28`
   - `lookahead_dist: 0.6 -> 0.5`
   - `min_lookahead_dist: 0.3 -> 0.25`
   - `transform_tolerance: 0.5 -> 0.8`
4. Added costmap transform tolerance for timing robustness:
   - local costmap `transform_tolerance: 0.8`
   - global costmap `transform_tolerance: 0.8`

### How to operate after this change

1. Relaunch to load new params:
   - `./dev.sh launch`
2. Optional lower load:
   - `./dev.sh headless`
3. Before first goal, verify:
   - `./dev.sh goal_status`
   - all core nodes `active [3]`
4. If red path appears but car pauses:
   - wait a few seconds once
   - if still stuck: `./dev.sh cancel_goal`, then send one fresh nearby goal
   - Duplicate Nav2 and SLAM nodes in graph
   - Invalid sequence number errors in lifecycle manager
   Reason: Multiple launch processes remained alive at the same time.

6. GUI-side errors appeared:
   - gzclient assertion failure
   - RViz OpenGL GLSL texture-unit error
   These were environment/display issues and not the root cause of Nav2 lifecycle failure.

## What was checked

The following checks were done in the container runtime:

1. Verified available Nav2 BT libraries in /opt/ros/foxy/lib
2. Verified available BT XML files in /opt/ros/foxy/share/nav2_bt_navigator/behavior_trees
3. Verified active node graph and duplicate node names
4. Verified lifecycle state for bt_navigator, controller_server, planner_server, recoveries_server, waypoint_follower
5. Verified key topics:
   - /map
   - /tf
   - /tf_static
   - /car_1/odom
   - /car_1/scan
6. Verified TF transform map -> car_1_base_link

## How the issue was found (log-first approach)

The debugging was done by reproducing your exact workflow and reading runtime logs in sequence.

1. Reproduced the failure with the same commands you use
   - ./dev.sh build
   - ros2 launch my_robot diag_launch.py (inside container)

2. Collected launch logs from the running terminal session
   - Read the full ros2 launch output (not only the last error line)
   - Identified first hard failure in bt_navigator before downstream errors

3. Verified environment compatibility against logs
   - Compared missing library error from logs with actual files in /opt/ros/foxy/lib
   - Compared BT XML load error with files in /opt/ros/foxy/share/nav2_bt_navigator/behavior_trees

4. Used graph/lifecycle evidence from logs and runtime tools
   - Checked lifecycle states to confirm where activation stopped
   - Checked node list to detect duplicate process trees (same node names repeated)
   - Checked TF and topic availability to confirm backend health after fixes

This log-first method avoided guesswork and made each fix directly tied to observed evidence.

## Files changed and why

1. my_robot/config/nav2/nav2_params.yaml
   - Added explicit plugin_lib_names list compatible with ROS2 Foxy libraries
   - Set default_bt_xml_filename to absolute file path:
     /opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
   Result: bt_navigator no longer failed on missing plugin or XML lookup.

2. my_robot/launch/diag_launch.py
   - Removed redundant static TF publisher nodes for base_link->chassis and chassis->laser.
   - Kept robot_state_publisher as the source of URDF static TF chain.
   - Added bt_xml substitution and passed it to bt_navigator parameters.
   Result: Cleaner TF ownership and reduced TF duplication risk.

3. dev.sh
   - Added new command: launch
   - launch command now runs inside container with explicit sourcing:
     - /opt/ros/foxy/setup.bash
     - /sim_ws/install/setup.bash (if present)
     - /usr/share/gazebo/setup.bash
   - Then executes:
     ros2 launch my_robot diag_launch.py
   Result: One-command reliable launch flow from project root.

## Final status after fixes

On clean single launch, core stack status:

1. Nav2 lifecycle nodes reached active:
   - bt_navigator
   - controller_server
   - planner_server
   - recoveries_server
   - waypoint_follower

2. Core topics are present:
   - /map, /tf, /tf_static, /car_1/odom, /car_1/scan

3. TF connectivity is valid:
   - map -> car_1_base_link is available

Remaining observed issues are GUI/OpenGL related in this environment:

1. gzclient can crash due to display/rendering context
2. RViz can show GLSL texture sampler error

These are visual frontend problems and do not indicate Nav2/SLAM backend bringup failure.

## Recommended run sequence now

From car_run directory:

1. ./dev.sh compose
2. ./dev.sh build
3. ./dev.sh launch

If you ever see duplicate-node warnings again, restart the sim container before relaunching.

## Container restart advice (when and how)

Use a container restart when you see symptoms like:

1. Duplicate node name warnings for core Nav2/SLAM nodes
2. Lifecycle errors such as invalid sequence number or repeated failed transitions
3. Old launch processes still running after Ctrl+C

From car_run directory, restart with:

```bash
docker restart f1tenth_gym_ros-sim-1
```

Then run the normal flow again:

```bash
./dev.sh build
./dev.sh launch
```

Optional quick check before relaunch (inside container):

```bash
docker exec -it f1tenth_gym_ros-sim-1 bash -lc 'source /opt/ros/foxy/setup.bash && ros2 node list'
```

If this list already contains old Nav2 nodes before launching, restart container first.

## Latest Update (2026-04-16, Late Night)

### Operator request

1. Keep report updated after every change.
2. Investigate repeated runtime pattern:
   - `Message Filter dropping message: frame 'car_1_laser' ... reason 'Unknown'`
   - `bt_navigator ... "send_goal failed"`
3. Recheck launch wiring and Nav2/SLAM params end-to-end.

### New findings

1. System CPU stayed very high (~780-800%), dominated by Gazebo + RViz + SLAM + Nav2 planner/controller.
2. TF/scan timing jitter under load caused intermittent SLAM message-filter drops for `car_1_laser`.
3. Those timing stalls correlated with Nav2 callback failures (`send_goal failed`) even when user goal input was valid.
4. Lifecycle status checks could intermittently look "unavailable" under heavy load due to CLI timeout behavior.

### Changes applied

1. `dev.sh`
   - Re-added `headless` command for manual GUI-off mode.
   - Re-added `save_map` command to store maps under `my_robot/maps/`.
   - Improved `perf` lifecycle checks:
     - If lifecycle query times out, script now checks whether lifecycle service exists before marking node unavailable.
     - Recommendation logic now distinguishes true node-down from timeout-under-load.

2. `my_robot/launch/diag_launch.py`
   - RViz launch now forces fixed frame to `map` (`-f map`) to reduce empty-frame transform warning spam.
   - Existing Nav2 respawn wiring for `bt_navigator` kept intact.

3. `my_robot/config/slam/slam_toolbox_params.yaml`
   - `map_file_name` kept at `/sim_ws/src/my_robot/maps/mitrack_map` (project-local map save).
   - Load/TF robustness tuning:
     - `throttle_scans: 1 -> 2`
     - `minimum_time_interval: 0.5 -> 0.25`
     - `transform_timeout: 0.5 -> 1.0`
     - `tf_buffer_duration: 60 -> 120`

4. `my_robot/config/nav2/nav2_params.yaml`
   - Reduced controller pressure and increased tolerance to TF jitter:
     - `controller_frequency: 8.0 -> 6.0`
     - `required_movement_radius: 0.5 -> 0.2`
     - `movement_time_allowance: 10.0 -> 20.0`
     - `xy_goal_tolerance: 0.25 -> 0.30`
     - `yaw_goal_tolerance: 0.25 -> 0.30`
     - `desired_linear_vel: 0.35 -> 0.28`
     - `lookahead_dist: 0.6 -> 0.5`
     - `min_lookahead_dist: 0.3 -> 0.25`
     - RPP `transform_tolerance: 0.5 -> 0.8`
     - recoveries `transform_tolerance: 0.5 -> 0.8`
     - `waypoint_follower.loop_rate: 10 -> 5`
     - local costmap: `update_frequency: 3.0 -> 2.0`, `publish_frequency: 1.0 -> 0.5`, added `transform_tolerance: 0.8`
     - global costmap: `update_frequency: 0.5 -> 0.3`, `publish_frequency: 0.5 -> 0.3`, added `transform_tolerance: 0.8`

5. `my_robot/urdf/xacros/race.xacro`
   - Reduced lidar plugin rate:
     - `<update_rate>40</update_rate> -> <update_rate>20</update_rate>`

### Validation snapshot

1. Syntax checks passed:
   - `bash -n dev.sh`
   - `python3 -m py_compile my_robot/launch/diag_launch.py`
2. Parameter grep verification confirmed all new values are present in config files.
3. Remaining risk: with very high CPU, occasional transient TF drop lines may still appear; frequency should reduce after headless mode and lower sensor/controller rates.

### Current operating sequence

1. `./dev.sh stop`
2. `docker restart f1tenth_gym_ros-sim-1`
3. `./dev.sh launch`
4. Wait 25-30 seconds.
5. `./dev.sh headless`
6. `./dev.sh perf`
7. Send one goal at a time (no rapid repeated goal spam).

## Latest Update (2026-04-16, Stage A: Waypoints)

### Objective

1. Start trajectory workflow by defining waypoints directly on map.
2. Save waypoint clicks into reusable CSV for next-stage trajectory generation.

### Changes applied

1. Added new ROS2 node:
    - `my_robot/my_robot/waypoint_recorder.py`
    - Subscribes to `/clicked_point` (RViz Publish Point tool).
    - Enforces map-frame waypoint capture (`frame_id = map`).
    - Publishes visual feedback markers on `/waypoint_markers`.
    - Saves CSV on Ctrl+C to `/sim_ws/src/my_robot/maps/*.csv`.
    - CSV columns: `id,x,y,z,yaw,v_hint` (yaw/v_hint left blank for Stage B).

2. Package wiring:
    - `my_robot/setup.py`
       - Added console script:
          - `waypoint_recorder = my_robot.waypoint_recorder:main`
    - `my_robot/package.xml`
       - Added dependencies:
          - `geometry_msgs`
          - `visualization_msgs`

3. Dev workflow command:
    - `dev.sh`
       - Added new command: `waypoint_record`
       - Usage:
          - `./dev.sh waypoint_record`
          - `./dev.sh waypoint_record <csv_name>`
       - Runs recorder inside container and saves to `my_robot/maps/`.

### Validation

1. `bash -n dev.sh` passed.
2. `python3 -m py_compile my_robot/my_robot/waypoint_recorder.py` passed.

### Stage A run steps

1. Ensure sim stack and RViz are running.
2. Run: `./dev.sh waypoint_record mitrack_waypoints.csv`
3. In RViz, use **Publish Point** and click desired path points in order.
4. Press Ctrl+C in waypoint terminal to save CSV.
5. Output file will be in `my_robot/maps/mitrack_waypoints.csv`.

## Latest Update (2026-04-16, Stage B: Reference Trajectory)

### Objective

1. Convert Stage A waypoint CSV into controller-ready reference trajectory.
2. Keep outputs in map frame and store under `my_robot/maps/`.

### Changes applied

1. Added Stage B builder script:
    - `my_robot/my_robot/trajectory_builder.py`
    - Input: waypoint CSV (`id,x,y,z,yaw,v_hint`)
    - Output: dense trajectory CSV with columns:
       - `idx,s,x,y,yaw,kappa,v_ref`
    - Core processing:
       - removes duplicate consecutive waypoints
       - piecewise-linear interpolation to fixed spacing
       - computes yaw from local tangent
       - computes curvature (`kappa`) via finite differences
       - builds speed profile from `v_hint` (if present) or `v_max`
       - applies lateral acceleration cap and forward/back acceleration smoothing

2. Package wiring:
    - `my_robot/setup.py`
       - added console script:
          - `trajectory_builder = my_robot.trajectory_builder:main`

3. Dev workflow command:
    - `dev.sh`
       - added `trajectory_build`
       - usage:
          - `./dev.sh trajectory_build [waypoint_csv] [trajectory_csv]`
       - behavior:
          - defaults to latest `waypoints*.csv` if input omitted
          - default output: `<waypoint_stem>_trajectory.csv`
          - runs inside container using `ros2 run my_robot trajectory_builder`

### Validation

1. Syntax checks passed:
    - `bash -n dev.sh`
    - `python3 -m py_compile my_robot/my_robot/trajectory_builder.py`

2. End-to-end run passed on real recorded file:
    - command:
       - `./dev.sh trajectory_build waypoints_20260416_232730.csv`
    - generated:
       - `my_robot/maps/waypoints_20260416_232730_trajectory.csv`
    - output summary:
       - `Trajectory points: 76`
       - params:
          - `spacing=0.100`
          - `v_max=0.800`
          - `a_lat_max=1.500`
          - `a_long_max=1.000`

### Stage B run steps (current)

1. Record waypoints (Stage A):
    - `./dev.sh waypoint_record`
2. Build trajectory:
    - `./dev.sh trajectory_build <waypoint_csv>`
3. Use generated trajectory CSV in Stage C controller integration.

## Latest Update (2026-04-16, Stage C: Trajectory Follower)

### Objective

1. Follow Stage B reference trajectory directly from CSV.
2. Publish control commands for car motion without Nav2 goal dispatch.

### Changes applied

1. Added Stage C follower node:
   - `my_robot/my_robot/trajectory_follower.py`
   - Reads trajectory CSV with fields: `x,y,yaw,v_ref`
   - Uses TF lookup (`map -> car_1_base_link`) for current pose
   - Implements pure-pursuit style control with lookahead target
   - Publishes:
     - `/car_1/cmd_vel` (Twist)
     - `/reference_trajectory` (Path visualization)
     - `/reference_target_marker` (current target marker)
   - Stops robot at goal tolerance and on shutdown.

2. Package wiring:
   - `my_robot/setup.py`
     - added console script:
       - `trajectory_follower = my_robot.trajectory_follower:main`
   - `my_robot/package.xml`
     - added dependencies:
       - `nav_msgs`
       - `tf2_ros`

3. Dev workflow command:
   - `dev.sh`
     - added `trajectory_follow`
     - usage:
       - `./dev.sh trajectory_follow [trajectory_csv]`
     - defaults to latest `*trajectory*.csv` in `my_robot/maps/`.

### Validation

1. Syntax checks passed:
   - `bash -n dev.sh`
   - `python3 -m py_compile my_robot/my_robot/trajectory_follower.py`
2. Build completed with updated package entrypoints.
3. Command startup check passed:
   - `./dev.sh trajectory_follow waypoints_20260416_232730_trajectory.csv`
   - follower node loaded trajectory and started control loop.

### Runtime note

1. During one smoke run, TF reported:
   - `target_frame map does not exist`
2. This indicates follower was started while required map-frame transform was not available in that moment.
3. Operational requirement for Stage C:
   - start follower only after full sim/SLAM stack is up and `map -> car_1_base_link` exists.
   - do not run active Nav2 goal and trajectory follower at the same time.

### Stage C run steps (current)

1. `./dev.sh launch`
2. Wait 20-30 seconds.
3. Optional: `./dev.sh headless`
4. Ensure trajectory exists:
   - `./dev.sh trajectory_build <waypoint_csv>`
5. Start tracking:
   - `./dev.sh trajectory_follow <trajectory_csv>`
6. Stop with Ctrl+C.

## Latest Update (2026-04-17, Auto-Move on Launch)

### Operator symptom

1. Car started moving right after `./dev.sh launch` without sending a new Nav2 goal.

### Root cause

1. `trajectory_follower` from Stage C remained active and kept publishing `/car_1/cmd_vel`.
2. Because it is independent of Nav2 goals, launch appeared to auto-drive even with no new goal.

### Evidence

1. `ros2 topic info /car_1/cmd_vel -v` showed `Node name: trajectory_follower` as publisher.
2. Process inspection showed follower binaries/runner under `ros2 run my_robot trajectory_follower`.

### Fix applied

1. Hardened cleanup in `dev.sh` `cleanup_runtime()` to kill Stage C/Stage A helper processes as well:
   - `trajectory_follower`
   - `ros2 run my_robot trajectory_follower`
   - `/sim_ws/install/my_robot/lib/my_robot/trajectory_follower`
   - `waypoint_recorder`
   - `ros2 run my_robot waypoint_recorder`
   - existing `trajectory_builder` cleanup kept.
2. This ensures `./dev.sh stop` and `./dev.sh launch` clear stale helper nodes before startup.

### Operational rule

1. Run either Nav2 goal flow or `trajectory_follow`, not both simultaneously.
2. Before a fresh launch, run `./dev.sh stop` once if unsure.

---

## Latest Update (2026-04-17, Map Mismatch + Trajectory Sync Issues)

### Operator symptom

1. Car followed trajectory but kept hitting walls even though the node started correctly.
2. Problem was reproducible on repeated `./dev.sh trajectory_follow moretrack_trajectory.csv` without re-recording waypoints.

### Root causes identified

#### 1. Wrong map used for trajectory recording (primary cause)

1. Existing trajectory `waypoints_20260416_232730_trajectory.csv` was recorded on the smaller `mitrack` track.
2. When running on `moretrack` (larger track), the coordinate systems are different — the car aimed for waypoints that fell outside the actual drivable area.

**Fix**: Documented correct workflow: record new waypoints on the active track before building trajectory.

#### 2. No TF staleness check (secondary cause — hitting walls under CPU load)

1. Old `_lookup_robot_pose()` used `lookup_transform(..., rclpy.time.Time())` which always returns the latest cached transform.
2. Under high CPU load (800%+ common in Gazebo + Nav2 stack), the TF cache can be 500ms–1s stale.
3. Car used an old pose → computed wrong target → steered into wall.

**Fix**: Added staleness check in `_lookup_robot_pose()`:
- Compare TF stamp against current ROS clock.
- If age > `tf_max_age_sec` (default 0.5s), reject transform and stop the car.
- Stale count tracked; warm-up gate reset on each stale/failed read.

#### 3. Startup cold-start position mismatch

1. `current_target_idx` always started at 0 (first trajectory point).
2. After SLAM builds the map there may be a small offset versus the trajectory recording session.
3. Car would aim for the wrong initial waypoint.

**Fix**: On first call (`current_target_idx == 0`), `_find_target_index()` searches the **entire** trajectory to pick the globally nearest point as the start.

#### 4. TF warm-up sync guard missing

1. Old code sent drive commands on the very first control tick, before SLAM converged the map frame.
2. Resulted in initial jerk/wrong direction burst before SLAM settled.

**Fix**: Added `tf_warmup_count=5` consecutive fresh-TF reads required before any `cmd_vel` is published. Car holds still during warm-up. Gate resets whenever TF is stale or unavailable.

#### 5. Narrow search window (minor)

1. `_find_target_index` searched only 300 points ahead.
2. If car slipped ahead (stale TF jump), it could not recover backward.

**Fix**: Window increased to 400 points (= 40m at 0.10m spacing).

### Files changed

| File | Change |
|------|--------|
| `my_robot/my_robot/trajectory_follower.py` | TF staleness check, warm-up gate, global cold-start search, wider search window |
| `my_robot/launch/diag_launch.py` | Added `map` launch argument (default `moretrack`) for future map switching |
| `dev.sh` | Added `maps` command (lists available track maps with recommendations) |

### New parameters in trajectory_follower

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tf_max_age_sec` | `0.5` | Max allowed TF age in seconds before treating as stale |
| `tf_warmup_count` | `5` | Consecutive fresh TF reads required before control starts |

### Validation

1. `python3 -m py_compile my_robot/my_robot/trajectory_follower.py` → OK
2. `bash -n dev.sh` → OK

### Correct workflow for trajectory tracking on a new track

```bash
# 1. Launch simulation
./dev.sh launch

# 2. Wait ~20s for RViz to open

# 3. Record new waypoints on the active track
./dev.sh waypoint_record moretrack_waypoints.csv

# 4. Build trajectory from those waypoints
./dev.sh trajectory_build moretrack_waypoints.csv moretrack_trajectory.csv

# 5. Follow the new trajectory (always use trajectory matched to current track)
./dev.sh trajectory_follow moretrack_trajectory.csv
```

### Operational rules (updated)

1. Always record waypoints on the **same track** that the simulation is running.
2. Never reuse a trajectory file recorded on a different track.
3. `./dev.sh maps` shows all available maps and recommended tracks.
4. `trajectory_follow` now waits for TF to stabilize before moving (warm-up gate).
5. If car jerks at startup → increase `tf_warmup_count` parameter.
6. If car veers under high CPU → decrease `tf_max_age_sec` to 0.3 for stricter staleness rejection.

---

## Latest Update (2026-04-17, Sim/Wall Clock Mismatch — Car Never Moved)

### Operator symptom

1. After the TF staleness fix, car never moved at all.
2. Log showed continuous stale TF warnings with absurd age values:
   ```
   [WARN] Stale TF: age=1776380672.07s > limit=0.5s (stale count=586). Holding last command.
   ```

### Root cause

1. The manual staleness check compared `tf.header.stamp` (in **Gazebo sim time**, a large epoch-based value ~1.77 billion seconds) against `self.get_clock().now()` (returning **wall clock time**, also large but different offset).
2. The subtraction always produced a huge positive number, so **every** transform was flagged as stale immediately.
3. Consequence: car was permanently held, warm-up gate never cleared.

### Fix applied

1. Removed the manual clock-based age calculation entirely from `_lookup_robot_pose()`.
2. Now uses `tf_buffer.lookup_transform(..., timeout=rclpy.duration.Duration(seconds=0.1))` — tf2 itself blocks briefly and raises `TransformException` if no valid transform is available.
3. This is correct regardless of sim vs wall time since tf2 handles the clock internally.
4. Removed now-unused `tf_max_age_sec` parameter and `time` / `builtin_interfaces` imports.

### Files changed

| File | Change |
|------|--------|
| `my_robot/my_robot/trajectory_follower.py` | Removed manual age check; use tf2 `timeout=` arg; removed unused imports and `tf_max_age_sec` param |

### Validation

1. `python3 -m py_compile my_robot/my_robot/trajectory_follower.py` → OK

### Lesson learned

Never compare `tf.header.stamp` to `rclpy.Clock().now()` in a simulation — the TF stamp is in sim time while the node clock may be wall time unless `use_sim_time:=true` is explicitly set. Always delegate staleness/timeout to tf2's own `timeout` parameter.

---

## Latest Update (2026-04-17, PID Controller as Separate Node)

### Operator request

1. Implement PID controller, but keep existing `trajectory_follower.py` untouched.
2. Use PID controller as the default for future trajectory following.

### Solution applied

1. Restored `my_robot/my_robot/trajectory_follower.py` to non-PID controller logic.
2. Added new dedicated node: `my_robot/my_robot/pid_trajectory_follower.py`.
3. Registered new ROS2 entry point in `my_robot/setup.py`:
   - `pid_trajectory_follower = my_robot.pid_trajectory_follower:main`
4. Updated `dev.sh` `trajectory_follow` command to run PID node by default:
   - `ros2 run my_robot pid_trajectory_follower ...`
5. Updated `cleanup_runtime()` in `dev.sh` to kill stale PID follower processes.

### PID controller design (new node)

1. Control error:
   - heading error + weighted cross-track error
2. Angular control:
   - PID on control error (`kp`, `ki`, `kd`) with integral clamp
3. Speed policy:
   - adaptive slowdown on large control error to improve cornering stability
4. Startup robustness:
   - same TF warm-up guard and tf2 lookup timeout strategy as current stable follower

### New PID parameters

1. `pid_kp` (default `1.6`)
2. `pid_ki` (default `0.04`)
3. `pid_kd` (default `0.18`)
4. `cte_gain` (default `0.9`)
5. `pid_integral_limit` (default `0.8`)
6. `turn_slowdown_gain` (default `0.35`)
7. `min_linear_speed` (default `0.08`)

### Validation

1. `python3 -m py_compile my_robot/my_robot/trajectory_follower.py my_robot/my_robot/pid_trajectory_follower.py` → OK
2. `bash -n dev.sh` → OK

### Operational note

1. Future `./dev.sh trajectory_follow <trajectory.csv>` now uses PID controller node.
2. Legacy non-PID node remains available as `trajectory_follower` for fallback/comparison.

---

## Latest Update (2026-04-17, dev.sh PID and Non-PID Split)

### Operator request

1. Expose both PID and non-PID trajectory follow modes in `dev.sh`.
2. Clarify what non-PID controller is used.

### Changes applied

1. Added explicit commands in `dev.sh`:
   - `trajectory_follow_pid`
   - `trajectory_follow_non_pid`
2. Kept `trajectory_follow` as default PID alias for backward compatibility.
3. Refactored run logic through shared helper function `trajectory_follow_with_node()`.
4. Updated help/usage text to show all 3 follow commands.

### Controller mapping

1. `trajectory_follow` -> `pid_trajectory_follower` (PID, default)
2. `trajectory_follow_pid` -> `pid_trajectory_follower` (PID, explicit)
3. `trajectory_follow_non_pid` -> `trajectory_follower` (non-PID)

### Non-PID controller details

1. The non-PID node (`trajectory_follower`) uses a geometric pure-pursuit style heading controller.
2. Steering law:
   - target heading error `alpha = heading_to_target - yaw`
   - angular command from curvature-like term `omega = heading_gain * (2*v*sin(alpha)/lookahead)`
3. It is not PID (no integral/derivative terms).

### Validation

1. `bash -n dev.sh` -> OK
2. `./dev.sh --help` includes:
   - `trajectory_follow`
   - `trajectory_follow_pid`
   - `trajectory_follow_non_pid`

---

## Latest Update (2026-04-17, Navigation Data Collection Pipeline)

### Operator request

1. Collect data during navigation for analysis/learning.
2. Include sensor readings, control inputs, system state, and error metrics.
3. Store data in a structured dataset for future use.

### Solution applied

1. Added new ROS2 node:
   - `my_robot/my_robot/navigation_data_collector.py`
2. Added package entry point in `my_robot/setup.py`:
   - `navigation_data_collector = my_robot.navigation_data_collector:main`
3. Added dependency in `my_robot/package.xml`:
   - `sensor_msgs`
4. Added new `dev.sh` command:
   - `./dev.sh data_collect [output_csv] [trajectory_csv]`
5. Added cleanup kill patterns for `navigation_data_collector` in `cleanup_runtime()`.

### Dataset content (CSV schema)

1. Timestamp:
   - `timestamp_sec`
2. System state (pose/velocity):
   - `pose_x, pose_y, pose_yaw`
   - `odom_linear_x, odom_angular_z`
3. Control inputs:
   - `cmd_linear_x, cmd_angular_z`
4. IMU readings:
   - `imu_ax, imu_ay, imu_az`
   - `imu_gx, imu_gy, imu_gz`
5. Lidar readings (structured summary):
   - `scan_count, scan_min, scan_max, scan_mean, scan_std`
   - `scan_front_min, scan_left_min, scan_right_min`
6. Trajectory error metrics (if trajectory file provided):
   - `traj_nearest_idx, traj_target_idx`
   - `traj_cross_track_error, traj_heading_error, traj_goal_distance`

### Topics used

1. `/car_1/scan` (Lidar)
2. `/car_1/imu` (IMU, if available)
3. `/car_1/odom` (odometry)
4. `/car_1/cmd_vel` (applied control)
5. TF: `map -> car_1_base_link` for pose extraction

### Output and usage

1. Default output path:
   - `my_robot/datasets/nav_dataset_<timestamp>.csv`
2. Start collection:
   - `./dev.sh data_collect`
3. With explicit output and trajectory metrics:
   - `./dev.sh data_collect run1.csv moretrack_trajectory.csv`
4. Stop with `Ctrl+C`.

### Validation

1. `python3 -m py_compile my_robot/my_robot/navigation_data_collector.py` -> OK
2. `bash -n dev.sh` -> OK
3. `./dev.sh --help` shows `data_collect` command.

---

## Latest Update (2026-04-17, data_collect Runtime Fix)

### Operator symptom

1. `./dev.sh data_collect` crashed during runtime with:
   - `ValueError: not enough values to unpack (expected 8, got 7)`
2. Failure point: `_log_step()` unpacking scan statistics tuple.

### Root cause

1. `_scan_stats()` had inconsistent tuple sizes across branches.
2. Two early-return branches returned 7 values instead of 8.
3. Normal branch returned 8 values, causing intermittent unpack mismatch depending on scan readiness.

### Fix applied

1. Updated `_scan_stats()` return annotation to 8 values.
2. Updated all early-return branches to return 8 values consistently.
3. File changed:
   - `my_robot/my_robot/navigation_data_collector.py`

### Validation

1. `python3 -m py_compile my_robot/my_robot/navigation_data_collector.py` -> OK

### Operator action required

1. Rebuild workspace so container uses updated node code:
   - `./dev.sh build`
2. Re-run collector:
   - `./dev.sh data_collect`
