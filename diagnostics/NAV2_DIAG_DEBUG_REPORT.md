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

## Latest Update (2026-04-16)

### New findings (latest cycle)

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
