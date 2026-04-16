# Nav2 + Gazebo + RViz Debug Report

Date: 2026-04-16
Workspace: car_run

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
