#!/bin/bash

# ╔════════════════════════════════════════════════════════════════════════════╗
# ║                    ROS2 F1TENTH DEVELOPMENT MASTER SCRIPT                  ║
# ║                  One Script for All Development Workflows                  ║
# ╚════════════════════════════════════════════════════════════════════════════╝

set -e

usage() {
    cat << 'EOF'
╔════════════════════════════════════════════════════════════════════════════╗
║                     DEVELOPMENT WORKFLOW - MASTER SCRIPT                   ║
╚════════════════════════════════════════════════════════════════════════════╝

USAGE: ./dev.sh [command]

COMMANDS:
═════════════════════════════════════════════════════════════════════════════

1. compose
   Start docker-compose environment
   Run in: TERMINAL 1
   Usage: ./dev.sh compose
   
   (Keep this terminal running - shows container logs)

───────────────────────────────────────────────────────────────────────────

2. build
   Build packages and open container terminal
   Run in: TERMINAL 2 (after compose is ready)
   Usage: ./dev.sh build
   
   Wait 15-20 seconds after running 'compose' before using this

───────────────────────────────────────────────────────────────────────────

3. full
   Complete workflow: stop, start compose, build, and open terminal
   Run in: TERMINAL 1 (one-shot automated)
   Usage: ./dev.sh full

───────────────────────────────────────────────────────────────────────────

4. launch
    Clean old processes, build packages, then launch Gazebo + SLAM + Nav2
    in the current terminal with full visible output
    Run in: TERMINAL 2
    Usage: ./dev.sh launch

───────────────────────────────────────────────────────────────────────────

5. logs
    Show live launch logs from inside the container
    Usage: ./dev.sh logs

───────────────────────────────────────────────────────────────────────────

6. stop
    Stop running Gazebo + SLAM + Nav2 processes inside the container
    Usage: ./dev.sh stop

───────────────────────────────────────────────────────────────────────────

7. health
    Check for duplicate critical ROS nodes in the running container
    Usage: ./dev.sh health

───────────────────────────────────────────────────────────────────────────

8. cancel_goal
    Cancel all active /navigate_to_pose goals (safe when Nav2 appears stuck)
    Usage: ./dev.sh cancel_goal

───────────────────────────────────────────────────────────────────────────

9. perf
    Snapshot CPU/memory and Nav2 runtime health indicators
    Usage: ./dev.sh perf

───────────────────────────────────────────────────────────────────────────

10. logdir
    Print latest ROS2 log directory path inside container
    Usage: ./dev.sh logdir

───────────────────────────────────────────────────────────────────────────

11. capture
    Capture triage snapshot to host file under diagnostics/
    Usage: ./dev.sh capture

───────────────────────────────────────────────────────────────────────────

12. headless
    Close Gazebo GUI (gzclient) and keep simulation server (gzserver) running
    Usage: ./dev.sh headless

───────────────────────────────────────────────────────────────────────────

13. save_map
    Save current SLAM map with optional name to my_robot/maps/
    Usage: ./dev.sh save_map [map_name]
    
    Examples:
      ./dev.sh save_map                    (saves as mitrack_map_TIMESTAMP)
      ./dev.sh save_map levine_custom      (saves as levine_custom.yaml)

───────────────────────────────────────────────────────────────────────────

14. waypoint_record
        Record RViz clicked waypoints (/clicked_point) and save CSV in my_robot/maps/
        Usage: ./dev.sh waypoint_record [csv_name]
    
        Examples:
            ./dev.sh waypoint_record
            ./dev.sh waypoint_record mitrack_waypoints.csv

───────────────────────────────────────────────────────────────────────────

15. trajectory_build
        Convert waypoint CSV to dense reference trajectory CSV
        Usage: ./dev.sh trajectory_build [waypoint_csv] [trajectory_csv]

        Examples:
            ./dev.sh trajectory_build
            ./dev.sh trajectory_build waypoints_20260416_232730.csv
            ./dev.sh trajectory_build waypoints.csv mitrack_ref_traj.csv

───────────────────────────────────────────────────────────────────────────

16. trajectory_follow
    Follow reference trajectory CSV using PID controller (default)
    Usage: ./dev.sh trajectory_follow [trajectory_csv]

    Examples:
      ./dev.sh trajectory_follow
      ./dev.sh trajectory_follow waypoints_20260416_232730_trajectory.csv

───────────────────────────────────────────────────────────────────────────

17. trajectory_follow_pid
    Follow reference trajectory CSV using PID controller (explicit)
    Usage: ./dev.sh trajectory_follow_pid [trajectory_csv]

    Examples:
      ./dev.sh trajectory_follow_pid
      ./dev.sh trajectory_follow_pid moretrack_trajectory.csv

───────────────────────────────────────────────────────────────────────────

18. trajectory_follow_non_pid
    Follow reference trajectory CSV using legacy non-PID controller
    (geometric pure-pursuit style heading controller)
    Usage: ./dev.sh trajectory_follow_non_pid [trajectory_csv]

    Examples:
      ./dev.sh trajectory_follow_non_pid
      ./dev.sh trajectory_follow_non_pid moretrack_trajectory.csv

───────────────────────────────────────────────────────────────────────────

19. data_collect
    Collect navigation dataset (sensors + controls + state + error metrics)
    Usage: ./dev.sh data_collect [output_csv] [trajectory_csv]

    Examples:
    ./dev.sh data_collect
    ./dev.sh data_collect run1.csv
    ./dev.sh data_collect run1.csv moretrack_trajectory.csv

───────────────────────────────────────────────────────────────────────────

20. bc_train
    Train behavioral-cloning model from dataset CSV
    Usage: ./dev.sh bc_train [dataset_csv] [model_json]

    Examples:
    ./dev.sh bc_train
    ./dev.sh bc_train nav_dataset_20260417_020049.csv
    ./dev.sh bc_train run1.csv bc_model_run1.json

───────────────────────────────────────────────────────────────────────────

21. bc_train_host
    Train behavioral-cloning model on HOST (no Docker/ROS needed)
    Usage: ./dev.sh bc_train_host [dataset_csv] [model_json]

    Examples:
    ./dev.sh bc_train_host
    ./dev.sh bc_train_host nav_dataset_20260417_020049.csv
    ./dev.sh bc_train_host run1.csv bc_model_run1.json

───────────────────────────────────────────────────────────────────────────

22. trajectory_follow_bc
    Follow trajectory using trained behavioral-cloning model
    Usage: ./dev.sh trajectory_follow_bc [model_json] [trajectory_csv]

    Examples:
    ./dev.sh trajectory_follow_bc
    ./dev.sh trajectory_follow_bc bc_model_run1.json moretrack_trajectory.csv

───────────────────────────────────────────────────────────────────────────

23. maps
    List all available map files in my_robot/maps/
    Usage: ./dev.sh maps

═════════════════════════════════════════════════════════════════════════════
EOF
    exit 0
}

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_header() {
    echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${NC} $1"
    echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

CONTAINER="f1tenth_gym_ros-sim-1"
LAUNCH_LOG="/tmp/diag_launch.log"
ROS2_CONDA_ENV="${ROS2_CONDA_ENV:-ROS2}"

activate_host_ros2_env() {
    # Best effort: keep docker-only commands working even if conda is unavailable.
    if [ "${DEV_SH_SKIP_ROS2_ENV:-0}" = "1" ]; then
        return 0
    fi

    if [ "${CONDA_DEFAULT_ENV:-}" = "$ROS2_CONDA_ENV" ]; then
        return 0
    fi

    if command -v conda >/dev/null 2>&1; then
        conda_base="$(conda info --base 2>/dev/null || true)"
        if [ -n "$conda_base" ] && [ -f "$conda_base/etc/profile.d/conda.sh" ]; then
            # shellcheck source=/dev/null
            source "$conda_base/etc/profile.d/conda.sh"
        fi
    else
        for conda_sh in \
            "$HOME/miniconda3/etc/profile.d/conda.sh" \
            "$HOME/anaconda3/etc/profile.d/conda.sh" \
            "$HOME/mambaforge/etc/profile.d/conda.sh" \
            "$HOME/miniforge3/etc/profile.d/conda.sh"; do
            if [ -f "$conda_sh" ]; then
                # shellcheck source=/dev/null
                source "$conda_sh"
                break
            fi
        done
    fi

    if command -v conda >/dev/null 2>&1; then
        conda activate "$ROS2_CONDA_ENV" >/dev/null 2>&1 || true
    fi
}

latest_ros_log_dir_in_container() {
    docker exec -i "$CONTAINER" bash -lc 'ls -1dt /root/.ros/log/*/ 2>/dev/null | head -n 1 | sed "s:/$::"'
}

require_container() {
    if ! docker ps | grep -q "$CONTAINER"; then
        print_error "Container '$CONTAINER' not found or not running"
        print_info "Start it first with: ./dev.sh compose"
        exit 1
    fi
}

container_shell_prelude() {
    cat << 'EOF'
cd /sim_ws

source /opt/ros/foxy/setup.bash

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

source /usr/share/gazebo/setup.bash
EOF
}

cleanup_runtime() {
    # Cleanup can legitimately return non-zero (e.g. process already gone).
    # Do not let set -e abort the full launch flow.
    docker exec -i "$CONTAINER" bash -lc '
cd /sim_ws

pkill -f "ros2 launch my_robot diag_launch.py" || true
pkill -f "/opt/ros/foxy/lib/nav2_" || true
pkill -f "/opt/ros/foxy/lib/slam_toolbox/async_slam_toolbox_node" || true
pkill -f "/opt/ros/foxy/lib/gazebo_ros/spawn_entity.py" || true
pkill -f "/opt/ros/foxy/lib/rviz2/rviz2" || true
pkill -f "trajectory_follower" || true
pkill -f "ros2 run my_robot trajectory_follower" || true
pkill -f "/sim_ws/install/my_robot/lib/my_robot/trajectory_follower" || true
pkill -f "pid_trajectory_follower" || true
pkill -f "ros2 run my_robot pid_trajectory_follower" || true
pkill -f "/sim_ws/install/my_robot/lib/my_robot/pid_trajectory_follower" || true
pkill -f "navigation_data_collector" || true
pkill -f "ros2 run my_robot navigation_data_collector" || true
pkill -f "/sim_ws/install/my_robot/lib/my_robot/navigation_data_collector" || true
pkill -f "bc_trajectory_follower" || true
pkill -f "ros2 run my_robot bc_trajectory_follower" || true
pkill -f "/sim_ws/install/my_robot/lib/my_robot/bc_trajectory_follower" || true
pkill -f "waypoint_recorder" || true
pkill -f "ros2 run my_robot waypoint_recorder" || true
pkill -f "trajectory_builder" || true
pkill -f "gzserver" || true
rm -f /tmp/diag_launch.log
sleep 1
' || true
    return 0
}

check_duplicate_critical_nodes() {
    docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)

critical_nodes='^(\/bt_navigator|\/controller_server|\/planner_server|\/recoveries_server|\/waypoint_follower|\/slam_toolbox|\/robot_state_publisher)$'

matches=\$(ros2 node list 2>/dev/null | grep -E \"\$critical_nodes\" | sort || true)

if [ -z \"\$matches\" ]; then
    echo \"NO_NODES\"
    exit 0
fi

dupes=\$(echo \"\$matches\" | uniq -c | awk '\$1 > 1 {print \$0}')

if [ -n \"\$dupes\" ]; then
    echo \"DUPLICATES_FOUND\"
    echo \"\$dupes\"
    exit 2
fi

echo \"OK\"
"
}

build_workspace_noninteractive() {
    docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)

colcon build --packages-select f1tenth_description my_robot my_racing_agent
"
}

# ════════════════════════════════════════════════════════════════════════════
# DOCKER COMPOSE START
# ════════════════════════════════════════════════════════════════════════════

compose() {
    print_header "Starting Docker-Compose Environment"
    
    cd f1tenth_gym_ros
    
    print_info "Building and starting containers..."
    print_info "This may take 1-2 minutes on first run"
    print_info "Keep this terminal running (shows container logs)"
    echo ""
    
    docker-compose up --build
}

# ════════════════════════════════════════════════════════════════════════════
# BUILD & OPEN TERMINAL
# ════════════════════════════════════════════════════════════════════════════

build() {
    print_header "Building Packages & Opening Container Terminal"

    require_container
    
    print_success "Found container: $CONTAINER"
    echo ""
    
    docker exec -it "$CONTAINER" bash -lc "$(container_shell_prelude)

echo \"\"
echo \"=== Building ROS2 Packages ===\"
echo \"\"

colcon build --packages-select f1tenth_description my_robot my_racing_agent

echo \"\"
echo \"=== Build Complete ===\"
echo \"Entering interactive container shell...\"
echo \"\"

source install/setup.bash

exec bash
"
}

# ════════════════════════════════════════════════════════════════════════════
# FULL WORKFLOW
# ════════════════════════════════════════════════════════════════════════════

full() {
    print_header "Full Development Workflow"
    
    print_info "Stopping any existing containers..."
    
    cd f1tenth_gym_ros
    docker-compose down 2>/dev/null || true
    cd ..
    sleep 2
    
    print_info "Starting docker-compose..."
    
    cd f1tenth_gym_ros
    docker-compose up --build &
    cd ..
    
    print_info "Waiting for container to start..."
    
    for i in {1..30}; do
        if docker ps | grep -q "$CONTAINER"; then
            print_success "Container is ready!"
            break
        fi
        sleep 1
        echo -n "."
    done
    
    echo ""
    
    sleep 5
    
    print_info "Building packages..."

    docker exec -it "$CONTAINER" bash -lc "$(container_shell_prelude)

colcon build --packages-select f1tenth_description my_robot my_racing_agent

echo \"\"
echo \"Build complete. Opening shell...\"
echo \"\"

source install/setup.bash

exec bash
"
}

    # ════════════════════════════════════════════════════════════════════════════
    # LAUNCH DIAGNOSTIC STACK
    # ════════════════════════════════════════════════════════════════════════════

launch() {
    print_header "Clean + Build + Launch (Gazebo + SLAM + Nav2)"

    require_container

    print_success "Found container: $CONTAINER"
    print_info "Cleaning stale ROS/Gazebo processes"
    cleanup_runtime

    print_info "Checking duplicate critical nodes after cleanup"
    dup_check_out="$(check_duplicate_critical_nodes || true)"
    if echo "$dup_check_out" | grep -q "DUPLICATES_FOUND"; then
        print_error "Duplicate critical nodes still detected after cleanup"
        echo "$dup_check_out"
        print_info "Try: docker restart $CONTAINER"
        exit 1
    fi

    print_info "Building packages before launch"
    print_info "Running launch in this terminal"
    echo ""

    docker exec -it "$CONTAINER" bash -lc "$(container_shell_prelude)

colcon build --packages-select f1tenth_description my_robot my_racing_agent
source install/setup.bash

# Auto health check after stack startup (runs in background, launch stays foreground).
(
  sleep 25
  echo \"\"
  echo \"=== Auto Health Check (post-start) ===\"
  critical_nodes='^(\/bt_navigator|\/controller_server|\/planner_server|\/recoveries_server|\/waypoint_follower|\/slam_toolbox|\/robot_state_publisher)$'
  matches=\$(ros2 node list 2>/dev/null | grep -E \"\$critical_nodes\" | sort || true)
  if [ -z \"\$matches\" ]; then
      echo \"HEALTH: No critical nodes detected yet\"
  else
      dupes=\$(echo "\$matches" | uniq -c | awk '\$1 > 1 {print \$0}')
      if [ -n \"\$dupes\" ]; then
          echo \"HEALTH: DUPLICATE critical nodes found\"
          echo \"\$dupes\"
      else
          echo \"HEALTH: OK (no duplicate critical nodes)\"
      fi
  fi
  echo \"======================================\"
  echo \"\"
) &

ros2 launch my_robot diag_launch.py
"
}

logs() {
    print_header "Showing Launch Logs"

    require_container

    latest_log_dir="$(latest_ros_log_dir_in_container)"
    if [ -z "$latest_log_dir" ]; then
        print_error "No ROS log directory found inside container"
        print_info "Run ./dev.sh launch first"
        exit 1
    fi

    print_info "Following recent log lines from: $latest_log_dir"
    docker exec -it "$CONTAINER" bash -lc "tail -n 40 -F '$latest_log_dir'/*.log"
}

stop() {
    print_header "Stopping Gazebo + SLAM + Nav2"

    require_container

    cleanup_runtime
    print_success "Stopped running launch-related processes"
}

health() {
    print_header "ROS Node Health Check"

    require_container

    out="$(check_duplicate_critical_nodes || true)"

    if echo "$out" | grep -q "DUPLICATES_FOUND"; then
        print_error "Duplicate critical nodes found"
        echo "$out"
        print_info "Recommended: ./dev.sh stop"
        print_info "If still duplicated: docker restart $CONTAINER"
        exit 1
    fi

    if echo "$out" | grep -q "NO_NODES"; then
        print_info "No critical Nav2/SLAM nodes are currently running"
        exit 0
    fi

    print_success "No duplicate critical nodes detected"
}

cancel_goal() {
    print_header "Cancel Active NavigateToPose Goals"

    require_container

    docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)

echo \"Calling /navigate_to_pose/_action/cancel_goal ...\"
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}' || true
"

    print_success "Cancel request sent (all active goals)"
}

perf() {
    print_header "Performance + Runtime Snapshot"

    require_container

    cpu_raw="$(docker stats --no-stream --format '{{.CPUPerc}}' "$CONTAINER" | tr -d '%')"
    cpu_int="${cpu_raw%.*}"
    [ -z "$cpu_int" ] && cpu_int=0

    echo "=== docker stats (single snapshot) ==="
    docker stats --no-stream --format 'table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.MemPerc}}' "$CONTAINER"
    echo ""

    docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)

echo '=== Top CPU processes (container) ==='
ps -eo pid,pcpu,pmem,comm,args --sort=-pcpu | head -n 12
echo ''

echo '=== Nav2 action servers ==='
ros2 action list 2>/dev/null | grep -E 'navigate_to_pose|compute_path_to_pose|follow_path' || true
echo ''

echo '=== Lifecycle states ==='
for n in /controller_server /planner_server /recoveries_server /bt_navigator /waypoint_follower; do
    state=\$(ros2 lifecycle get \"\$n\" 2>/dev/null | tr '\n' ' ' || true)
    if [ -n \"\$state\" ]; then
        echo \"\$n -> \$state\"
    else
        svc=\"\${n}/get_state\"
        if ros2 service list 2>/dev/null | grep -qx \"\$svc\"; then
            echo \"\$n -> lifecycle service present (state query timeout under load)\"
        else
            echo \"\$n -> unavailable\"
        fi
    fi
done
"

    bt_state="$(docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)
ros2 lifecycle get /bt_navigator 2>/dev/null | tr '\n' ' ' || true
")"

        nav_action_present="$(docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)
ros2 action list 2>/dev/null | grep -qx '/navigate_to_pose' && echo yes || echo no
")"

        bt_get_state_present="$(docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)
ros2 service list 2>/dev/null | grep -qx '/bt_navigator/get_state' && echo yes || echo no
")"

    latest_log_dir="$(latest_ros_log_dir_in_container)"
    if [ -n "$latest_log_dir" ]; then
        echo ""
        echo "=== Recent warning/error patterns (latest ROS logs) ==="
        docker exec -i "$CONTAINER" bash -lc "grep -RniE 'send_goal failed|Failed to make progress|Control loop missed|Message Filter dropping|Failed to create a plan|Navigation failed' '$latest_log_dir' 2>/dev/null | tail -n 40 || true"
    fi

    echo ""
    echo "=== Quick Recommendation ==="
    if [ "$cpu_int" -ge 500 ]; then
        echo "CPU pressure is HIGH (${cpu_raw}%). Slowdown is likely load-related (Gazebo + RViz + Nav2)."
        echo "Action: avoid sending repeated goals; cancel once (./dev.sh cancel_goal), wait 2-3s, then send one new goal."
    else
        echo "CPU pressure is moderate (${cpu_raw}%)."
    fi

    if echo "$bt_state" | grep -qi 'active \[3\]'; then
        echo "bt_navigator is ACTIVE: prefer cancel+single retry before relaunch."
    elif [ "$nav_action_present" = "yes" ] || [ "$bt_get_state_present" = "yes" ]; then
        echo "bt_navigator appears UP (action/service present), but lifecycle query timed out under load."
        echo "Action: use headless mode (./dev.sh headless), wait 3-5s, then send one goal."
    else
        echo "bt_navigator is NOT active: relaunch is recommended (./dev.sh launch)."
    fi
}

logdir() {
    print_header "Latest ROS Log Directory"

    require_container

    latest_log_dir="$(latest_ros_log_dir_in_container)"
    if [ -z "$latest_log_dir" ]; then
        print_error "No ROS log directory found"
        print_info "Run ./dev.sh launch first"
        exit 1
    fi

    print_success "$latest_log_dir"
}

capture() {
    print_header "Capture Nav2 Triage Snapshot"

    require_container

    mkdir -p diagnostics
    ts="$(date +%Y%m%d_%H%M%S)"
    out="diagnostics/nav2_capture_${ts}.log"

    {
        echo "===== NAV2 TRIAGE SNAPSHOT ====="
        echo "Timestamp: $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
        echo "Container: $CONTAINER"
        echo ""

        echo "=== docker stats (single snapshot) ==="
        docker stats --no-stream --format 'table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.MemPerc}}' "$CONTAINER"
        echo ""

        echo "=== ROS node list ==="
        docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)
ros2 node list 2>/dev/null | sort || true"
        echo ""

        echo "=== Nav2 lifecycle states ==="
        docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)
for n in /controller_server /planner_server /recoveries_server /bt_navigator /waypoint_follower; do
  echo -n \"\$n -> \"
  ros2 lifecycle get \"\$n\" 2>/dev/null | tr '\n' ' ' || echo 'unavailable'
  echo ''
done"
        echo ""

        echo "=== Action servers ==="
        docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)
ros2 action list 2>/dev/null | sort || true"
        echo ""

        latest_log_dir="$(latest_ros_log_dir_in_container)"
        echo "Latest ROS log dir: ${latest_log_dir:-N/A}"
        if [ -n "$latest_log_dir" ]; then
            echo ""
            echo "=== Recent issue patterns from ROS logs ==="
            docker exec -i "$CONTAINER" bash -lc "grep -RniE 'send_goal failed|Failed to make progress|Control loop missed|Message Filter dropping|Failed to create a plan|Navigation failed' '$latest_log_dir' 2>/dev/null | tail -n 120 || true"
        fi
    } > "$out"

    print_success "Wrote snapshot: $out"
}

headless() {
    print_header "Switch To Headless Gazebo"

    require_container

    print_info "Stopping Gazebo GUI client (gzclient)"
    docker exec -i "$CONTAINER" bash -lc "pkill -x gzclient || true"

    print_info "Process status after switch:"
    docker exec -i "$CONTAINER" bash -lc 'echo "  gzserver: $(pgrep -x gzserver | wc -l)"; echo "  gzclient: $(pgrep -x gzclient | wc -l)"'

    print_success "Headless mode enabled (simulation still running in gzserver)"
}

save_map() {
    print_header "Save SLAM Map"

    require_container

    map_name="${1:-mitrack_map_$(date +%Y%m%d_%H%M%S)}"
    
    print_info "Saving map as: $map_name"
    
    docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)

echo 'Calling /slam_toolbox/save_map service...'
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap '{name: {data: \"/sim_ws/src/my_robot/maps/$map_name\"}}'
"

    print_info "Map saved directly to my_robot/maps/"
    sleep 1
    
    if ls ./my_robot/maps/${map_name}* 2>/dev/null | grep -q .; then
        print_success "Map files created:"
        ls -lh ./my_robot/maps/${map_name}* 2>/dev/null
    else
        print_info "Checking container for map files..."
        docker exec -i "$CONTAINER" bash -lc "ls -lh /sim_ws/src/my_robot/maps/${map_name}* 2>/dev/null || echo 'Check SLAM logs if no files created'"
    fi
}

waypoint_record() {
    print_header "Record Waypoints From RViz"

    require_container

    csv_name="${1:-waypoints_$(date +%Y%m%d_%H%M%S).csv}"
    if [[ "$csv_name" != *.csv ]]; then
        csv_name="${csv_name}.csv"
    fi
    output_file="/sim_ws/src/my_robot/maps/${csv_name}"

    print_info "Open RViz and click points using 'Publish Point' tool"
    print_info "Press Ctrl+C in this terminal when done"
    print_info "CSV output: $output_file"
    echo ""

    docker exec -it "$CONTAINER" bash -lc "$(container_shell_prelude)

source install/setup.bash
ros2 run my_robot waypoint_recorder --ros-args -p output_file:='$output_file' -p frame_id:='map'
"

    print_success "Waypoint recording finished"
}

trajectory_build() {
    print_header "Build Reference Trajectory"

    require_container

    waypoint_csv="$1"
    trajectory_csv="$2"

    if [ -z "$waypoint_csv" ]; then
        waypoint_csv="$(ls -1t ./my_robot/maps/waypoints*.csv 2>/dev/null | head -n 1 | xargs -n1 basename 2>/dev/null || true)"
    fi

    if [ -z "$waypoint_csv" ]; then
        print_error "No waypoint CSV found in my_robot/maps"
        print_info "Record waypoints first: ./dev.sh waypoint_record"
        exit 1
    fi

    if [[ "$waypoint_csv" != *.csv ]]; then
        waypoint_csv="${waypoint_csv}.csv"
    fi

    if [ -z "$trajectory_csv" ]; then
        stem="${waypoint_csv%.csv}"
        trajectory_csv="${stem}_trajectory.csv"
    elif [[ "$trajectory_csv" != *.csv ]]; then
        trajectory_csv="${trajectory_csv}.csv"
    fi

    waypoint_path="/sim_ws/src/my_robot/maps/${waypoint_csv}"
    trajectory_path="/sim_ws/src/my_robot/maps/${trajectory_csv}"

    print_info "Waypoint input:  ${waypoint_path}"
    print_info "Trajectory out: ${trajectory_path}"
    echo ""

    docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)

source install/setup.bash
ros2 run my_robot trajectory_builder --input '$waypoint_path' --output '$trajectory_path' --spacing 0.10 --v-max 0.80 --a-lat-max 1.50 --a-long-max 1.00
"

    if ls ./my_robot/maps/${trajectory_csv} 1>/dev/null 2>&1; then
        print_success "Trajectory CSV created: ./my_robot/maps/${trajectory_csv}"
        print_info "Preview:"
        head -n 8 ./my_robot/maps/${trajectory_csv}
    else
        print_error "Trajectory file not found on host path"
        print_info "Check container path: ${trajectory_path}"
        exit 1
    fi
}

trajectory_follow_with_node() {
    node_executable="$1"
    controller_label="$2"
    trajectory_csv="$3"

    print_header "Follow Reference Trajectory (${controller_label})"

    require_container

    if [ -z "$trajectory_csv" ]; then
        trajectory_csv="$(ls -1t ./my_robot/maps/*trajectory*.csv 2>/dev/null | head -n 1 | xargs -n1 basename 2>/dev/null || true)"
    fi

    if [ -z "$trajectory_csv" ]; then
        print_error "No trajectory CSV found in my_robot/maps"
        print_info "Build one first: ./dev.sh trajectory_build"
        exit 1
    fi

    if [[ "$trajectory_csv" != *.csv ]]; then
        trajectory_csv="${trajectory_csv}.csv"
    fi

    trajectory_path="/sim_ws/src/my_robot/maps/${trajectory_csv}"

    print_info "Trajectory input: ${trajectory_path}"
    print_info "Controller: ${controller_label}"
    print_info "Start this only when no Nav2 goal is active"
    print_info "Press Ctrl+C to stop follower"
    echo ""

    docker exec -it "$CONTAINER" bash -lc "$(container_shell_prelude)

source install/setup.bash
ros2 run my_robot ${node_executable} --ros-args -p trajectory_file:='$trajectory_path' -p global_frame:='map' -p robot_frame:='car_1_base_link' -p cmd_topic:='/car_1/cmd_vel'
"

    print_success "Trajectory follower stopped"
}

trajectory_follow() {
    trajectory_follow_with_node "pid_trajectory_follower" "PID (default)" "$1"
}

trajectory_follow_pid() {
    trajectory_follow_with_node "pid_trajectory_follower" "PID" "$1"
}

trajectory_follow_non_pid() {
    trajectory_follow_with_node "trajectory_follower" "Non-PID (Pure Pursuit style)" "$1"
}

data_collect() {
    print_header "Collect Navigation Dataset"

    require_container

    output_csv="$1"
    trajectory_csv="$2"

    if [ -z "$output_csv" ]; then
        output_csv="nav_dataset_$(date +%Y%m%d_%H%M%S).csv"
    fi
    if [[ "$output_csv" != *.csv ]]; then
        output_csv="${output_csv}.csv"
    fi

    if [ -z "$trajectory_csv" ]; then
        trajectory_csv="$(ls -1t ./my_robot/maps/*trajectory*.csv 2>/dev/null | head -n 1 | xargs -n1 basename 2>/dev/null || true)"
    fi
    if [[ -n "$trajectory_csv" && "$trajectory_csv" != *.csv ]]; then
        trajectory_csv="${trajectory_csv}.csv"
    fi

    output_path="/sim_ws/src/my_robot/datasets/${output_csv}"

    if [ -n "$trajectory_csv" ]; then
        trajectory_path="/sim_ws/src/my_robot/maps/${trajectory_csv}"
        print_info "Trajectory for error metrics: ${trajectory_path}"
        traj_param="-p trajectory_file:='${trajectory_path}'"
    else
        print_info "No trajectory found; error metrics will be NaN"
        traj_param=""
    fi

    print_info "Dataset output: ${output_path}"
    print_info "Topics: /car_1/scan, /car_1/imu, /car_1/odom, /car_1/cmd_vel"
    print_info "Press Ctrl+C to stop collection"
    echo ""

    docker exec -it "$CONTAINER" bash -lc "$(container_shell_prelude)

mkdir -p /sim_ws/src/my_robot/datasets
source install/setup.bash
ros2 run my_robot navigation_data_collector --ros-args -p output_file:='${output_path}' ${traj_param} -p global_frame:='map' -p robot_frame:='car_1_base_link' -p scan_topic:='/car_1/scan' -p imu_topic:='/car_1/imu' -p odom_topic:='/car_1/odom' -p cmd_topic:='/car_1/cmd_vel' -p log_rate:=20.0
"

    if ls ./my_robot/datasets/${output_csv} 1>/dev/null 2>&1; then
        print_success "Dataset saved: ./my_robot/datasets/${output_csv}"
        print_info "Preview:"
        head -n 5 ./my_robot/datasets/${output_csv}
    else
        print_info "Dataset should be at: ${output_path}"
    fi
}

bc_train() {
    print_header "Train Behavioral Cloning Model"

    require_container

    dataset_csv="$1"
    model_json="$2"

    if [ -z "$dataset_csv" ]; then
        dataset_csv="$(ls -1t ./my_robot/datasets/nav_dataset*.csv 2>/dev/null | head -n 1 | xargs -n1 basename 2>/dev/null || true)"
    fi

    if [ -z "$dataset_csv" ]; then
        print_error "No dataset CSV found in my_robot/datasets"
        print_info "Collect one first: ./dev.sh data_collect"
        exit 1
    fi

    if [[ "$dataset_csv" != *.csv ]]; then
        dataset_csv="${dataset_csv}.csv"
    fi

    if [ -z "$model_json" ]; then
        model_json="bc_model_$(date +%Y%m%d_%H%M%S).json"
    fi
    if [[ "$model_json" != *.json ]]; then
        model_json="${model_json}.json"
    fi

    dataset_path="/sim_ws/src/my_robot/datasets/${dataset_csv}"
    model_path="/sim_ws/src/my_robot/models/${model_json}"

    print_info "Dataset input: ${dataset_path}"
    print_info "Model output: ${model_path}"
    echo ""

    docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)

mkdir -p /sim_ws/src/my_robot/models
source install/setup.bash
ros2 run my_robot train_behavioral_cloning --input '${dataset_path}' --output '${model_path}'
"

    if ls ./my_robot/models/${model_json} 1>/dev/null 2>&1; then
        print_success "BC model saved: ./my_robot/models/${model_json}"
    else
        print_error "Model file not found on host path"
        print_info "Check container path: ${model_path}"
        exit 1
    fi
}

bc_train_host() {
    print_header "Train Behavioral Cloning Model (Host / No ROS)"

    dataset_csv="$1"
    model_json="$2"

    if [ -z "$dataset_csv" ]; then
        dataset_csv="$(ls -1t ./my_robot/datasets/nav_dataset*.csv 2>/dev/null | head -n 1 || true)"
    fi

    if [ -z "$dataset_csv" ]; then
        print_error "No dataset CSV found in my_robot/datasets"
        print_info "Collect one first: ./dev.sh data_collect"
        exit 1
    fi

    if [[ "$dataset_csv" != *.csv ]]; then
        dataset_csv="${dataset_csv}.csv"
    fi
    # Resolve to host path if only a basename was given
    if [[ "$dataset_csv" != */* ]]; then
        dataset_csv="./my_robot/datasets/${dataset_csv}"
    fi

    if [ -z "$model_json" ]; then
        model_json="bc_model_$(date +%Y%m%d_%H%M%S).json"
    fi
    if [[ "$model_json" != *.json ]]; then
        model_json="${model_json}.json"
    fi
    if [[ "$model_json" != */* ]]; then
        mkdir -p ./my_robot/models
        model_json="./my_robot/models/${model_json}"
    fi

    print_info "Dataset input: ${dataset_csv}"
    print_info "Model output:  ${model_json}"
    echo ""

    python3 ./my_robot/my_robot/train_behavioral_cloning.py \
        --input "${dataset_csv}" \
        --output "${model_json}"

    if [ -f "$model_json" ]; then
        print_success "BC model saved: ${model_json}"
    else
        print_error "Model file not created: ${model_json}"
        exit 1
    fi
}

trajectory_follow_bc() {
    print_header "Follow Reference Trajectory (Behavioral Cloning)"

    require_container

    model_json="$1"
    trajectory_csv="$2"

    if [ -z "$model_json" ]; then
        model_json="$(ls -1t ./my_robot/models/bc_model*.json 2>/dev/null | head -n 1 | xargs -n1 basename 2>/dev/null || true)"
    fi
    if [ -z "$model_json" ]; then
        print_error "No BC model found in my_robot/models"
        print_info "Train one first: ./dev.sh bc_train"
        exit 1
    fi
    if [[ "$model_json" != *.json ]]; then
        model_json="${model_json}.json"
    fi

    if [ -z "$trajectory_csv" ]; then
        trajectory_csv="$(ls -1t ./my_robot/maps/*trajectory*.csv 2>/dev/null | head -n 1 | xargs -n1 basename 2>/dev/null || true)"
    fi
    if [ -z "$trajectory_csv" ]; then
        print_error "No trajectory CSV found in my_robot/maps"
        print_info "Build one first: ./dev.sh trajectory_build"
        exit 1
    fi
    if [[ "$trajectory_csv" != *.csv ]]; then
        trajectory_csv="${trajectory_csv}.csv"
    fi

    model_path="/sim_ws/src/my_robot/models/${model_json}"
    trajectory_path="/sim_ws/src/my_robot/maps/${trajectory_csv}"

    print_info "BC model: ${model_path}"
    print_info "Trajectory: ${trajectory_path}"
    print_info "Start this only when no Nav2 goal is active"
    print_info "Press Ctrl+C to stop follower"
    echo ""

    docker exec -it "$CONTAINER" bash -lc "$(container_shell_prelude)

source install/setup.bash
ros2 run my_robot bc_trajectory_follower --ros-args -p model_file:='${model_path}' -p trajectory_file:='${trajectory_path}' -p global_frame:='map' -p robot_frame:='car_1_base_link' -p scan_topic:='/car_1/scan' -p odom_topic:='/car_1/odom' -p cmd_topic:='/car_1/cmd_vel' -p control_rate:=20.0
"

    print_success "BC trajectory follower stopped"
}

maps() {
    print_header "Available Map Files"
    
    maps_dir="./my_robot/maps"
    
    if [ ! -d "$maps_dir" ]; then
        print_error "Maps directory not found: $maps_dir"
        exit 1
    fi
    
    print_info "Map files (.yaml):"
    echo ""
    ls -lhS "$maps_dir"/*.yaml 2>/dev/null | awk '{print "  " $9 " (" $5 ")"}' || {
        print_error "No map files found"
    }
    
    echo ""
    print_info "Recommended tracks:"
    echo "  • moretrack.yaml     - Larger track (recommended for trajectory tracking)"
    echo "  • trackhalf.yaml     - Half track variant"
    echo "  • mitrack_map_*.yaml - Saved mitrack maps from previous sessions"
    echo ""
    print_info "Quick start with better track:"
    echo "  ./dev.sh waypoint_record moretrack_waypoints.csv"
    echo "  ./dev.sh trajectory_build moretrack_waypoints.csv moretrack_trajectory.csv"
    echo "  ./dev.sh trajectory_follow moretrack_trajectory.csv"
}

# ════════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════════

activate_host_ros2_env

case "${1:-help}" in
    compose)
        compose
        ;;
    build)
        build
        ;;
    full)
        full
        ;;
    launch)
        launch
        ;;
    logs)
        logs
        ;;
    stop)
        stop
        ;;
    health)
        health
        ;;
    cancel_goal)
        cancel_goal
        ;;
    perf)
        perf
        ;;
    logdir)
        logdir
        ;;
    capture)
        capture
        ;;
    headless)
        headless
        ;;
    save_map)
        save_map "$2"
        ;;
    waypoint_record)
        waypoint_record "$2"
        ;;
    trajectory_build)
        trajectory_build "$2" "$3"
        ;;
    trajectory_follow)
        trajectory_follow "$2"
        ;;
    trajectory_follow_pid)
        trajectory_follow_pid "$2"
        ;;
    trajectory_follow_non_pid)
        trajectory_follow_non_pid "$2"
        ;;
    data_collect)
        data_collect "$2" "$3"
        ;;
    bc_train)
        bc_train "$2" "$3"
        ;;
    bc_train_host)
        bc_train_host "$2" "$3"
        ;;
    trajectory_follow_bc)
        trajectory_follow_bc "$2" "$3"
        ;;
    maps)
        maps
        ;;
    help|--help|-h|"")
        usage
        ;;
    *)
        print_error "Unknown command: $1"
        echo ""
        usage
        exit 1
        ;;
esac