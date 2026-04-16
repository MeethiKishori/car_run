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
pkill -f "gzclient" || true
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
    echo \"\$n -> unavailable\"
  fi
done
"

    bt_state="$(docker exec -i "$CONTAINER" bash -lc "$(container_shell_prelude)
ros2 lifecycle get /bt_navigator 2>/dev/null | tr '\n' ' ' || true
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

# ════════════════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════════════════

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