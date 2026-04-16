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
    Launch Gazebo + SLAM + Nav2 from inside the running container
    Run in: TERMINAL 2 (after build)
    Usage: ./dev.sh launch

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
    
    CONTAINER="f1tenth_gym_ros-sim-1"
    
    if ! docker ps | grep -q "$CONTAINER"; then
        print_error "Container '$CONTAINER' not found or not running"
        print_info "Start it first with: ./dev.sh compose"
        exit 1
    fi
    
    print_success "Found container: $CONTAINER"
    echo ""
    
    docker exec -it "$CONTAINER" bash -c '
cd /sim_ws

source /opt/ros/foxy/setup.bash

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

source /usr/share/gazebo/setup.bash

echo ""
echo "=== Building ROS2 Packages ==="
echo ""

colcon build --packages-select f1tenth_description my_robot my_racing_agent

echo ""
echo "=== Build Complete ==="
echo "Entering interactive container shell..."
echo ""

source install/setup.bash


exec bash
'
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
    
    CONTAINER="f1tenth_gym_ros-sim-1"
    
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
    
    docker exec -it "$CONTAINER" bash -c '
    cd /sim_ws

    source /opt/ros/foxy/setup.bash

    if [ -f install/setup.bash ]; then
        source install/setup.bash
    fi

    source /usr/share/gazebo/setup.bash

    colcon build --packages-select f1tenth_description my_robot my_racing_agent

    echo ""
    echo "Build complete. Opening shell..."
    echo ""

    source install/setup.bash

    exec bash
    '
    }

    # ════════════════════════════════════════════════════════════════════════════
    # LAUNCH DIAGNOSTIC STACK
    # ════════════════════════════════════════════════════════════════════════════

    launch() {
        print_header "Launching Gazebo + SLAM + Nav2"

        CONTAINER="f1tenth_gym_ros-sim-1"

        if ! docker ps | grep -q "$CONTAINER"; then
            print_error "Container '$CONTAINER' not found or not running"
            print_info "Start it first with: ./dev.sh compose"
            exit 1
        fi

        print_success "Found container: $CONTAINER"
        print_info "Sourcing ROS + workspace setup inside container"
        print_info "Running: ros2 launch my_robot diag_launch.py"
        echo ""

        docker exec -it "$CONTAINER" bash -lc '
    cd /sim_ws

    source /opt/ros/foxy/setup.bash

    if [ -f install/setup.bash ]; then
        source install/setup.bash
    fi

    source /usr/share/gazebo/setup.bash

    ros2 launch my_robot diag_launch.py
    '
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