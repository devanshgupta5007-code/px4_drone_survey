#!/usr/bin/env bash
# ==============================================================================
# PX4 Autonomous Aerial Survey & Box Detection Simulation Launcher
# - Hardware Accelerated with NVIDIA RTX 2050
# - Demarcated Survey Area (60m x 50m) with Perimeter Borders & Corner Pylons
# - High-Efficiency Lawnmower Survey with Dubins Arcs (default: 6.0 m/s)
# - Real-time OpenCV 45-deg Camera Detection & Georeferencing Window
# ==============================================================================

set -eo pipefail

PROJECT_DIR="/home/devi/px4_drone_survey"
PX4_DIR="/home/devi/PX4-Autopilot"
CONTAINER="Khojo-o-Drone"

# Defaults
HEADLESS=0
SPEED="5.5"

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --headless) HEADLESS=1; shift ;;
        --speed) SPEED="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: ./run_sim.sh [--headless] [--speed <m/s>]"
            echo "  --headless       Run Gazebo simulation without GUI"
            echo "  --speed <m/s>    Set survey ground cruise speed (default: 5.5 m/s)"
            exit 0
            ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

echo "=================================================================="
echo "    PX4 DRONE SURVEY SIMULATION: 45-DEG CAMERA + MAVSDK           "
echo "    Survey Zone: 120m x 90m (Demarcated with Perimeter Pylons)    "
echo "    Speed: $SPEED m/s (S-Curve Stabilized) | GPU: RTX 2050        "
echo "=================================================================="

# Ensure models & worlds are synced
echo "[SETUP] Syncing custom 45-deg camera models and demarcated world to PX4..."
cp -r "$PROJECT_DIR/models/mono_cam_45" "$PX4_DIR/Tools/simulation/gz/models/"
cp -r "$PROJECT_DIR/models/x500_mono_cam_45" "$PX4_DIR/Tools/simulation/gz/models/"
cp "$PROJECT_DIR/worlds/box_survey.sdf" "$PX4_DIR/Tools/simulation/gz/worlds/"

# Cleanup previous instances
echo "[SETUP] Terminating any stale processes..."
pkill -9 -x px4 2>/dev/null || true
pkill -9 -x gz-sim-server 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 -f 'make px4_sitl' 2>/dev/null || true
pkill -9 -f '[s]urvey_mission' 2>/dev/null || true
pkill -9 -f '[m]avsdk_server' 2>/dev/null || true
distrobox-enter -n "$CONTAINER" < /dev/null -- bash -c "pkill -9 -x px4 2>/dev/null || true; pkill -9 -x gz-sim-server 2>/dev/null || true; pkill -9 -f 'gz sim' 2>/dev/null || true; pkill -9 -f '[g]z-sim' 2>/dev/null || true; pkill -9 -x gz 2>/dev/null || true; pkill -9 -f '[r]uby.*gz' 2>/dev/null || true; pkill -9 -f '[s]urvey_mission' 2>/dev/null || true; pkill -9 -f '[m]avsdk_server' 2>/dev/null || true" > /dev/null 2>&1 || true
sleep 1

# Setup trap to terminate children on Ctrl+C
cleanup() {
    trap - SIGINT SIGTERM EXIT
    echo -e "\n[CLEANUP] Gracefully shutting down simulation and vision pipeline..."
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -x gz-sim-server 2>/dev/null || true
    pkill -9 -f 'gz sim' 2>/dev/null || true
    pkill -9 -f 'make px4_sitl' 2>/dev/null || true
    pkill -9 -f '[s]urvey_mission' 2>/dev/null || true
    pkill -9 -f '[m]avsdk_server' 2>/dev/null || true
    distrobox-enter -n "$CONTAINER" < /dev/null -- bash -c "pkill -9 -x px4 2>/dev/null || true; pkill -9 -x gz-sim-server 2>/dev/null || true; pkill -9 -f 'gz sim' 2>/dev/null || true; pkill -9 -f '[g]z-sim' 2>/dev/null || true; pkill -9 -x gz 2>/dev/null || true; pkill -9 -f '[r]uby.*gz' 2>/dev/null || true; pkill -9 -f '[s]urvey_mission' 2>/dev/null || true; pkill -9 -f '[m]avsdk_server' 2>/dev/null || true" > /dev/null 2>&1 || true
    echo "[CLEANUP] Done. Clean state restored."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Start PX4 SITL with Gazebo Harmonic using NVIDIA GPU
echo "[LAUNCH] Starting PX4 SITL + Gazebo Harmonic (NVIDIA RTX 2050)..."
GZ_HEADLESS_FLAG=""
if [ "$HEADLESS" -eq 1 ]; then
    GZ_HEADLESS_FLAG="HEADLESS=1"
fi

distrobox-enter -n "$CONTAINER" < /dev/null -- bash -c "
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    export DISPLAY=\"${DISPLAY:-:0}\"
    export WAYLAND_DISPLAY=\"${WAYLAND_DISPLAY:-wayland-1}\"
    export XDG_RUNTIME_DIR=\"${XDG_RUNTIME_DIR:-/run/user/1000}\"
    export PX4_GZ_WORLD=box_survey
    export PX4_SIM_MODEL=x500_mono_cam_45
    cd $PX4_DIR && sleep infinity | $GZ_HEADLESS_FLAG make px4_sitl gz_x500_mono_cam_45
" &
PX4_PID=$!

# Wait for PX4 to initialize
echo "[MAVSDK] Waiting for PX4 Autopilot to boot..."
for i in {1..40}; do
    if distrobox-enter -n "$CONTAINER" -- bash -c "pgrep -x px4 > /dev/null"; then
        echo " -> PX4 Autopilot SITL is active!"
        break
    fi
    sleep 1
done

echo "[MAVSDK] Waiting for sensors and EKF2 state estimator convergence (8s)..."
sleep 8

# Launch MAVSDK Survey Mission, Tkinter GCS & Box Detector
echo "[MISSION] Launching 5-Lane High-Efficiency Survey ($SPEED m/s) with Continuous Dubins Turns..."
echo "[GCS] Dedicated Desktop Tkinter GCS active (Video + Telemetry + 2D Arena Map)!"
echo "[RECORDER] Recording full mission video to: detections/survey_mission_recording.mp4"
VISION_HEADLESS_ARG=""
if [ "$HEADLESS" -eq 1 ]; then
    VISION_HEADLESS_ARG="--headless"
fi

distrobox-enter -n "$CONTAINER" -- bash -c "
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
    export DISPLAY=\"${DISPLAY:-:0}\"
    export WAYLAND_DISPLAY=\"${WAYLAND_DISPLAY:-wayland-1}\"
    export XDG_RUNTIME_DIR=\"${XDG_RUNTIME_DIR:-/run/user/1000}\"
    export QT_QPA_PLATFORM=xcb
    export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
    cd $PROJECT_DIR/src && python3 survey_mission.py --speed $SPEED $VISION_HEADLESS_ARG
"

echo "[MISSION] Autonomous survey mission completed successfully."
cleanup
