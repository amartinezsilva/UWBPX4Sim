#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cleanup_running=0

detect_default_ros_ws() {
  local candidate

  for candidate in "$SCRIPT_DIR/../.." "$SCRIPT_DIR/../../.."; do
    candidate="$(cd "$candidate" && pwd)"
    if [[ -d "$candidate/src" && -d "$candidate/install" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  for candidate in "$SCRIPT_DIR/../.." "$SCRIPT_DIR/../../.."; do
    candidate="$(cd "$candidate" && pwd)"
    if [[ -d "$candidate/src" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  cd "$SCRIPT_DIR/../.." && pwd
}

get_tmux_size() {
  local cols lines

  cols="$(tput cols 2>/dev/null || printf '240')"
  lines="$(tput lines 2>/dev/null || printf '70')"

  if [[ -z "$cols" || "$cols" -lt 160 ]]; then
    cols=240
  fi
  if [[ -z "$lines" || "$lines" -lt 50 ]]; then
    lines=70
  fi

  printf '%s %s\n' "$cols" "$lines"
}

create_window_with_panes() {
  local target="$1"
  local pane_count="$2"
  local pane_idx

  # Re-tile after every split: splitting the active pane repeatedly halves
  # it each time, so with many vehicles tmux runs out of room ("no space
  # for new pane") before a single tiling at the end would help.
  for ((pane_idx = 1; pane_idx < pane_count; pane_idx++)); do
    tmux split-window -t "$target"
    tmux select-layout -t "$target" tiled
  done

  tmux select-layout -t "$target" tiled
}

DEFAULT_ROS_WS="$(detect_default_ros_ws)"

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ROS_WS="${ROS_WS:-$DEFAULT_ROS_WS}"
UWB_LAYOUT_FILE="${UWB_LAYOUT_FILE:-$SCRIPT_DIR/config/demo_nlos.yaml}"
LAYOUT_TOOL="$SCRIPT_DIR/tools/configure_uwb_layout.py"

# GZ_WORLD, unless already set (e.g. by setup_simulator.sh's .setup_env, or
# by hand), defaults to the layout's own `world:` field -- resolved by
# name against PX4's worlds directory, the same way setup_simulator.sh's
# own world step does. That way running this script standalone (without
# having just run setup in the same shell) still honors a world named
# directly in the layout, not just PX4's default one.
if [[ -z "${GZ_WORLD:-}" && -f "$UWB_LAYOUT_FILE" ]]; then
  GZ_WORLD="$(python3 "$LAYOUT_TOOL" --layout "$UWB_LAYOUT_FILE" --emit-world 2>/dev/null || true)"
  if [[ -n "$GZ_WORLD" && ! -f "$PX4_DIR/Tools/simulation/gz/worlds/$GZ_WORLD.sdf" ]]; then
    echo "[WARN] Layout names world '$GZ_WORLD', but it isn't installed under $PX4_DIR/Tools/simulation/gz/worlds -- using PX4's default world. Re-run setup_simulator.sh to install it." >&2
    GZ_WORLD=""
  fi
fi
GZ_WORLD="${GZ_WORLD:-default}"

print_usage() {
  cat <<EOF2
Usage: $(basename "$0")

Options:
  (env) UWB_LAYOUT_FILE  Layout YAML with per-robot spawn_pose and sensor placement (default: config/demo_nlos.yaml)
  (env) GZ_WORLD  Gazebo world name from PX4 Tools/simulation/gz/worlds (default: the
                       layout's own 'world:' field if it names one installed there, else "default")
  --no-qgc             Don't require or start QGroundControl. PX4 then needs NAV_DLL_ACT=0 to
                       arm without a ground station, which this sets (as a runtime default,
                       not saved) on every instance once it has booted.
  --headless           Run Gazebo without its GUI (sets HEADLESS=1 for PX4's gz launcher).
  --detach             Don't attach to the tmux session, and leave the simulation running when
                       this script exits; stop it with 'tmux kill-session -t \$SESSION'. For
                       unattended runs.
  -h, --help           Show this help message.
EOF2
}

USE_QGC=1
HEADLESS_GZ=0
DETACH=0

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --no-qgc) USE_QGC=0 ;;
      --headless) HEADLESS_GZ=1 ;;
      --detach) DETACH=1 ;;
      -h|--help)
        print_usage
        exit 0
        ;;
      *)
        echo "[ERROR] Unknown argument: $1" >&2
        print_usage >&2
        exit 1
        ;;
    esac
    shift
  done
}

find_qgc() {
  local candidate

  if [[ -n "${QGC_PATH:-}" ]]; then
    printf '%s\n' "$QGC_PATH"
    return 0
  fi

  for candidate in "$HOME/Desktop"/QGroundControl*.AppImage "$HOME/Downloads"/QGroundControl*.AppImage; do
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "[ERROR] Missing required command: $1" >&2
    exit 1
  fi
}

require_directory() {
  if [[ ! -d "$1" ]]; then
    echo "[ERROR] Directory not found: $1" >&2
    exit 1
  fi
}

build_ros_shell_command() {
  local ros_command="$1"
  local wrapped

  printf -v wrapped "%q -lc %q" "$ROS_SHELL" "source \"$ROS_SETUP_SCRIPT\" && $ros_command"
  printf '%s' "$wrapped"
}

QGC_PATH="$(find_qgc || true)"
parse_args "$@"

if (( USE_QGC )) && [[ -z "$QGC_PATH" ]]; then
  echo "[ERROR] QGroundControl AppImage not found. Set QGC_PATH or place it in ~/Desktop or ~/Downloads (or pass --no-qgc)." >&2
  exit 1
fi

require_directory "$ROS_WS"
require_directory "$PX4_DIR"
if [[ ! -f "$UWB_LAYOUT_FILE" ]]; then
  echo "[ERROR] Layout YAML not found: $UWB_LAYOUT_FILE" >&2
  exit 1
fi
require_command tmux
require_command MicroXRCEAgent
require_command python3

if [[ -f "$ROS_WS/install/setup.bash" ]]; then
  ROS_SHELL="bash"
  ROS_SETUP_SCRIPT="$ROS_WS/install/setup.bash"
elif [[ -f "$ROS_WS/install/setup.zsh" ]]; then
  ROS_SHELL="zsh"
  ROS_SETUP_SCRIPT="$ROS_WS/install/setup.zsh"
  require_command zsh
else
  echo "[ERROR] Could not find a ROS setup script under $ROS_WS/install." >&2
  exit 1
fi

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [[ ! -x "$PX4_BIN" ]]; then
  echo "[ERROR] PX4 SITL binary not found at $PX4_BIN. Build PX4 with 'make px4_sitl' first." >&2
  exit 1
fi

if [[ ! -f "$LAYOUT_TOOL" ]]; then
  echo "[ERROR] Layout tool not found: $LAYOUT_TOOL" >&2
  exit 1
fi

mapfile -t SPAWN_SPECS < <(python3 "$LAYOUT_TOOL" --layout "$UWB_LAYOUT_FILE" --emit-spawn-layout)
if (( ${#SPAWN_SPECS[@]} == 0 )); then
  echo "[ERROR] No vehicles found in layout: $UWB_LAYOUT_FILE" >&2
  exit 1
fi

PX4_GZ_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
for spec in "${SPAWN_SPECS[@]}"; do
  IFS=$'\t' read -r _vehicle_type _vehicle_id model_name _autostart _instance_index _x _y _z _roll _pitch _yaw <<< "$spec"
  if [[ ! -d "$PX4_GZ_MODELS_DIR/$model_name" ]]; then
    echo "[ERROR] Missing generated Gazebo model: $PX4_GZ_MODELS_DIR/$model_name" >&2
    echo "[ERROR] Copy the generated models from UWBPX4Sim_repo/models/custom_models into PX4 first." >&2
    exit 1
  fi
done

# Static anchors are a plain Gazebo model: no PX4 instance spawns them.
STATIC_ANCHORS_MODEL="$(python3 "$LAYOUT_TOOL" --layout "$UWB_LAYOUT_FILE" --emit-static-anchors-model)"
if [[ -n "$STATIC_ANCHORS_MODEL" && ! -d "$PX4_GZ_MODELS_DIR/$STATIC_ANCHORS_MODEL" ]]; then
  echo "[ERROR] Missing generated Gazebo model: $PX4_GZ_MODELS_DIR/$STATIC_ANCHORS_MODEL" >&2
  echo "[ERROR] Re-run setup_simulator.sh to generate and install it." >&2
  exit 1
fi

SESSION="${SESSION:-sim_session}"
UWB_BRIDGE_CONFIG="${UWB_BRIDGE_CONFIG:-$SCRIPT_DIR/ROS2/px4_sim_offboard/config/uwb_bridge.yaml}"

if [[ ! -f "$UWB_BRIDGE_CONFIG" ]]; then
  echo "[ERROR] Bridge config not found: $UWB_BRIDGE_CONFIG" >&2
  echo "[ERROR] Generate it first with 'python3 tools/configure_uwb_layout.py --layout \"$UWB_LAYOUT_FILE\"'." >&2
  exit 1
fi

UWB_PAIR_TOPICS=()
mapfile -t UWB_PAIR_TOPICS < <(
  awk -F'"' '/ros_topic_name:/ {print $2}' "$UWB_BRIDGE_CONFIG"
)

graceful_shutdown() {
  local pane_id

  if (( cleanup_running )); then
    return
  fi
  cleanup_running=1
  trap - EXIT INT TERM

  echo "[CLEANUP] Graceful shutdown…"

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    while IFS= read -r pane_id; do
      tmux send-keys -t "$pane_id" C-c 2>/dev/null || true
    done < <(tmux list-panes -a -t "$SESSION" -F "#{session_name}:#{window_index}.#{pane_index}")
    sleep 3

    echo "[CLEANUP] Killing simulators…"
    killall -9 gzserver gzclient ruby MicroXRCEAgent px4 2>/dev/null || true
    # killall isn't installed everywhere (e.g. this repo's Docker image), and
    # without it the Gazebo server ("gz sim", a ruby process) survives and the
    # next launch attaches to its stale world. pkill covers that case.
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -x px4 2>/dev/null || true
    pkill -9 -x MicroXRCEAgent 2>/dev/null || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
  fi
}

# Detached runs leave the session up when this script exits; the caller
# stops it (graceful_shutdown would otherwise kill it straight away).
if (( ! DETACH )); then
  trap graceful_shutdown EXIT INT TERM
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true

read -r TMUX_COLS TMUX_LINES < <(get_tmux_size)
tmux new-session -d -x "$TMUX_COLS" -y "$TMUX_LINES" -s "$SESSION" -n tools
create_window_with_panes "$SESSION:0" 2

if (( USE_QGC )); then
  tmux send-keys -t "$SESSION:0.0" "chmod +x \"$QGC_PATH\" && \"$QGC_PATH\"" C-m
fi
tmux send-keys -t "$SESSION:0.1" "MicroXRCEAgent udp4 -p 8888" C-m

HEADLESS_ENV=""
if (( HEADLESS_GZ )); then
  HEADLESS_ENV="HEADLESS=1 "
fi

# Without a ground station, PX4 refuses to arm while NAV_DLL_ACT > 0 ("No
# connection to the GCS"; commander's rcAndDataLinkCheck.cpp). A
# PX4_PARAM_NAV_DLL_ACT=0 env override doesn't work: rcS applies it before the
# airframe (4001_gz_x500 runs `param set-default NAV_DLL_ACT 2`), and since 0
# equals the firmware default nothing is stored, so the airframe's 2 wins.
# Instead, once each instance has finished booting (its uXRCE-DDS client,
# started at the end of rcS, answers), lower the default at runtime. Defaults
# aren't saved to parameters.bson, so later runs with QGC are unaffected.
nav_dll_cmd() {
  local px4_bin_dir
  px4_bin_dir="$(dirname "$PX4_BIN")"
  local cmd="echo '[INFO] QGroundControl disabled (--no-qgc): setting NAV_DLL_ACT=0 on each PX4 instance once booted.'"
  local i
  for i in "$@"; do
    cmd+=" && until \"$px4_bin_dir/px4-uxrce_dds_client\" --instance $i status >/dev/null 2>&1; do sleep 1; done"
    cmd+=" && \"$px4_bin_dir/px4-param\" --instance $i set-default NAV_DLL_ACT 0 >/dev/null && echo '[INFO]   instance $i: NAV_DLL_ACT=0'"
  done
  printf '%s' "$cmd"
}

# Retries until Gazebo answers. `gz service` exits 0 even when the call times
# out, so success is read from the reply rather than the exit code.
static_anchors_cmd() {
  local req="sdf_filename: \\\"$PX4_GZ_MODELS_DIR/$STATIC_ANCHORS_MODEL/model.sdf\\\", name: \\\"$STATIC_ANCHORS_MODEL\\\", allow_renaming: false"
  printf '%s' "echo '[INFO] Waiting for Gazebo to spawn the static anchors...' && until gz service -s /world/${GZ_WORLD}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req \"$req\" 2>/dev/null | grep -q 'data: true'; do sleep 1; done && echo '[INFO] Static anchors spawned.'"
}

tmux new-window -t "$SESSION:1" -n px4
create_window_with_panes "$SESSION:1" "${#SPAWN_SPECS[@]}"

SPAWNED_MODELS=()
PX4_INSTANCES=()
vehicle_idx=0
for spec in "${SPAWN_SPECS[@]}"; do
  IFS=$'\t' read -r vehicle_type vehicle_id model_name autostart instance_index x y z roll pitch yaw <<< "$spec"
  pane_index="$vehicle_idx"
  sleep_delay=$((2 + vehicle_idx * 2))
  pose_csv="${x},${y},${z},${roll},${pitch},${yaw}"
  SPAWNED_MODELS+=("$model_name")
  PX4_INSTANCES+=("$instance_index")

  if (( instance_index == 1 )); then
    spawn_cmd="sleep $sleep_delay && cd \"$PX4_DIR\" && ${HEADLESS_ENV}PX4_GZ_MODEL_POSE='$pose_csv' PX4_SYS_AUTOSTART=$autostart PX4_SIM_MODEL=gz_$model_name PX4_GZ_WORLD=${GZ_WORLD} \"$PX4_BIN\" -i $instance_index"
  else
    spawn_cmd="sleep $sleep_delay && cd \"$PX4_DIR\" && ${HEADLESS_ENV}PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_POSE='$pose_csv' PX4_SYS_AUTOSTART=$autostart PX4_SIM_MODEL=gz_$model_name PX4_GZ_WORLD=${GZ_WORLD} \"$PX4_BIN\" -i $instance_index"
  fi

  tmux send-keys -t "$SESSION:1.$pane_index" "$spawn_cmd" C-m
  vehicle_idx=$((vehicle_idx + 1))
done

if (( ! USE_QGC )); then
  tmux send-keys -t "$SESSION:0.0" "$(nav_dll_cmd "${PX4_INSTANCES[@]}")" C-m
fi

tmux new-window -t "$SESSION:2" -n ros2
create_window_with_panes "$SESSION:2" 3

tmux send-keys -t "$SESSION:2.0" "$(build_ros_shell_command "sleep 12 && cd \"$ROS_WS\" && PX4_SIM_OFFBOARD_BRIDGE_CONFIG=\"$UWB_BRIDGE_CONFIG\" ros2 launch px4_sim_offboard uwb_bridge_launch.py")" C-m
tmux send-keys -t "$SESSION:2.1" "$(build_ros_shell_command "sleep 25 && cd \"$ROS_WS\" && UWB_LAYOUT_FILE=\"$UWB_LAYOUT_FILE\" ros2 launch px4_sim_offboard offboard_launch.py")" C-m
spawned_models_str="${SPAWNED_MODELS[*]}"
tmux send-keys -t "$SESSION:2.2" "echo '[INFO] UWB launcher.' && echo '[INFO] Layout file: $UWB_LAYOUT_FILE' && echo '[INFO] Spawned models: $spawned_models_str' && echo '[INFO] Bridge topics should appear under /uwb_gz_simulator/distances/aItJ and /uwb_gz_simulator/distances_ground_truth/aItJ.' && echo '[INFO] Offboard nodes are configured directly from the layout YAML.'" C-m
if [[ -n "$STATIC_ANCHORS_MODEL" ]]; then
  tmux send-keys -t "$SESSION:2.2" "$(static_anchors_cmd)" C-m
fi

if (( DETACH )); then
  echo "[INFO] Simulation running detached in tmux session '$SESSION' (stop: tmux kill-session -t $SESSION)."
else
  tmux attach-session -t "$SESSION"
fi
