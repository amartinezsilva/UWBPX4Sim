#!/bin/bash
#
# setup_simulator.sh - Guided setup for UWBPX4Sim.
#
# Automates the steps described in README.md section "2. Setting up the
# plugin in PX4 SITL":
#   1. Generate per-robot models + the GZ/ROS2 bridge config from a layout YAML
#   2. Copy the generated models into <PX4-Autopilot>/Tools/simulation/gz/models
#   3. Copy uwb_gazebo_plugin/ into <PX4-Autopilot>/src/modules/simulation/gz_plugins
#   4. Register the plugin in gz_plugins/CMakeLists.txt
#   5. Register the plugin instance in gz_bridge/server.config, with its
#      parameter values synced from uwb_gazebo_plugin/params.yaml
#   6. (optional) Copy a custom world (e.g. worlds/walls_nlos.sdf)
#   7. (optional) Rebuild PX4 (make px4_sitl)
#   8. Check the ROS 2 workspace side (px4_sim_offboard, eliko_ros)
#
# All PX4 tree edits are idempotent: re-running this script is safe and will
# not duplicate entries or re-download/re-copy unchanged files.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ASSUME_YES=0
DRY_RUN=0
DO_BUILD=""      # "", "yes", "no"
SKIP_MODELS=0
SKIP_PLUGIN=0
SKIP_ROS=0
WORLD_ARG=""

CONFIG_DIR="$SCRIPT_DIR/config"

LAYOUT_FILE="${UWB_LAYOUT_FILE:-}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ROS_WS="${ROS_WS:-}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

log_step()  { printf '\n==> %s\n' "$1"; }
log_info()  { printf '[INFO] %s\n' "$1"; }
log_warn()  { printf '[WARN] %s\n' "$1" >&2; }
log_error() { printf '[ERROR] %s\n' "$1" >&2; }
log_ok()    { printf '[OK] %s\n' "$1"; }

confirm() {
  # confirm "question" [default: y|n]
  # Under -y/--yes, answers with the prompt's own default rather than a
  # blanket "yes" -- so slow/optional steps (declared default "n") stay
  # opt-in even in non-interactive mode, unless requested via an explicit flag.
  local prompt="$1" default="${2:-n}" reply
  if (( ASSUME_YES )); then
    [[ "$default" == "y" ]]
    return
  fi
  if [[ "$default" == "y" ]]; then
    read -r -p "$prompt [Y/n] " reply || reply=""
    [[ -z "$reply" || "$reply" =~ ^[Yy]$ ]]
  else
    read -r -p "$prompt [y/N] " reply || reply=""
    [[ "$reply" =~ ^[Yy]$ ]]
  fi
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log_error "Missing required command: $1"
    exit 1
  fi
}

detect_default_ros_ws() {
  local candidate
  for candidate in "$SCRIPT_DIR/../.." "$SCRIPT_DIR/../../.."; do
    [[ -d "$candidate" ]] || continue
    candidate="$(cd "$candidate" && pwd)"
    if [[ -d "$candidate/src" && -d "$candidate/install" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  for candidate in "$SCRIPT_DIR/../.." "$SCRIPT_DIR/../../.."; do
    [[ -d "$candidate" ]] || continue
    candidate="$(cd "$candidate" && pwd)"
    if [[ -d "$candidate/src" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

is_subpath() {
  # is_subpath <child> <parent>
  local child parent
  child="$(cd "$1" 2>/dev/null && pwd)" || return 1
  parent="$(cd "$2" 2>/dev/null && pwd)" || return 1
  [[ "$child" == "$parent" || "$child" == "$parent"/* ]]
}

select_world() {
  # Populates WORLD_ARG from worlds/*.sdf, or leaves it empty (use PX4's
  # default world). Unlike layouts, a world is optional: an empty worlds/
  # directory is not an error, it just means there is nothing to offer, and
  # a "None" choice is always on the menu.
  local worlds=() reply i names

  shopt -s nullglob
  worlds=("$SCRIPT_DIR"/worlds/*.sdf)
  shopt -u nullglob
  if (( ${#worlds[@]} > 0 )); then
    mapfile -t worlds < <(printf '%s\n' "${worlds[@]}" | sort -u)
  fi

  if (( ${#worlds[@]} == 0 )); then
    log_warn "No custom world files found under $SCRIPT_DIR/worlds; using PX4's default world."
    return
  fi

  if (( ASSUME_YES )); then
    names="$(IFS=,; i="${worlds[*]##*/}"; printf '%s' "$i")"
    log_info "Non-interactive: keeping PX4's default world (pass --world to use one of: $names)"
    return
  fi

  echo
  echo "Available custom worlds in $SCRIPT_DIR/worlds:"
  echo "  0) None (use PX4's default world)"
  for i in "${!worlds[@]}"; do
    printf '  %d) %s\n' "$((i + 1))" "$(basename "${worlds[$i]}")"
  done

  while true; do
    read -r -p "Select a world [0-${#worlds[@]}]: " reply || { log_error "No input received."; exit 1; }
    if [[ "$reply" =~ ^[0-9]+$ ]] && (( reply >= 0 && reply <= ${#worlds[@]} )); then
      if (( reply > 0 )); then
        WORLD_ARG="${worlds[$((reply - 1))]}"
      fi
      return
    fi
    echo "Invalid selection: '$reply'. Enter a number between 0 and ${#worlds[@]}."
  done
}

select_layout() {
  # Populates LAYOUT_FILE, either by listing config/*.yaml for the user to
  # pick from, or (under -y) by auto-selecting a sensible default.
  local layouts=() reply i default

  shopt -s nullglob
  layouts=("$CONFIG_DIR"/*.yaml "$CONFIG_DIR"/*.yml)
  shopt -u nullglob
  if (( ${#layouts[@]} > 0 )); then
    mapfile -t layouts < <(printf '%s\n' "${layouts[@]}" | sort -u)
  fi

  if (( ${#layouts[@]} == 0 )); then
    log_error "No UWB layout YAML files found under $CONFIG_DIR"
    exit 1
  fi

  if (( ASSUME_YES )); then
    default="$CONFIG_DIR/uwb_layout.example.yaml"
    for i in "${!layouts[@]}"; do
      if [[ "${layouts[$i]}" == "$default" ]]; then
        LAYOUT_FILE="$default"
        log_info "Non-interactive: selected default layout $(basename "$LAYOUT_FILE")"
        return
      fi
    done
    LAYOUT_FILE="${layouts[0]}"
    log_info "Non-interactive: selected layout $(basename "$LAYOUT_FILE") (first available; no --layout given)"
    return
  fi

  echo
  echo "Available UWB layout files in $CONFIG_DIR:"
  for i in "${!layouts[@]}"; do
    printf '  %d) %s\n' "$((i + 1))" "$(basename "${layouts[$i]}")"
  done

  while true; do
    read -r -p "Select a layout [1-${#layouts[@]}]: " reply || { log_error "No input received."; exit 1; }
    if [[ "$reply" =~ ^[0-9]+$ ]] && (( reply >= 1 && reply <= ${#layouts[@]} )); then
      LAYOUT_FILE="${layouts[$((reply - 1))]}"
      return
    fi
    echo "Invalid selection: '$reply'. Enter a number between 1 and ${#layouts[@]}."
  done
}

print_usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Guided setup for the UWBPX4Sim plugin, models, and ROS 2 bridge.

Options:
  --layout FILE       Layout YAML to configure (also settable via UWB_LAYOUT_FILE).
                       If omitted, lists config/*.yaml and prompts you to pick one
                       (auto-selects uwb_layout.example.yaml, or the first found, under -y)
  --px4-dir DIR       PX4-Autopilot checkout (default: \$PX4_DIR or ~/PX4-Autopilot)
  --ros-ws DIR        ROS 2 workspace root (default: \$ROS_WS or auto-detected)
  --world NAME|PATH   Also install a custom world into PX4 (e.g. walls_nlos, or a full path).
                       If omitted, lists worlds/*.sdf and prompts you to pick one, or none
                       (under -y, keeps PX4's default world unless a world is named here)
  --build             Rebuild PX4 (make px4_sitl) after patching, without asking
  --no-build          Skip the PX4 rebuild step, without asking
  --skip-models       Skip layout generation + model copy step
  --skip-plugin       Skip plugin copy + CMakeLists.txt/server.config patch step
  --skip-ros          Skip the ROS 2 workspace checks
  -y, --yes           Non-interactive: answer every prompt with its default
                       (skips the optional world install and PX4/colcon
                       rebuilds unless --build/--world are also given)
  -n, --dry-run       Print what would happen without changing anything
  -h, --help          Show this help message

Environment overrides: UWB_LAYOUT_FILE, PX4_DIR, ROS_WS
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --layout) LAYOUT_FILE="$2"; shift 2 ;;
    --px4-dir) PX4_DIR="$2"; shift 2 ;;
    --ros-ws) ROS_WS="$2"; shift 2 ;;
    --world) WORLD_ARG="$2"; shift 2 ;;
    --build) DO_BUILD="yes"; shift ;;
    --no-build) DO_BUILD="no"; shift ;;
    --skip-models) SKIP_MODELS=1; shift ;;
    --skip-plugin) SKIP_PLUGIN=1; shift ;;
    --skip-ros) SKIP_ROS=1; shift ;;
    -y|--yes) ASSUME_YES=1; shift ;;
    -n|--dry-run) DRY_RUN=1; shift ;;
    -h|--help) print_usage; exit 0 ;;
    *) log_error "Unknown argument: $1"; print_usage >&2; exit 1 ;;
  esac
done

if [[ -z "$ROS_WS" ]]; then
  ROS_WS="$(detect_default_ros_ws || true)"
fi

# Resolve which layout YAML to use: an explicit --layout/UWB_LAYOUT_FILE wins
# outright; otherwise list config/*.yaml and let the user pick one (or, under
# -y, auto-select a sensible default). Errors out if none exist at all.
if [[ -n "$LAYOUT_FILE" ]]; then
  if [[ ! -f "$LAYOUT_FILE" ]]; then
    log_error "Layout YAML not found: $LAYOUT_FILE"
    exit 1
  fi
else
  select_layout
fi

echo "UWBPX4Sim guided setup"
echo "======================="
log_info "UWBPX4Sim root : $SCRIPT_DIR"
log_info "Layout file    : $LAYOUT_FILE"
log_info "PX4-Autopilot  : $PX4_DIR"
log_info "ROS 2 workspace: ${ROS_WS:-<not found>}"
(( DRY_RUN )) && log_warn "Dry-run mode: no files will be changed."

# ---------------------------------------------------------------------------
# 0. Preflight checks
# ---------------------------------------------------------------------------

log_step "Step 0/6: Checking prerequisites"

require_command python3
require_command rsync

if ! python3 -c "import yaml" >/dev/null 2>&1; then
  log_error "Python module 'pyyaml' is required (pip install pyyaml)."
  exit 1
fi
log_ok "python3 + pyyaml available"
log_ok "Layout file: $LAYOUT_FILE"

# These are computed unconditionally (plain string paths, no filesystem
# requirement yet) so later steps never trip on `set -u`, even when the
# model/plugin steps are skipped but --world or --build are still used.
PX4_GZ_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"
PX4_GZ_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
PX4_GZ_PLUGINS_DIR="$PX4_DIR/src/modules/simulation/gz_plugins"
PX4_SERVER_CONFIG="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"
PLUGIN_PARAMS_FILE="$SCRIPT_DIR/uwb_gazebo_plugin/params.yaml"

if (( ! SKIP_PLUGIN || ! SKIP_MODELS )); then
  if [[ ! -d "$PX4_DIR" ]]; then
    log_error "PX4-Autopilot directory not found: $PX4_DIR (override with --px4-dir)"
    exit 1
  fi
  for d in "$PX4_GZ_MODELS_DIR" "$PX4_GZ_PLUGINS_DIR"; do
    if [[ ! -d "$d" ]]; then
      log_error "This does not look like a PX4-Autopilot checkout (missing $d)."
      exit 1
    fi
  done
  if [[ ! -f "$PX4_SERVER_CONFIG" ]]; then
    log_error "server.config not found at $PX4_SERVER_CONFIG"
    exit 1
  fi
  log_ok "PX4-Autopilot checkout looks valid"
fi

if (( ! SKIP_PLUGIN )) && [[ ! -f "$PLUGIN_PARAMS_FILE" ]]; then
  log_error "Plugin params file not found: $PLUGIN_PARAMS_FILE"
  exit 1
fi

# ---------------------------------------------------------------------------
# 1. Generate models + bridge config from the layout YAML
# ---------------------------------------------------------------------------

if (( SKIP_MODELS )); then
  log_step "Step 1/6: Generating models + bridge config (skipped: --skip-models)"
else
  log_step "Step 1/6: Generating models + bridge config from $(basename "$LAYOUT_FILE")"
  if (( DRY_RUN )); then
    log_info "[DRY-RUN] python3 tools/configure_uwb_layout.py --layout \"$LAYOUT_FILE\" --uwb-root \"$SCRIPT_DIR\""
  else
    python3 "$SCRIPT_DIR/tools/configure_uwb_layout.py" --layout "$LAYOUT_FILE" --uwb-root "$SCRIPT_DIR"
    log_ok "Generated models/custom_models/ and ROS2/px4_sim_offboard/config/uwb_bridge.yaml"
  fi

  CUSTOM_MODELS_DIR="$SCRIPT_DIR/models/custom_models"
  if [[ -d "$CUSTOM_MODELS_DIR" ]]; then
    log_step "Copying generated models into PX4"
    shopt -s nullglob
    model_dirs=("$CUSTOM_MODELS_DIR"/*/)
    shopt -u nullglob
    if (( ${#model_dirs[@]} == 0 )); then
      log_warn "No generated model directories found under $CUSTOM_MODELS_DIR"
    fi
    for model_dir in "${model_dirs[@]}"; do
      model_name="$(basename "$model_dir")"
      dest="$PX4_GZ_MODELS_DIR/$model_name"
      if (( DRY_RUN )); then
        log_info "[DRY-RUN] rsync -a --delete \"$model_dir\" \"$dest/\""
      else
        mkdir -p "$dest"
        rsync -a --delete "$model_dir" "$dest/"
        log_ok "Copied $model_name -> $dest"
      fi
    done
  fi
fi

# ---------------------------------------------------------------------------
# 2. Copy the plugin and register it with PX4's build + server config
# ---------------------------------------------------------------------------

if (( SKIP_PLUGIN )); then
  log_step "Step 2/6: Installing uwb_gazebo_plugin into PX4 (skipped: --skip-plugin)"
else
  log_step "Step 2/6: Copying uwb_gazebo_plugin into PX4"
  PLUGIN_DEST="$PX4_GZ_PLUGINS_DIR/uwb_gazebo_plugin"
  if (( DRY_RUN )); then
    log_info "[DRY-RUN] rsync -a --delete --exclude COLCON_IGNORE --exclude params.yaml \"$SCRIPT_DIR/uwb_gazebo_plugin/\" \"$PLUGIN_DEST/\""
  else
    mkdir -p "$PLUGIN_DEST"
    rsync -a --delete --exclude COLCON_IGNORE --exclude params.yaml "$SCRIPT_DIR/uwb_gazebo_plugin/" "$PLUGIN_DEST/"
    log_ok "Copied uwb_gazebo_plugin -> $PLUGIN_DEST"
  fi

  log_step "Registering plugin in gz_plugins/CMakeLists.txt"
  CMAKE_FILE="$PX4_GZ_PLUGINS_DIR/CMakeLists.txt"
  if (( DRY_RUN )); then
    log_info "[DRY-RUN] would idempotently add add_subdirectory(uwb_gazebo_plugin) and 'UWBGazeboPlugin' to $CMAKE_FILE"
  else
    result="$(python3 - "$CMAKE_FILE" <<'PYEOF'
import os
import re
import shutil
import sys

path = sys.argv[1]
text = open(path).read()
changed = False

if "add_subdirectory(uwb_gazebo_plugin)" not in text:
    lines = text.splitlines(keepends=True)
    last_idx = None
    for i, line in enumerate(lines):
        if re.match(r"\s*add_subdirectory\(", line):
            last_idx = i
    if last_idx is None:
        print("ERROR: no add_subdirectory(...) line found to anchor insertion", file=sys.stderr)
        sys.exit(1)
    indent = re.match(r"(\s*)", lines[last_idx]).group(1)
    lines.insert(last_idx + 1, f"{indent}add_subdirectory(uwb_gazebo_plugin)\n")
    text = "".join(lines)
    changed = True

def add_dep(m):
    body = m.group(0)
    if "UWBGazeboPlugin" in body:
        return body
    return body[:-1].rstrip() + " UWBGazeboPlugin)"

new_text, n = re.subn(r"add_custom_target\(px4_gz_plugins[^)]*\)", add_dep, text)
if n == 0:
    print("WARNING: no add_custom_target(px4_gz_plugins ...) found in this CMakeLists.txt", file=sys.stderr)
elif new_text != text:
    text = new_text
    changed = True

if changed:
    if not os.path.exists(path + ".orig"):
        shutil.copy2(path, path + ".orig")
    open(path, "w").write(text)
    print("PATCHED")
else:
    print("ALREADY_OK")
PYEOF
)"
    if [[ "$result" == "PATCHED" ]]; then
      log_ok "CMakeLists.txt updated (backup: ${CMAKE_FILE}.orig)"
    else
      log_ok "CMakeLists.txt already registers the plugin"
    fi
  fi

  log_step "Syncing plugin parameters from params.yaml into gz_bridge/server.config"
  if (( DRY_RUN )); then
    log_info "[DRY-RUN] would sync $PLUGIN_PARAMS_FILE into the <plugin ... custom::UWBGazeboSystem> block in $PX4_SERVER_CONFIG"
  else
    result="$(python3 - "$PX4_SERVER_CONFIG" "$PLUGIN_PARAMS_FILE" <<'PYEOF'
import os
import re
import shutil
import sys

import yaml

PLUGIN_NAME = "custom::UWBGazeboSystem"
PLUGIN_FILENAME = "libUWBGazeboPlugin.so"

server_path, params_path = sys.argv[1], sys.argv[2]

with open(params_path) as f:
    params = yaml.safe_load(f) or {}

if not isinstance(params, dict):
    print(f"ERROR: {params_path} must contain a mapping of parameter: value", file=sys.stderr)
    sys.exit(1)


def fmt(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


block_lines = [
    f'    <plugin entity_name="*" entity_type="world" filename="{PLUGIN_FILENAME}" name="{PLUGIN_NAME}">'
]
for key, value in params.items():
    block_lines.append(f"      <{key}>{fmt(value)}</{key}>")
block_lines.append("    </plugin>")
new_block = "\n".join(block_lines) + "\n"

text = open(server_path).read()
pattern = re.compile(
    r'[ \t]*<plugin\b[^>]*name="' + re.escape(PLUGIN_NAME) + r'"[^>]*>.*?</plugin>\n?',
    re.DOTALL,
)
match = pattern.search(text)

if match:
    if match.group(0).rstrip("\n") == new_block.rstrip("\n"):
        print("ALREADY_OK")
        sys.exit(0)
    new_text = text[: match.start()] + new_block + text[match.end() :]
    action = "UPDATED"
else:
    idx = text.rfind("</plugins>")
    if idx == -1:
        print("ERROR: </plugins> closing tag not found", file=sys.stderr)
        sys.exit(1)
    # Insert at the start of the </plugins> line, not at the "<" itself --
    # otherwise that line's own leading indentation gets pulled in front of
    # our block instead of staying with </plugins>, corrupting indentation
    # and breaking the byte-exact comparison above on every later run.
    line_start = text.rfind("\n", 0, idx) + 1
    new_text = text[:line_start] + new_block + text[line_start:]
    action = "INSERTED"

if not os.path.exists(server_path + ".orig"):
    shutil.copy2(server_path, server_path + ".orig")
open(server_path, "w").write(new_text)
print(action)
PYEOF
)"
    case "$result" in
      INSERTED)
        log_ok "server.config updated (backup: ${PX4_SERVER_CONFIG}.orig)"
        log_info "Plugin parameters inserted from $(basename "$PLUGIN_PARAMS_FILE")."
        ;;
      UPDATED)
        log_ok "server.config updated (backup: ${PX4_SERVER_CONFIG}.orig)"
        log_info "Plugin parameters re-synced from $(basename "$PLUGIN_PARAMS_FILE")."
        log_warn "Any manual edits previously made directly to the plugin block in server.config were overwritten."
        ;;
      ALREADY_OK)
        log_ok "server.config parameters already match $(basename "$PLUGIN_PARAMS_FILE")"
        ;;
      *)
        log_error "Failed to sync plugin parameters into server.config"
        exit 1
        ;;
    esac
  fi
fi

# ---------------------------------------------------------------------------
# 3. Optional custom world
# ---------------------------------------------------------------------------

log_step "Step 3/6: Custom Gazebo world"
if [[ -z "$WORLD_ARG" ]]; then
  select_world
fi
if [[ -n "$WORLD_ARG" ]]; then
  if [[ ! -d "$PX4_GZ_WORLDS_DIR" ]]; then
    log_error "PX4 worlds directory not found: $PX4_GZ_WORLDS_DIR (check --px4-dir)"
    exit 1
  fi
  if [[ -f "$WORLD_ARG" ]]; then
    WORLD_SRC="$WORLD_ARG"
  elif [[ -f "$SCRIPT_DIR/worlds/$WORLD_ARG.sdf" ]]; then
    WORLD_SRC="$SCRIPT_DIR/worlds/$WORLD_ARG.sdf"
  elif [[ -f "$SCRIPT_DIR/worlds/$WORLD_ARG" ]]; then
    WORLD_SRC="$SCRIPT_DIR/worlds/$WORLD_ARG"
  else
    log_error "Could not resolve world '$WORLD_ARG' (looked for the path itself and worlds/$WORLD_ARG.sdf)"
    exit 1
  fi
  world_name="$(basename "$WORLD_SRC")"
  if (( DRY_RUN )); then
    log_info "[DRY-RUN] cp \"$WORLD_SRC\" \"$PX4_GZ_WORLDS_DIR/$world_name\""
  else
    cp -f "$WORLD_SRC" "$PX4_GZ_WORLDS_DIR/$world_name"
    log_ok "Copied $world_name -> $PX4_GZ_WORLDS_DIR/"
    log_info "Run with: export GZ_WORLD=${world_name%.sdf}"
  fi
else
  log_info "Skipping custom world (using PX4's default world)."
fi

# ---------------------------------------------------------------------------
# 4. Rebuild PX4
# ---------------------------------------------------------------------------

log_step "Step 4/6: Rebuild PX4 SITL"
if [[ -n "${PX4_DIR:-}" && -d "$PX4_DIR" ]]; then
  should_build=0
  if [[ "$DO_BUILD" == "yes" ]]; then
    should_build=1
  elif [[ "$DO_BUILD" == "no" ]]; then
    should_build=0
  elif confirm "Rebuild PX4 now with 'make px4_sitl'? This can take several minutes." n; then
    should_build=1
  fi

  if (( should_build )); then
    if (( DRY_RUN )); then
      log_info "[DRY-RUN] (cd \"$PX4_DIR\" && make px4_sitl)"
    else
      ( cd "$PX4_DIR" && make px4_sitl )
      log_ok "PX4 rebuilt"
    fi
  else
    log_info "Skipped. Rebuild later with: cd $PX4_DIR && make px4_sitl"
  fi
else
  log_info "Skipping (no PX4 directory to build)."
fi

# ---------------------------------------------------------------------------
# 5. ROS 2 workspace checks
# ---------------------------------------------------------------------------

if (( SKIP_ROS )); then
  log_step "Step 5/6: ROS 2 workspace checks (skipped: --skip-ros)"
else
  log_step "Step 5/6: Checking the ROS 2 workspace"
  if [[ -z "$ROS_WS" ]]; then
    log_warn "Could not auto-detect a ROS 2 workspace. Clone/host this repo under <ros_ws>/src and pass --ros-ws."
  elif ! is_subpath "$SCRIPT_DIR" "$ROS_WS/src"; then
    log_warn "UWBPX4Sim ($SCRIPT_DIR) is not under $ROS_WS/src; colcon will not find px4_sim_offboard there."
  else
    log_ok "UWBPX4Sim is inside $ROS_WS/src"

    if [[ ! -d "$ROS_WS/src/eliko_ros" ]] && ! find "$ROS_WS/src" -maxdepth 2 -iname "eliko_ros" -print -quit | grep -q .; then
      log_warn "eliko_ros not found under $ROS_WS/src. Clone it: git clone https://github.com/robotics-upo/eliko_ros.git $ROS_WS/src/eliko_ros"
    else
      log_ok "eliko_ros found"
    fi

    if [[ -d "$ROS_WS/install/px4_sim_offboard" ]]; then
      log_ok "px4_sim_offboard already built in $ROS_WS/install"
    else
      if confirm "Build px4_sim_offboard now with colcon?" y; then
        if (( DRY_RUN )); then
          log_info "[DRY-RUN] (cd \"$ROS_WS\" && colcon build --packages-up-to px4_sim_offboard)"
        else
          require_command colcon
          ( cd "$ROS_WS" && colcon build --packages-up-to px4_sim_offboard )
          log_ok "px4_sim_offboard built"
        fi
      else
        log_info "Skipped. Build later with: cd $ROS_WS && colcon build --packages-up-to px4_sim_offboard"
      fi
    fi

    built=0
    for setup in "$ROS_WS/install/setup.bash" "$ROS_WS/install/setup.zsh"; do
      [[ -f "$setup" ]] && built=1
    done
    (( built )) || log_warn "No install/setup.bash or setup.zsh found yet in $ROS_WS; source it after building."
  fi
fi

# ---------------------------------------------------------------------------
# 6. Summary
# ---------------------------------------------------------------------------

log_step "Step 6/6: Done"
echo
echo "Next steps:"
echo "  1. Source your ROS 2 workspace:   source ${ROS_WS:-<ros_ws>}/install/setup.bash"
echo "  2. Launch the simulation:"
echo "       export UWB_LAYOUT_FILE=\"$LAYOUT_FILE\""
[[ -n "$WORLD_ARG" ]] && echo "       export GZ_WORLD=\"${world_name%.sdf}\""
echo "       export PX4_DIR=\"$PX4_DIR\""
[[ -n "$ROS_WS" ]] && echo "       export ROS_WS=\"$ROS_WS\""
echo "       $SCRIPT_DIR/simulator_launcher.sh"
echo
echo "Re-run this script any time you change the layout YAML, add/remove robots,"
echo "or want to re-sync the plugin into a freshly cloned PX4 checkout."
