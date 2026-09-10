#!/bin/bash
#
# setup_simulator.sh - Guided setup for UWBPX4Sim.
#
# Automates the steps described in README.md section "2. Setting up the
# plugin in PX4 SITL":
#   0. (interactive only) Open the browser GUI (gui/app.py) to build the
#      layout and tune plugin parameters, unless --no-gui/--layout/-y is given
#   1. Generate per-robot models + the GZ/ROS2 bridge config from a layout YAML
#   2. Copy the generated models into <PX4-Autopilot>/Tools/simulation/gz/models
#   3. Copy uwb_gazebo_plugin/ into <PX4-Autopilot>/src/modules/simulation/gz_plugins
#   4. Register the plugin in gz_plugins/CMakeLists.txt
#   5. Register the plugin instance in gz_bridge/server.config, with its
#      parameter values synced from uwb_gazebo_plugin/params.yaml
#   6. Copy every custom world (worlds/*.sdf) into PX4, then select one by
#      name -- from the layout's own `world:` field, --world, or the
#      terminal picker -- falling back to PX4's default world if the name
#      isn't found there
#   7. (optional) Rebuild PX4 (make px4_sitl)
#   8. Check the ROS 2 workspace side (px4_sim_offboard, eliko_ros)
#
# All PX4 tree edits are idempotent: re-running this script is safe and will
# not duplicate entries or re-download/re-copy unchanged files.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ASSUME_YES=0
DRY_RUN=0
NO_GUI=0
DO_BUILD=""      # "", "yes", "no"
SKIP_MODELS=0
SKIP_PLUGIN=0
SKIP_ROS=0
WORLD_ARG=""

CONFIG_DIR="$SCRIPT_DIR/config"
GUI_PORT="${UWB_GUI_PORT:-5050}"

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

run_gui() {
  # Populates LAYOUT_FILE (and, if chosen there, WORLD_ARG) via the browser
  # GUI instead of the terminal pickers below. Returns 1 on anything short
  # of a clean finish (missing deps, port taken, GUI closed without
  # clicking "Continue Setup") so the caller can fall back to select_layout.
  local gui_dir="$SCRIPT_DIR/gui" selection_file gui_pid

  if [[ ! -f "$gui_dir/app.py" ]]; then
    log_warn "Configuration GUI not found under $gui_dir; using the terminal picker instead."
    return 1
  fi
  if ! python3 -c "import flask" >/dev/null 2>&1; then
    log_warn "Python module 'flask' not available; using the terminal picker instead."
    log_warn "(apt install python3-flask, or pip install flask, to enable the GUI)"
    return 1
  fi

  selection_file="$(mktemp)"

  echo
  log_info "Configuration GUI: http://localhost:$GUI_PORT"
  log_info "Open that in your browser to build a layout and tune plugin parameters,"
  log_info "then click 'Continue Setup' there. Ctrl-C here cancels the whole setup."

  python3 "$gui_dir/app.py" --uwb-root "$SCRIPT_DIR" --port "$GUI_PORT" --selection-file "$selection_file" &
  gui_pid=$!
  trap 'kill "$gui_pid" 2>/dev/null || true' EXIT INT TERM
  wait "$gui_pid" || true
  trap - EXIT INT TERM

  if [[ ! -s "$selection_file" ]]; then
    log_warn "GUI exited without finishing; using the terminal picker instead."
    rm -f "$selection_file"
    return 1
  fi

  # shellcheck disable=SC1090
  source "$selection_file"
  rm -f "$selection_file"
  log_ok "Got layout and settings from the GUI."
  return 0
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
    default="$CONFIG_DIR/demo_nlos.yaml"
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
                       If omitted, opens the browser GUI to build one (or lists
                       config/*.yaml and prompts you to pick one if the GUI isn't
                       available; auto-selects demo_nlos.yaml, or the
                       first found, under -y)
  --no-gui            Skip the browser GUI and go straight to the terminal
                       layout/world pickers
  --px4-dir DIR       PX4-Autopilot checkout (default: \$PX4_DIR or ~/PX4-Autopilot)
  --ros-ws DIR        ROS 2 workspace root (default: \$ROS_WS or auto-detected)
  --world NAME|PATH   World to use, by name -- either a custom one (worlds/*.sdf,
                       always copied into PX4 regardless of this flag) or one PX4
                       already ships (e.g. baylands); a full path to an .sdf file
                       also works. Falls back to PX4's default world if the name
                       isn't found there. Overrides the layout's own `world:`
                       field. If omitted (and the layout doesn't set `world:`
                       either), lists worlds/*.sdf and prompts you to pick one, or
                       none (under -y, keeps PX4's default world unless a world is
                       named by one of those two)
  --build             Rebuild PX4 (make px4_sitl) after patching, without asking
  --no-build          Skip the PX4 rebuild step, without asking
  --skip-models       Skip layout generation + model copy step
  --skip-plugin       Skip plugin copy + CMakeLists.txt/server.config patch step
  --skip-ros          Skip the ROS 2 workspace checks
  -y, --yes           Non-interactive: answer every prompt with its default
                       (skips the GUI, the optional world install, and PX4/colcon
                       rebuilds unless --build/--world are also given)
  -n, --dry-run       Print what would happen without changing anything
  -h, --help          Show this help message

Environment overrides: UWB_LAYOUT_FILE, PX4_DIR, ROS_WS, UWB_GUI_PORT (default 5050)
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
    --no-gui) NO_GUI=1; shift ;;
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
# outright; otherwise, when running interactively, try the browser GUI
# first (it can also set WORLD_ARG) and fall back to the terminal picker if
# it's unavailable or unfinished; under -y/--dry-run/--no-gui, go straight
# to the terminal picker. Errors out if no layout is ever resolved.
if [[ -n "$LAYOUT_FILE" ]]; then
  if [[ ! -f "$LAYOUT_FILE" ]]; then
    log_error "Layout YAML not found: $LAYOUT_FILE"
    exit 1
  fi
else
  layout_from_gui=0
  if (( ! NO_GUI && ! ASSUME_YES && ! DRY_RUN )) && [[ -t 0 ]] && run_gui; then
    layout_from_gui=1
  fi
  (( layout_from_gui )) || select_layout
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
# 3. Gazebo world
# ---------------------------------------------------------------------------

log_step "Step 3/6: Gazebo world"

if [[ ! -d "$PX4_GZ_WORLDS_DIR" ]]; then
  log_error "PX4 worlds directory not found: $PX4_GZ_WORLDS_DIR (check --px4-dir)"
  exit 1
fi

# Every custom world under worlds/ is copied into PX4 unconditionally --
# not just whichever one ends up selected -- so a layout's `world:` field
# (or --world) can name any of them, exactly like naming one of PX4's own
# built-in worlds: both are resolved the same way below, by looking for
# <name>.sdf directly in PX4's worlds directory once this copy has run.
shopt -s nullglob
custom_worlds=("$SCRIPT_DIR"/worlds/*.sdf)
shopt -u nullglob
if (( ${#custom_worlds[@]} > 0 )); then
  for w in "${custom_worlds[@]}"; do
    if (( DRY_RUN )); then
      log_info "[DRY-RUN] cp \"$w\" \"$PX4_GZ_WORLDS_DIR/$(basename "$w")\""
    else
      cp -f "$w" "$PX4_GZ_WORLDS_DIR/$(basename "$w")"
    fi
  done
  (( DRY_RUN )) || log_ok "Copied ${#custom_worlds[@]} custom world(s) -> $PX4_GZ_WORLDS_DIR/"
fi

# --world can also be given a direct path to an .sdf file that isn't under
# worlds/ at all -- copy it in under its own basename too, so it's found
# by the same name-based lookup below as everything else.
if [[ -n "$WORLD_ARG" && -f "$WORLD_ARG" ]]; then
  if (( DRY_RUN )); then
    log_info "[DRY-RUN] cp \"$WORLD_ARG\" \"$PX4_GZ_WORLDS_DIR/$(basename "$WORLD_ARG")\""
  else
    cp -f "$WORLD_ARG" "$PX4_GZ_WORLDS_DIR/$(basename "$WORLD_ARG")"
  fi
fi

# Precedence: --world flag > the layout's own `world:` field > (interactive
# fallback, when neither is set) the terminal picker > PX4's default world.
if [[ -z "$WORLD_ARG" ]]; then
  WORLD_ARG="$(python3 "$SCRIPT_DIR/tools/configure_uwb_layout.py" --layout "$LAYOUT_FILE" --emit-world)"
fi
if [[ -z "$WORLD_ARG" ]] && [[ -t 0 ]] && (( ! ASSUME_YES && ! DRY_RUN )); then
  select_world
fi

if [[ -n "$WORLD_ARG" ]]; then
  world_name="$(basename "$WORLD_ARG")"
  world_name="${world_name%.sdf}"
  if [[ -f "$PX4_GZ_WORLDS_DIR/$world_name.sdf" ]]; then
    WORLD_ARG="$world_name"
    log_ok "Using world '$world_name' ($PX4_GZ_WORLDS_DIR/$world_name.sdf)."
    log_info "Run with: export GZ_WORLD=$world_name"
  else
    log_warn "World '$world_name' not found in $PX4_GZ_WORLDS_DIR -- using PX4's default world instead."
    WORLD_ARG=""
  fi
else
  log_info "Using PX4's default world."
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

  # BUILD_TESTING=OFF works around a known PX4 build failure ("Unknown CMake
  # command link_fuzztest") that happens whenever ROS 2's ament_cmake is on
  # the CMake search path (true for both the Docker image and any native
  # setup with ROS 2 sourced): ament_cmake_test's own `option(BUILD_TESTING
  # ... ON)` claims that cache variable before PX4's CMAKE_TESTING gate ever
  # runs, silently pulling in PX4's test tree -- including its incomplete
  # fuzztest CMake integration -- on a plain `make px4_sitl`. Passing
  # -DBUILD_TESTING=OFF on the initial cmake invocation pre-empts that.
  if (( should_build )); then
    if (( DRY_RUN )); then
      log_info "[DRY-RUN] (cd \"$PX4_DIR\" && CMAKE_ARGS=\"-DBUILD_TESTING=OFF\" make px4_sitl)"
    else
      ( cd "$PX4_DIR" && CMAKE_ARGS="${CMAKE_ARGS:-} -DBUILD_TESTING=OFF" make px4_sitl )
      log_ok "PX4 rebuilt"
    fi
  else
    log_info "Skipped. Rebuild later with: cd $PX4_DIR && CMAKE_ARGS=\"-DBUILD_TESTING=OFF\" make px4_sitl"
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

# This script cannot export these into whatever shell invoked it -- a child
# process can never modify its parent's environment, that's a hard Unix
# constraint, not an oversight. Instead, write the chosen settings to a
# sourceable file: callers that stay alive across both this script and
# simulator_launcher.sh (like docker/entrypoint.sh's menu loop) can source
# it themselves to pick them up automatically; everyone else can still just
# copy the export lines printed below, or `source .setup_env` by hand.
SETUP_ENV_FILE="$SCRIPT_DIR/.setup_env"
if (( DRY_RUN )); then
  log_info "[DRY-RUN] would write chosen settings to $SETUP_ENV_FILE"
else
  {
    printf 'export UWB_LAYOUT_FILE=%q\n' "$LAYOUT_FILE"
    [[ -n "$WORLD_ARG" ]] && printf 'export GZ_WORLD=%q\n' "${world_name%.sdf}"
    printf 'export PX4_DIR=%q\n' "$PX4_DIR"
    [[ -n "$ROS_WS" ]] && printf 'export ROS_WS=%q\n' "$ROS_WS"
  } > "$SETUP_ENV_FILE"
  log_ok "Wrote $SETUP_ENV_FILE"
fi

echo "Next steps:"
echo "  1. Source your ROS 2 workspace:   source ${ROS_WS:-<ros_ws>}/install/setup.bash"
echo "  2. Launch the simulation:"
echo "       source \"$SETUP_ENV_FILE\"   # sets the exports below in one go"
echo "       export UWB_LAYOUT_FILE=\"$LAYOUT_FILE\""
[[ -n "$WORLD_ARG" ]] && echo "       export GZ_WORLD=\"${world_name%.sdf}\""
echo "       export PX4_DIR=\"$PX4_DIR\""
[[ -n "$ROS_WS" ]] && echo "       export ROS_WS=\"$ROS_WS\""
echo "       $SCRIPT_DIR/simulator_launcher.sh"
echo
echo "Re-run this script any time you change the layout YAML, add/remove robots,"
echo "or want to re-sync the plugin into a freshly cloned PX4 checkout."
