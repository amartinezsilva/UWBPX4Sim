#!/bin/bash
#
# entrypoint-lib.sh - Function library for UWBPX4Sim's guided container
# entrypoint. Sourced by entrypoint.sh (a thin wrapper that just calls
# uwbpx4sim_main "$@" after sourcing this), and also meant to be sourced by
# a *derived* image's own entrypoint.sh (mr-radio-localization's, for
# instance) that wants the same menu/setup/demo/exit machinery plus its
# own extra options -- it does that by sourcing this file, redefining
# print_menu/run_menu (bash resolves function calls by name at call time,
# so uwbpx4sim_main's internal `run_menu` call below will invoke whichever
# version is defined when it actually runs), and then calling
# uwbpx4sim_main itself. Nothing in this file executes anything on its
# own -- it only defines functions -- so sourcing it has no side effects
# beyond that.

# Colors/spinner only when writing to a real terminal, and never if the
# caller opted out via the NO_COLOR convention (https://no-color.org) --
# scripted/piped invocations (docker run <image> <cmd>, log capture, CI)
# must see plain, undecorated output.
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  USE_COLOR=1
  C_RESET=$'\033[0m'
  C_BOLD=$'\033[1m'
  C_DIM=$'\033[2m'
  C_CYAN=$'\033[36m'
  C_GREEN=$'\033[32m'
  C_YELLOW=$'\033[33m'
else
  USE_COLOR=""
  C_RESET="" C_BOLD="" C_DIM="" C_CYAN="" C_GREEN="" C_YELLOW=""
fi

# Log helpers keep the literal "[INFO]"/"[WARN]"/"[OK]" tags uncolored and
# contiguous (color wraps the whole line from outside), so anything that
# greps for those tags -- a human skimming, or a test harness -- still
# matches regardless of whether color is on.
log_info() { printf '%s[INFO] %s%s\n' "$C_CYAN" "$1" "$C_RESET"; }
log_warn() { printf '%s[WARN] %s%s\n' "$C_YELLOW" "$1" "$C_RESET" >&2; }
log_ok()   { printf '%s[OK] %s%s\n' "$C_GREEN" "$1" "$C_RESET"; }

# A spinner driven by the real sourcing work below, not a fake delay: it
# runs concurrently in the background for exactly as long as that work
# takes (near-instant most of the time, longer on a cold/slow disk) and
# gets killed the moment it's done. Sourcing itself must still happen in
# the foreground/current shell -- a background job's environment changes
# never propagate back to its parent, so the spinner can only wrap it, not
# replace it.
SPINNER_PID=""

spinner_start() {
  local msg="$1..."
  if [[ -z "$USE_COLOR" ]]; then
    printf '%s\n' "$msg"
    return
  fi
  {
    local frames='⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏' i=0
    while :; do
      printf '\r%s %s%s%s' "${frames:i%${#frames}:1}" "$C_CYAN" "$msg" "$C_RESET"
      i=$((i + 1))
      sleep 0.08
    done
  } &
  SPINNER_PID=$!
}

spinner_stop() {
  if [[ -n "$SPINNER_PID" ]]; then
    kill "$SPINNER_PID" 2>/dev/null || true
    wait "$SPINNER_PID" 2>/dev/null || true
    printf '\r\033[K'
    SPINNER_PID=""
  fi
}

spinner_start "Preparing environment"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
spinner_stop
log_ok "Environment ready."

# Top-level under the workspace (see the Dockerfile's CACHEBUST comment) --
# not nested under any other repo, so a derived image can add its own
# packages as siblings under ros2_ws/src without touching this path.
UWBPX4SIM_DIR="$HOME/ros2_ws/src/UWBPX4Sim"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

print_welcome() {
  cat <<EOF

${C_CYAN}${C_BOLD}==================================================
  UWBPX4Sim -- SITL dev container
==================================================${C_RESET}

PX4 SITL, Gazebo Harmonic, and the UWB Gazebo plugin/ROS 2 bridge,
pre-built and ready to go.

${C_DIM}Tip: UWBPX4Sim/config, uwb_gazebo_plugin, and worlds are bind-mounted
from the host -- edit a layout, params.yaml, or drop in a custom .sdf
world there, then re-run setup (option 1) to apply it.

Tip: for QGroundControl/Gazebo windows to appear, run on the HOST
first: xhost +local:docker (see docker-compose.yml's comments for the
extra step Wayland/XWayland desktops need).

Tip: this container keeps running in the background even if you close
this terminal -- run setup.sh again (or docker exec -it <container>
/entrypoint.sh) any time to get back to this menu with everything you
configured still in place. Pick "Exit" below when you actually want to
stop it.${C_RESET}
EOF
}

print_menu() {
  # Tick shown next to "Launch simulation" once setup_simulator.sh has run
  # and written its chosen layout/world/PX4_DIR to .setup_env -- lets the
  # user tell at a glance, before picking option 2, whether a setup is
  # already loaded or still needs to be run first. Checked fresh on every
  # call (not cached) so it reflects setup having *just* finished too.
  local setup_tick=""
  if [[ -f "$UWBPX4SIM_DIR/.setup_env" ]]; then
    setup_tick=" ${C_GREEN}✓ configured${C_RESET}"
  fi
  cat <<EOF

${C_CYAN}==================================================${C_RESET}
 ${C_BOLD}UWBPX4Sim${C_RESET} dev container
${C_CYAN}==================================================${C_RESET}
  ${C_GREEN}1)${C_RESET} Run UWBPX4Sim setup (setup_simulator.sh)
     Configure the PX4/Gazebo plugin, models, and layout.
  ${C_GREEN}2)${C_RESET} Launch simulation${setup_tick}
     Launch PX4 SITL + Gazebo + the UWB bridge/offboard nodes.
  ${C_GREEN}3)${C_RESET} Open a shell
  ${C_YELLOW}4)${C_RESET} Exit
${C_CYAN}==================================================${C_RESET}
EOF
}

run_setup() {
  if [[ ! -f "$UWBPX4SIM_DIR/setup_simulator.sh" ]]; then
    log_warn "setup_simulator.sh not found in $UWBPX4SIM_DIR."
    log_warn "The image's UWBPX4Sim checkout may predate that script, or your bind"
    log_warn "mounts may be pointed at the wrong path. Rebuild the image, or check"
    log_warn "docker-compose.yml's volumes, then try again."
    return 1
  fi
  # Not exec'd: setup_simulator.sh is one step in an interactive session, not
  # the container's only job, so control must return here to the menu loop
  # afterwards instead of the container exiting the moment it finishes.
  ( cd "$UWBPX4SIM_DIR" && ./setup_simulator.sh )
  log_info "Setup finished. Back to the menu."

  # setup_simulator.sh ran as a child process, so it could not export
  # straight into this shell -- it wrote its chosen settings to .setup_env
  # instead. Load them here so "Launch simulation" (option 2) picks up the
  # same layout/world/PX4_DIR/ROS_WS without the user re-typing them.
  local setup_env_file="$UWBPX4SIM_DIR/.setup_env"
  if [[ -f "$setup_env_file" ]]; then
    # shellcheck disable=SC1090
    source "$setup_env_file"
    log_info "Loaded simulation settings from setup:"
    log_info "  UWB_LAYOUT_FILE=$UWB_LAYOUT_FILE"
    [[ -n "${GZ_WORLD:-}" ]] && log_info "  GZ_WORLD=$GZ_WORLD"
    log_info "  PX4_DIR=$PX4_DIR"
    [[ -n "${ROS_WS:-}" ]] && log_info "  ROS_WS=$ROS_WS"
  fi
}

run_demo() {
  if [[ ! -d "$PX4_DIR/src/modules/simulation/gz_plugins/uwb_gazebo_plugin" ]]; then
    echo
    log_warn "UWBPX4Sim does not look set up yet under $PX4_DIR."
    if confirm_yes "Run the setup now instead?"; then
      run_setup
    fi
    return 1
  fi

  if [[ ! -f "$UWBPX4SIM_DIR/simulator_launcher.sh" ]]; then
    log_warn "simulator_launcher.sh not found in $UWBPX4SIM_DIR."
    return 1
  fi

  echo
  log_info "Launching the simulation."
  echo
  read -r -p "Press Enter to continue (Ctrl-C to cancel)... " _ || true
  # Not exec'd, same reason as run_setup: come back to the menu once the
  # tmux session ends instead of exiting the container.
  ( cd "$UWBPX4SIM_DIR" && ./simulator_launcher.sh )
  log_info "Simulator session ended. Back to the menu."
}

confirm_yes() {
  local reply
  read -r -p "$1 [y/N] " reply || reply=""
  [[ "$reply" =~ ^[Yy]$ ]]
}

# Exit code do_exit uses to ask setup.sh to actually remove the container
# (rather than just stop it) after this attached session ends -- chosen to
# not collide with a plain signal death (128+signal: 130 SIGINT, 137
# SIGKILL, 143 SIGTERM) or common shell error codes (126/127). setup.sh's
# final `docker exec -it ... /entrypoint.sh` checks for this exact value.
REMOVE_EXIT_CODE=99

do_exit() {
  log_info "Exiting."
  if [[ $$ -eq 1 ]]; then
    # We ARE the container's main process (e.g. started via a plain
    # `docker run`/`docker compose run --rm` instead of setup.sh's detached
    # pattern) -- exiting normally already stops the container.
    exit 0
  fi
  # We're a `docker exec`'d session inside a container whose real PID 1 is
  # setup.sh's idle placeholder -- plain `exit` would only end THIS
  # attached session and leave the container (and PID 1) running, which is
  # right for closing your terminal but wrong for a deliberate Exit.
  # `docker exec` shares the container's PID namespace, so PID 1 here
  # really is the container's init process -- signal it directly to stop
  # the whole container; no docker CLI access from inside needed.
  #
  # Stopping is not the same as removing it, though (see setup.sh's
  # CONTAINER_NAME comment) -- ask which one they actually want.
  if confirm_yes "Remove the container completely (discards this setup) instead of just stopping it?"; then
    log_info "Stopping the background container for removal."
    # Deliberately fire-and-forget, not waiting for PID 1 to actually die:
    # once it does, Docker's container teardown SIGKILLs every remaining
    # process in the namespace near-simultaneously, ourselves included --
    # verified empirically (waiting here got this session SIGKILLed before
    # it could reach its own exit below, exit code 137, never 99). Exiting
    # right away instead wins that race, so setup.sh reliably sees our
    # chosen exit code.
    kill -TERM 1 2>/dev/null || true
    exit "$REMOVE_EXIT_CODE"
  fi
  log_info "Stopping the background container (setup is preserved)."
  kill -TERM 1 2>/dev/null || true
  exit 0
}

run_menu() {
  local choice
  print_welcome
  while true; do
    print_menu
    read -r -p "Select an option [1-4]: " choice || { echo; exec bash; }
    case "$choice" in
      1) run_setup || true ;;
      2) run_demo || true ;;
      3) bash || true; log_info "Shell closed. Back to the menu." ;;
      4) do_exit ;;
      *) echo "Invalid selection: '$choice'" ;;
    esac
  done
}

uwbpx4sim_main() {
  if [[ $# -gt 0 && "$1" != "menu" ]]; then
    exec "$@"
  fi

  if [[ ! -t 0 ]]; then
    log_info "No interactive terminal attached; starting a shell instead of the menu."
    log_info "Re-run with an interactive TTY (./setup.sh, or docker exec -it <container> /entrypoint.sh) for the menu."
    exec bash
  fi

  run_menu
}
