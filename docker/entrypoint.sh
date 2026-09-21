#!/bin/bash
#
# entrypoint.sh - Interactive entrypoint for the uwbpx4sim image.
#
# With no arguments (or "menu"), loops on a menu letting the user run
# UWBPX4Sim's guided setup, launch the simulation, or open a shell -- each
# option returns to the menu when it finishes, so the container stays up for
# the whole session; pick "Exit" to actually end it. Any other arguments are
# exec'd directly, so `docker run <image> <cmd>` still works as a plain
# override.
#
# setup.sh runs the container detached -- PID 1 idles (trapping SIGTERM so
# "Exit" can actually stop it; see setup.sh's PID1_IDLE_CMD comment for why
# a bare `sleep infinity` wouldn't work), not this script -- and joins it
# with `docker exec -it ... /entrypoint.sh`, so this script is normally NOT
# PID 1. That's deliberate: it means the container (and everything
# built/configured inside it) keeps running in the background even if the
# terminal you joined from closes, and re-running setup.sh (or that same
# `docker exec`) later reattaches to a fresh menu on the exact same
# container/filesystem, picking up right where setup left off. "Exit"
# (in entrypoint-lib.sh's do_exit) detects this and stops the whole
# container, not just the attached session.
#
# All the actual logic lives in entrypoint-lib.sh, sourced below -- kept
# separate specifically so a derived image (mr-radio-localization's) can
# source that same library, redefine the menu, and reuse uwbpx4sim_main
# instead of duplicating this whole script. See that file's own header.

set -e

# shellcheck disable=SC1091
source /uwbpx4sim-entrypoint-lib.sh

uwbpx4sim_main "$@"
