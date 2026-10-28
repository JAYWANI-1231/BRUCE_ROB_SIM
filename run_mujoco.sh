#!/usr/bin/env bash
# ==============================================================================
# BRUCE Humanoid - Interactive MuJoCo Simulation Launcher
# ==============================================================================
# Usage:
#   ./run_mujoco.sh          (Interactive mode - type English instructions in terminal)
#   ./run_mujoco.sh --demo   (Automated demo sequence: wave, walk, kick, celebrate, dance)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="/home/ncircle/Desktop/mojokui/.venv/bin/python"

if [ ! -f "$PYTHON_BIN" ]; then
    PYTHON_BIN="python3"
fi

cd "$SCRIPT_DIR/MuJoCo"
exec "$PYTHON_BIN" bruce_interactive_sim.py "$@"

