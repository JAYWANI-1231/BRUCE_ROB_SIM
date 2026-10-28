#!/usr/bin/env bash
# ==============================================================================
# BRUCE Humanoid - GTA-Style Character Controller Launcher
# ==============================================================================
# Controls:
#   WASD       : Move forward / backward / strafe (Camera-relative)
#   Shift      : Sprint in movement direction
#   Ctrl       : Crouch / Sit (WASD = Crouch-walk)
#   Space      : Jump
#   Mouse Move : Orbit 3rd-person follow camera (Yaw & Pitch)
#   Mouse Wheel: Zoom camera in / out
#   Left Click : Kick soccer ball (or key 'F')
#   Right Click: Push obstacle boxes (or key 'E')
#   G          : Wave arm
#   R          : Reset robot & environment objects
#   Tab / Esc  : Toggle mouse lock / unlock
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="/home/ncircle/Desktop/mojokui/.venv/bin/python"

# Clean up snap environment pollution that breaks native GUI apps (e.g. GLFW / OpenGL)
unset GTK_PATH
unset GTK_MODULES
unset LOCPATH
unset GIO_MODULE_DIR

if [ ! -f "$PYTHON_BIN" ]; then
    PYTHON_BIN="python3"
fi

cd "$SCRIPT_DIR/MuJoCo"
chmod +x bruce_game_controller.py
exec "$PYTHON_BIN" bruce_game_controller.py "$@"
