#!/usr/bin/env bash
# Launch RViz on the noVNC virtual display (:1) with software rendering settings
# tuned for arm64 Docker Desktop (Apple Silicon). Classic swrast segfaults here;
# Gallium llvmpipe is more stable.
# NOTE: do NOT use `set -u` — the ROS2 setup.bash scripts reference unset vars
# (AMENT_TRACE_SETUP_FILES etc.) and will fail under nounset.

export DISPLAY=:1
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "$XDG_RUNTIME_DIR" && chmod 700 "$XDG_RUNTIME_DIR"

# Force software GL via Gallium llvmpipe, not the classic swrast driver.
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export LIBGL_DRI3_DISABLE=1
unset MESA_LOADER_DRIVER_OVERRIDE
unset __GLX_VENDOR_LIBRARY_NAME

# Qt / X11 stability flags.
export QT_X11_NO_MITSHM=1
export XLIB_SKIP_ARGB_VISUALS=1
# rviz2's Ogre renderer is happier without GLSL 4+ on llvmpipe.
export OGRE_RTT_MODE=Copy

source /opt/ros/humble/setup.bash
if [[ -f /repo/workspace/install/setup.bash ]]; then
    source /repo/workspace/install/setup.bash
fi

exec ros2 run rviz2 rviz2 "$@"
