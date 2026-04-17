#!/usr/bin/env bash

display_num="${SPOT_VNC_DISPLAY_NUM:-1}"
runtime_dir="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

mkdir -p "${runtime_dir}"
chmod 700 "${runtime_dir}" 2>/dev/null || true

export DISPLAY=":${display_num}"
export XDG_RUNTIME_DIR="${runtime_dir}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-llvmpipe}"
