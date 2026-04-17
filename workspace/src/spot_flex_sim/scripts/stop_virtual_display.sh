#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="${SPOT_VNC_STATE_DIR:-/tmp/spot_flex_sim_vnc}"
DISPLAY_NUM="${SPOT_VNC_DISPLAY_NUM:-1}"

stop_process() {
  local name="$1"
  local pid_file="$2"

  if [[ ! -f "${pid_file}" ]]; then
    return
  fi

  local pid
  pid="$(cat "${pid_file}")"
  rm -f "${pid_file}"

  if ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi

  kill "${pid}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done

  if kill -0 "${pid}" 2>/dev/null; then
    kill -9 "${pid}" 2>/dev/null || true
  fi

  echo "Stopped ${name} (pid ${pid})"
}

stop_process "websockify" "${STATE_DIR}/websockify.pid"
stop_process "x11vnc" "${STATE_DIR}/x11vnc.pid"
stop_process "Xvfb" "${STATE_DIR}/xvfb.pid"

rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"
