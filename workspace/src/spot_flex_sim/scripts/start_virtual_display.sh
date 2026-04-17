#!/usr/bin/env bash
set -euo pipefail

STATE_DIR="${SPOT_VNC_STATE_DIR:-/tmp/spot_flex_sim_vnc}"
DISPLAY_NUM="${SPOT_VNC_DISPLAY_NUM:-1}"
DISPLAY_NAME=":${DISPLAY_NUM}"
SCREEN_GEOMETRY="${SPOT_VNC_SCREEN_GEOMETRY:-1920x1080x24}"
VNC_PORT="${SPOT_VNC_PORT:-5900}"
NOVNC_PORT="${SPOT_NOVNC_PORT:-6080}"
NOVNC_BIND="${SPOT_NOVNC_BIND:-0.0.0.0}"
NOVNC_WEB_ROOT="${SPOT_NOVNC_WEB_ROOT:-/usr/share/novnc}"

XVFB_PID_FILE="${STATE_DIR}/xvfb.pid"
X11VNC_PID_FILE="${STATE_DIR}/x11vnc.pid"
WEBSOCKIFY_PID_FILE="${STATE_DIR}/websockify.pid"
XVFB_LOG_FILE="${STATE_DIR}/xvfb.log"
X11VNC_LOG_FILE="${STATE_DIR}/x11vnc.log"
WEBSOCKIFY_LOG_FILE="${STATE_DIR}/websockify.log"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

pid_is_running() {
  local pid_file="$1"
  [[ -f "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" 2>/dev/null
}

start_process() {
  local name="$1"
  local pid_file="$2"
  local log_file="$3"
  shift 3

  if pid_is_running "${pid_file}"; then
    echo "${name} already running (pid $(cat "${pid_file}"))"
    return
  fi

  nohup "$@" >"${log_file}" 2>&1 &
  local pid=$!
  echo "${pid}" >"${pid_file}"

  sleep 2
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "${name} failed to start. Log follows:" >&2
    cat "${log_file}" >&2 || true
    exit 1
  fi
}

wait_for_display() {
  local display_name="$1"
  for _ in $(seq 1 15); do
    if DISPLAY="${display_name}" xdpyinfo >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  echo "Display ${display_name} did not become ready." >&2
  cat "${XVFB_LOG_FILE}" >&2 || true
  exit 1
}

require_cmd Xvfb
require_cmd x11vnc
require_cmd websockify
require_cmd xdpyinfo

mkdir -p "${STATE_DIR}"

if ! pid_is_running "${XVFB_PID_FILE}" && ! DISPLAY="${DISPLAY_NAME}" xdpyinfo >/dev/null 2>&1; then
  rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"
fi

if pid_is_running "${XVFB_PID_FILE}"; then
  echo "Xvfb already running (pid $(cat "${XVFB_PID_FILE}"))"
elif DISPLAY="${DISPLAY_NAME}" xdpyinfo >/dev/null 2>&1; then
  echo "Display ${DISPLAY_NAME} already exists; reusing it."
else
  start_process \
    "Xvfb" \
    "${XVFB_PID_FILE}" \
    "${XVFB_LOG_FILE}" \
    Xvfb "${DISPLAY_NAME}" -screen 0 "${SCREEN_GEOMETRY}" -ac +extension GLX +render -noreset
fi

wait_for_display "${DISPLAY_NAME}"

start_process \
  "x11vnc" \
  "${X11VNC_PID_FILE}" \
  "${X11VNC_LOG_FILE}" \
  x11vnc -display "${DISPLAY_NAME}" -forever -shared -rfbport "${VNC_PORT}" -localhost -nopw

start_process \
  "websockify" \
  "${WEBSOCKIFY_PID_FILE}" \
  "${WEBSOCKIFY_LOG_FILE}" \
  websockify --web="${NOVNC_WEB_ROOT}" "${NOVNC_BIND}:${NOVNC_PORT}" "localhost:${VNC_PORT}"

cat <<EOF
Virtual display ready.
DISPLAY=${DISPLAY_NAME}
noVNC URL: http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale

If you are in VS Code or another devcontainer client, forward container port ${NOVNC_PORT}
to your host and open the URL above in your browser.
EOF
