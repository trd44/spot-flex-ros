#!/usr/bin/env bash
# start_display.sh — start the virtual X display + noVNC so RViz can be
# viewed in a browser.  Safe to run multiple times; already-running
# processes are left alone.
#
# Browser URL once started: http://localhost:6080/vnc.html

set -euo pipefail

DISPLAY_NUM=1
SCREEN_SIZE="1920x1080x24"
VNC_PORT=5900
NOVNC_PORT=6080
NOVNC_WEB=/usr/share/novnc
LOG_DIR=/tmp/display_logs
mkdir -p "$LOG_DIR"

echo "=== Spot ROS2 display setup ==="

# ── Xvfb ──────────────────────────────────────────────────────────────────
if pgrep -x Xvfb > /dev/null 2>&1; then
    echo "[Xvfb]      already running on :${DISPLAY_NUM}"
else
    echo "[Xvfb]      starting on :${DISPLAY_NUM} ${SCREEN_SIZE} ..."
    Xvfb ":${DISPLAY_NUM}" -screen 0 "${SCREEN_SIZE}" \
        -ac +extension GLX +render -noreset \
        > "$LOG_DIR/xvfb.log" 2>&1 &
    # Give it a moment to bind the socket.
    sleep 1
    if pgrep -x Xvfb > /dev/null 2>&1; then
        echo "[Xvfb]      started (pid $!)"
    else
        echo "[Xvfb]      FAILED — check $LOG_DIR/xvfb.log" >&2
        exit 1
    fi
fi

# ── x11vnc ────────────────────────────────────────────────────────────────
if pgrep -x x11vnc > /dev/null 2>&1; then
    echo "[x11vnc]    already running on VNC port ${VNC_PORT}"
else
    echo "[x11vnc]    starting on port ${VNC_PORT} ..."
    x11vnc -display ":${DISPLAY_NUM}" \
        -rfbport "${VNC_PORT}" \
        -forever -nopw -shared -quiet \
        > "$LOG_DIR/x11vnc.log" 2>&1 &
    sleep 0.5
    echo "[x11vnc]    started (pid $!)"
fi

# ── websockify / noVNC ────────────────────────────────────────────────────
if pgrep -f "websockify.*${NOVNC_PORT}" > /dev/null 2>&1; then
    echo "[noVNC]     already running on port ${NOVNC_PORT}"
else
    echo "[noVNC]     starting on port ${NOVNC_PORT} ..."
    websockify --web "${NOVNC_WEB}" \
        "${NOVNC_PORT}" "localhost:${VNC_PORT}" \
        > "$LOG_DIR/novnc.log" 2>&1 &
    sleep 0.5
    echo "[noVNC]     started (pid $!)"
fi

echo ""
echo "✓  Virtual display ready."
echo "   Open in browser:  http://localhost:${NOVNC_PORT}/vnc.html"
echo "   Then run RViz:    bash /repo/workspace/rviz_novnc.sh -d /repo/workspace/src/spot_flex_nav/rviz/mapping.rviz"
