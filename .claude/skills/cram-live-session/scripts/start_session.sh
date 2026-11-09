#!/usr/bin/env bash
# Start the live-session kernel and the cramera viewer inside this container, each only
# if it does not run yet. Run it detached, e.g.
#   docker exec -d CONTAINER bash /opt/cram/.claude/skills/cram-live-session/scripts/start_session.sh
# Settings come from the environment; the defaults are the neem-4 RoboCasa container's.
set -euo pipefail

REPOSITORY=${LIVE_REPOSITORY:-/opt/cram}
ENVIRONMENT_SCRIPT=${LIVE_ENVIRONMENT_SCRIPT:-/root/cramera_live_env.sh}
SESSION_DIRECTORY=${LIVE_SESSION_DIRECTORY:-/root/work/live_session}
VIEWER_PORT=${LIVE_VIEWER_PORT:-8711}
MEMORY_LIMIT_KB=${LIVE_MEMORY_LIMIT_KB:-150000000}

mkdir -p "$SESSION_DIRECTORY"
cd "$REPOSITORY"
source "$ENVIRONMENT_SCRIPT"
export QT_QPA_PLATFORM=offscreen

if ! pgrep -f "^[^ ]*python[^ ]* -m ipykernel_launcher -f $SESSION_DIRECTORY/kernel.json" > /dev/null; then
    (
        ulimit -v "$MEMORY_LIMIT_KB"
        exec python -m ipykernel_launcher -f "$SESSION_DIRECTORY/kernel.json" \
            > "$SESSION_DIRECTORY/kernel.log" 2>&1
    ) &
fi

if ! pgrep -f "cramera --no-browser --port $VIEWER_PORT" > /dev/null; then
    cramera --no-browser --port "$VIEWER_PORT" > "$SESSION_DIRECTORY/viewer.log" 2>&1 &
fi
wait
