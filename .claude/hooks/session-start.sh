#!/bin/bash
set -euo pipefail

# Generic, opt-in personal Claude Code notes hook.
#
# Populates CLAUDE.local.md (gitignored, never committed on any branch) from a
# personal branch on `origin` that each contributor names for themselves via
# local git config, so this stays a complete no-op for anyone who hasn't opted
# in, and collision-free for multiple contributors sharing one origin remote
# (each points at their own branch name instead of one hardcoded literal).
#
# Opt in once, locally, per clone (never committed):
#   git config claude.personalNotesBranch <your-branch-name>
#   git config claude.personalNotesPath   <path-on-that-branch>   # optional
#
# git config is per-clone, so it's the wrong mechanism anywhere sessions start
# from a fresh clone every time (e.g. cloud/web sessions) - there's no
# persistent .git/config for it to live in. For that case, opt in via
# persistent environment variables instead (configured once at the environment
# level, outside the repo, so they survive every fresh clone):
#   CLAUDE_PERSONAL_NOTES_BRANCH=<your-branch-name>
#   CLAUDE_PERSONAL_NOTES_PATH=<path-on-that-branch>   # optional
# See ./README.md for exactly how to wire these into a cloud environment.
# git config wins when both are set, so a local override always takes
# precedence over an environment-level default.
#
# With neither set, this exits immediately: no fetch, no write, no effect for
# anyone who hasn't configured it.
#
# Safe to re-run: it only ever overwrites CLAUDE.local.md, and does nothing if
# the configured branch or path isn't reachable (e.g. a fresh clone, or a fork
# that never created it).
#
# How this script gets invoked (see ../settings.json): Claude Code registers it
# as a SessionStart hook via `$CLAUDE_PROJECT_DIR/.claude/hooks/session-start.sh`.
# CLAUDE_PROJECT_DIR is an env var Claude Code itself injects into every hook
# command's environment, resolving to this project's root - so that path is
# correct regardless of Claude Code's own cwd when it runs the hook.
#
# Coexistence with your own settings: Claude Code merges the `hooks` arrays
# across all settings layers (managed > CLI args > .claude/settings.local.json
# > .claude/settings.json (this repo's, committed) > ~/.claude/settings.json)
# by concatenation, not override. So this SessionStart hook runs alongside -
# never instead of - any SessionStart hook you already have configured for
# yourself. settings.json is strict JSON with no comment support, which is why
# this explanation lives here instead of there.

NOTES_BRANCH="$(git config --get claude.personalNotesBranch || true)"
NOTES_BRANCH="${NOTES_BRANCH:-${CLAUDE_PERSONAL_NOTES_BRANCH:-}}"
[ -n "${NOTES_BRANCH}" ] || exit 0

NOTES_PATH="$(git config --get claude.personalNotesPath || true)"
NOTES_PATH="${NOTES_PATH:-${CLAUDE_PERSONAL_NOTES_PATH:-.claude/personal/cram-notes.md}}"

git fetch origin "${NOTES_BRANCH}" --quiet 2>/dev/null || exit 0

if git cat-file -e "origin/${NOTES_BRANCH}:${NOTES_PATH}" 2>/dev/null; then
  git show "origin/${NOTES_BRANCH}:${NOTES_PATH}" > CLAUDE.local.md
fi
