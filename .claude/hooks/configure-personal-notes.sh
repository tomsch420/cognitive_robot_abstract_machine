#!/bin/bash
set -euo pipefail

# Generic environment-setup script for the personal-notes SessionStart hook
# (./session-start.sh). Paste this into your cloud/web session environment's
# "setup script" (a field that runs arbitrary commands on every fresh clone,
# as opposed to a persistent environment-variable list) if that's what your
# environment offers instead of persistent environment variables.
#
# It reads the same two variable names as ./personal-notes.env.example
# (CLAUDE_PERSONAL_NOTES_BRANCH, CLAUDE_PERSONAL_NOTES_PATH) and seeds them
# into this fresh clone's git config, so session-start.sh finds them there
# exactly as it would for a persistent local clone where you ran `git config`
# yourself once. Set the two variables however your environment lets you
# (its own persistent-environment-variable feature, or literal `export` lines
# pasted above the call to this script) - this script only turns them into
# git config, it does not define them itself, so it stays generic: nothing
# here is specific to any one contributor's branch name.
#
# No-op (exit 0, no git config written) if CLAUDE_PERSONAL_NOTES_BRANCH isn't
# set, so it's always safe to include in a setup script even before you've
# opted in.
#
# Safe to re-run: `git config` simply overwrites the previous value.

NOTES_BRANCH="${CLAUDE_PERSONAL_NOTES_BRANCH:-}"
[ -n "${NOTES_BRANCH}" ] || exit 0

git config claude.personalNotesBranch "${NOTES_BRANCH}"

NOTES_PATH="${CLAUDE_PERSONAL_NOTES_PATH:-}"
if [ -n "${NOTES_PATH}" ]; then
  git config claude.personalNotesPath "${NOTES_PATH}"
fi
