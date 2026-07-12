# Personal Claude Code notes hook

An opt-in `SessionStart` hook that populates `CLAUDE.local.md` — which Claude Code already loads
automatically as project memory, and which is gitignored — from a personal branch you name for
yourself on `origin`, so your own workflow preferences ("always open my PRs as drafts," "never
touch branch X directly," etc.) persist across sessions without ever being committed to a shared
branch.

It is a complete no-op until you opt in, and collision-free for multiple contributors sharing one
`origin` remote, because each contributor points at their own branch name via their own config
rather than one hardcoded literal.

## How it decides what to read

`session-start.sh` looks for a branch name in this order, first one found wins:

1. **`git config claude.personalNotesBranch`** — local to one clone's `.git/config`.
2. **`CLAUDE_PERSONAL_NOTES_BRANCH` environment variable** — used only if the git config isn't set.

The path on that branch follows the same precedence (`claude.personalNotesPath` git config, then
`CLAUDE_PERSONAL_NOTES_PATH` env var, then the default `.claude/personal/cram-notes.md`).

If neither the git config nor the environment variable is set, the hook exits immediately: no
fetch, no write, no effect.

Which one you need depends on how your sessions start:

- **A persistent local clone** (you `git clone` once and keep working in it) → use git config. It's
  set once and stays in that clone's `.git/config` forever.
- **A fresh clone every session** (e.g. a cloud/web session environment that clones the repo from
  scratch each time) → git config **will not work**: `.git/config` is part of the fresh clone, so
  whatever you set there is gone by the next session. Use the environment variable instead,
  configured once at the environment level (outside the repo), so it survives every fresh clone.

## Setup: persistent local clone

Once per clone, never committed:

```bash
git config claude.personalNotesBranch <your-branch-name>
git config claude.personalNotesPath   <path-on-that-branch>   # optional, defaults to
                                                                 # .claude/personal/cram-notes.md
```

Push your notes file to that branch on `origin` (any branch name, any path — it never merges
anywhere):

```bash
git checkout --orphan <your-branch-name>
git rm -rf --cached .
mkdir -p .claude/personal
echo "- your notes here" > .claude/personal/cram-notes.md
git add .claude/personal/cram-notes.md
git commit -m "personal notes"
git push origin <your-branch-name>
```

Every new Claude Code session in this clone now runs the hook automatically and writes
`CLAUDE.local.md` from that branch.

## Setup: cloud/web sessions (fresh clone every time)

Push your notes file to `origin` exactly as above first. Then wire the environment variable into
your session environment's configuration — which of the two options below applies depends on what
your specific environment offers:

### Option A: your environment has a persistent environment-variable list

Copy [`personal-notes.env.example`](./personal-notes.env.example) into that list, with your own
values substituted:

```
CLAUDE_PERSONAL_NOTES_BRANCH=<your-branch-name>
CLAUDE_PERSONAL_NOTES_PATH=<path-on-that-branch>
```

`session-start.sh` reads these directly — nothing else to configure.

### Option B: your environment has a "setup script" (arbitrary commands run on every fresh clone)

Set the same two variables however that setup script can see them (its own env-var mechanism, or
literal `export` lines above the call), then run
[`configure-personal-notes.sh`](./configure-personal-notes.sh), e.g.:

```bash
export CLAUDE_PERSONAL_NOTES_BRANCH=<your-branch-name>
export CLAUDE_PERSONAL_NOTES_PATH=<path-on-that-branch>   # optional
"$CLAUDE_PROJECT_DIR/.claude/hooks/configure-personal-notes.sh"
```

This seeds the fresh clone's git config from those variables, so `session-start.sh` finds them
exactly as it would for a persistent local clone. It's a no-op if
`CLAUDE_PERSONAL_NOTES_BRANCH` isn't set, so it's safe to include even before you've opted in.

See your environment provider's docs for exactly where to paste a setup script or persistent
environment variables (for Claude Code on the web: <https://code.claude.com/docs/en/claude-code-on-the-web>).

## Verifying it worked

Start a fresh session and check whether `CLAUDE.local.md` exists at the project root with your
notes content. To check the mechanics without waiting for a real session boot, run the hook
directly:

```bash
"$CLAUDE_PROJECT_DIR/.claude/hooks/session-start.sh" && cat CLAUDE.local.md
```

## Safety

- No-op by default: nothing happens for anyone who hasn't configured either the git config or the
  environment variable.
- Never merges anything: the hook only ever *reads* the configured branch via `git show`. It never
  checks it out or merges it into your working branch.
- `CLAUDE.local.md` is gitignored, so populated notes can't accidentally end up in a commit on any
  branch, including this one.
- Safe to re-run: it only ever overwrites `CLAUDE.local.md`, and does nothing if the configured
  branch or path isn't reachable (e.g. a fresh clone, or a fork that never created it).
- Coexists with your own hooks: Claude Code merges `SessionStart` hook arrays across all settings
  layers by concatenation, not override, so this hook runs alongside — never instead of — any
  `SessionStart` hook you already have configured for yourself.
