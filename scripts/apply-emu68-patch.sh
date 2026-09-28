#!/usr/bin/env bash
set -Eeuo pipefail

EXPECTED="0ba3a899341dee958332a4182911861420b03bff"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH="$HERE/arm/emu68-aga.patch"
PATCH2="$HERE/arm/emu68-program-run.patch"
TARGET="${1:-$PWD}"

fail(){ echo "ERROR: $*" >&2; exit 1; }

command -v git >/dev/null 2>&1 || fail "git is required"
[[ -f "$PATCH" ]] || fail "integration patch not found: $PATCH"
[[ -f "$PATCH2" ]] || fail "program/splash patch not found: $PATCH2"
[[ -d "$TARGET/.git" ]] || fail "not a Git checkout: $TARGET"

TARGET="$(cd "$TARGET" && pwd)"
HEAD="$(git -C "$TARGET" rev-parse HEAD)"
[[ "$HEAD" == "$EXPECTED" ]] || fail "wrong Emu68 revision: $HEAD (expected $EXPECTED)"

if [[ -n "$(git -C "$TARGET" status --porcelain --untracked-files=no)" ]]; then
    fail "Emu68 worktree has tracked changes; use a clean checkout"
fi

# The second patch is intentionally based on the result of the main AGA patch,
# so validate/apply them in sequence. Roll back tracked changes if stage 2 ever
# fails; the caller started from a clean pinned checkout.
git -C "$TARGET" apply --check "$PATCH" || fail "main AGA patch does not apply cleanly"
git -C "$TARGET" apply "$PATCH"
if ! git -C "$TARGET" apply --check "$PATCH2"; then
    git -C "$TARGET" reset --hard "$EXPECTED" >/dev/null
    fail "program-scoped RUN/splash patch does not apply cleanly"
fi
git -C "$TARGET" apply "$PATCH2"

echo "PASS: PiStorm AGA/HAM6 + program-scoped RUN integration applied to Emu68 $EXPECTED"
echo "Configure with: -DAGA_PISTORM=$HERE"
