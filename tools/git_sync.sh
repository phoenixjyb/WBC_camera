#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 2 ]; then
    cat <<USAGE >&2
Usage: $(basename "$0") "commit message" path1 [path2 ...]
Automates staging the given paths, committing, rebasing on origin, and pushing.
USAGE
    exit 1
fi

commit_msg=$1
shift
paths=("$@")

if ! git rev-parse --show-toplevel >/dev/null 2>&1; then
    echo "Error: Not inside a git repository." >&2
    exit 1
fi

root=$(git rev-parse --show-toplevel)
cd "$root"

branch=$(git rev-parse --abbrev-ref HEAD)

echo "Preparing to sync branch '$branch' with commit message: $commit_msg"
echo "Paths to stage: ${paths[*]}"

git status --short --untracked-files=no -- "${paths[@]}"

read -r -p "Continue with staging the listed paths? [y/N] " answer
if [[ ! $answer =~ ^[Yy]$ ]]; then
    echo "Aborting by user request."
    exit 1
fi

for target in "${paths[@]}"; do
    if [ ! -e "$target" ]; then
        echo "Warning: $target does not exist and will be ignored." >&2
    fi
    git add "$target" 2>/dev/null || true
done

if git diff --cached --quiet; then
    echo "Nothing staged; aborting commit." >&2
    exit 1
fi

git commit -m "$commit_msg"

git fetch origin

git pull --rebase origin "$branch"

git push origin "$branch"

echo "Sync complete."
