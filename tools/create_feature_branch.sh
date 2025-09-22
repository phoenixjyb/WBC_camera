#!/usr/bin/env bash
set -euo pipefail

TARGET_BRANCH="${1:-feature/gik-prototype}"
BASE_REF="${BASE_REF:-master}"
USE_REMOTE_BASE="${USE_REMOTE_BASE:-0}"
REMOTE_NAME="${REMOTE_NAME:-origin}"

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  echo "Error: not inside a git repository." >&2
  exit 1
fi

if git show-ref --verify --quiet "refs/heads/${TARGET_BRANCH}"; then
  echo "Branch ${TARGET_BRANCH} already exists. Nothing to do."
  exit 0
fi

BASE_TARGET="${BASE_REF}"
if [[ "${USE_REMOTE_BASE}" == "1" ]]; then
  BASE_TARGET="${REMOTE_NAME}/${BASE_REF}"
fi

if ! git rev-parse --verify --quiet "${BASE_TARGET}^{commit}"; then
  echo "Error: base reference ${BASE_TARGET} not found." >&2
  exit 1
fi

if [[ "${USE_REMOTE_BASE}" == "1" ]]; then
  echo "Creating ${TARGET_BRANCH} from ${BASE_TARGET} (remote)."
else
  echo "Creating ${TARGET_BRANCH} from ${BASE_TARGET}."
fi

git branch "${TARGET_BRANCH}" "${BASE_TARGET}"

CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

if [[ "${CURRENT_BRANCH}" == "${TARGET_BRANCH}" ]]; then
  echo "Already on ${TARGET_BRANCH}."
else
  echo "Branch ${TARGET_BRANCH} created. Keep working on ${CURRENT_BRANCH} or switch later with: git switch ${TARGET_BRANCH}"
fi
