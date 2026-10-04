#!/usr/bin/env bash
# Pull the latest code on the Jetson and re-install. Run as boat from the checkout:
#   deploy/update.sh            # fetch, show incoming commits, fast-forward, sudo install.sh --restart
#   deploy/update.sh --no-restart
# Restarting mavlink-router drops QGC/MOOS links for a few seconds: don't update while armed.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

RESTART=--restart
[[ ${1:-} == --no-restart ]] && RESTART=

if [[ -n $(git status --porcelain --untracked-files=no) ]]; then
    echo "local changes in the checkout; commit or 'git stash' them first:" >&2
    git status --short --untracked-files=no >&2; exit 1
fi

git fetch --quiet origin
incoming=$(git log --oneline 'HEAD..@{u}')
if [[ -z $incoming ]]; then
    echo "already up to date ($(git log -1 --format='%h %s'))"
else
    echo "incoming:"; while IFS= read -r l; do echo "  $l"; done <<<"$incoming"
    git merge --ff-only --quiet '@{u}'
fi

# shellcheck disable=SC2086  # RESTART is empty or one flag
sudo deploy/install.sh $RESTART
