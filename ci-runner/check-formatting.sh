#!/usr/bin/env bash

set -x
set -e

if [[ `git status --porcelain` ]]; then
  echo "[check-formatting.sh]: must be performed on a clean git repository with no diff"
  exit 1
fi

dir=$(pwd)

bazel run --config=container //code_style:format_everything /revoy-open

if [[ `git status --porcelain` ]]; then
  echo "[check-formatting.sh]: formatting errors detected! To get rid of this error:  \`bazel run //code_style:format_everything -- /revoy-open \`, commit the changes, and push."
  echo "[check-formatting.sh]: the following files had errors:"
  git status
  git diff
  git reset --hard
  exit 1
fi
