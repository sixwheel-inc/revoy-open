#!/usr/bin/env bash

set -ex

image_url=$1

podman run \
  --privileged \
  --volume $GITHUB_WORKSPACE:/revoy-open \
  --volume ~/.cache/bazel:/root/.cache/bazel:z \
  --volume ~/.cache/bazel-disk:/root/.cache/bazel-disk:z \
  --volume ~/.cache/bazel-repo:/root/.cache/bazel-repo:z \
  --workdir /revoy-open \
  $image_url \
  ./ci-runner/chrono-test.sh
