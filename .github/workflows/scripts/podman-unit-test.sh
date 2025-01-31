#!/usr/bin/env bash

image_url=$1

mkdir -p ~/.cache/bazel
mkdir -p ~/.cache/bazel-disk
mkdir -p ~/.cache/bazel-repo

podman run \
  --volume $GITHUB_WORKSPACE:/revoy-open \
  --volume ~/.cache/bazel:/root/.cache/bazel:z \
  --volume ~/.cache/bazel-disk:/root/.cache/bazel-disk:z \
  --volume ~/.cache/bazel-repo:/root/.cache/bazel-repo:z \
  --workdir /revoy-open \
  $image_url \
  ./ci-runner/unit-test.sh
