#!/usr/bin/env bash

image_url=$1

mkdir -p ~/.cache/bazel
podman run \
  --volume $GITHUB_WORKSPACE:/revoy-open \
  --volume ~/.cache/bazel:/root/.cache/bazel:z \
  --workdir /revoy-open \
  $image_url \
  ./ci-runner/unit-test.sh
