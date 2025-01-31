#!/usr/bin/env bash

set -e
set -x

image_name=${1:-"revoy-open"}

mkdir -p ~/.cache/bazel

# Use this as an example, and put something like this
# as an alias / function in your local configs.
# Use the client of your choice, i.e. Docker, Podman, etc.
podman run \
  --name revoy-open-dev-env \
  --interactive --tty \
  --volume ~/revoy-open:/revoy-open:z \
  --volume ~/.cache/bazel:/root/.cache/bazel:z \
  --volume ~/.cache/bazel-disk:/root/.cache/bazel-disk:z \
  --workdir /revoy-open \
  $image_name \
  bash
