#!/usr/bin/env bash

image_url=$1

podman run \
  --volume $GITHUB_WORKSPACE:/revoy-open \
  --workdir /revoy-open \
  $image_url \
  ./ci-runner/check-formatting.sh

