#!/usr/bin/env bash

set -ex

bazel test \
  --noshow_progress \
  --config=container \
  --cache_test_results=no \
  --test_env=BAZEL_BIN=$(bazel info bazel-bin) \
  --test_output=all \
  //...

