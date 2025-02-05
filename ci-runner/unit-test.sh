#!/usr/bin/env bash

set -ex

# occasionally causes massive build time spikes because of cc_foreign cmake cache miss.
exclude_chrono="\
-//zaxis/... \
-//assembly/... \
-//revoy-chrono/... \
"

bazel test \
  --noshow_progress \
  --config=container \
  --cache_test_results=no \
  --test_env=BAZEL_BIN=$(bazel info bazel-bin) \
  --test_output=all \
  //... -- \
  $exclude_chrono \

