#!/usr/bin/env bash

set -ex

echo "run unittests, usually takes ~1-2 min, but will take 3-4 hours when cc_foreign cmake cache misses"

export BAZEL_BIN=$(bazel info bazel-bin)

bazel test \
  --noshow_progress \
  --config=container \
  --cache_test_results=no \
  --test_env=BAZEL_BIN=$BAZEL_BIN \
  --test_output=all \
  //zaxis/... \
  //assembly/... \
  //revoy-chrono/... \


echo "run scenarios and generate the mcaps"

export REVOY_IRRLICHT_DISABLE=true

SWERVE="bazel run \
  --config=container \
  --noshow_progress \
  //zaxis:run-zaxis -- \
  {1} swerve {2} {3} 10 \
"

echo "run swerving scenarios"
parallel -j1 $SWERVE ::: tractor-trailer tractor-revoy-trailer ::: 5 10 15 20 ::: 0.2 0.3 0.4 0.5 

STARTSTOP="bazel run \
  --config=container \
  --noshow_progress \
  //zaxis:run-zaxis -- \
  {1} start-stop {2} \
"

echo "run start-stop scenarios"
parallel -j1 $STARTSTOP ::: tractor-trailer tractor-revoy-trailer ::: 5 10 15 20 
