#!/usr/bin/env bash

set -ex

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
