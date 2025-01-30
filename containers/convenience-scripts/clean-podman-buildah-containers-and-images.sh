#!/usr/bin/env bash

# CAUTION: this will require you to redownload / rebuild the images.

podman rm --all
buildah rm --all
podman rmi --all
buildah rmi --all
