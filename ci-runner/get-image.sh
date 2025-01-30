#!/usr/bin/env bash
set -x
set -e
image_name=${1}
podman login ghcr.io --username $REGISTRY_USER --password $REGISTRY_PASSWORD
if podman image exists $image_name; then
   echo "image exist locally."
   exit 0
else
   echo "image does not exist locally trying to pull image"
   if podman pull $image_name; then
      echo "successfully pulled image"
      exit 0
   else
      echo "Image was not found."
      exit 1
   fi
fi
