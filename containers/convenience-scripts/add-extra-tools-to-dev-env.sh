#!/usr/bin/env bash

set -x
set -e

created_by="revoy"
author="revoy"
image_name=${1:-"revoy-open"}

dnf_install="dnf install -y"

# assume container is named revoy-open
container=$(buildah from revoy-open)

# basic utils that help with packages, navigation, editing
utils="wget curl unzip tar gzip git fd-find ripgrep fish helix"
buildah run $container $dnf_install $utils

# cleanup after installations
buildah run $container dnf clean all

# commit
buildah config --author $author --created-by $created_by --label name=$image_name $container
buildah commit $container $image_name
