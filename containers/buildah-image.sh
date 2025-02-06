#!/usr/bin/env bash

set -x
set -e

tag=${1:-"latest"}

created_by="revoy"
author="revoy"
image_name="revoy-open"

echo "Making Container Image with Buildah." 
echo "Image Name: ${image_name}" 
echo "Tag: ${tag}"

main_packages="
  clang \
  git \
  parallel \
"

python_packages="\
 gcc \
 make \
 python-unversioned-command \
 python3 \
 python3-devel \
 python3-libs \
 python3-pip \
"

formatting_packages="\
  clang-tools-extra \
"

setup_packages="\
  diffutils \
  dpkg \
  unzip \
  wget \
"

golang_packages=" \
  patch \
"

foxglove_packages=" \
  protobuf-devel \
  lz4-devel \
  libzstd-devel \
"

projectchrono_packages=" \
  eigen3-devel \
  irrlicht-devel \
  freeglut-devel \
"

dnf_install="dnf install -y"

# initalize container
container=$(buildah from fedora:40)

# most of the packages we need are available via dnf
buildah run $container $dnf_install \
    $formatting_packages \
    $foxglove_packages \
    $main_packages \
    $python_packages \
    $setup_packages \
    $golang_packages \
    $projectchrono_packages

buildah run $container dnf clean all

# use the bazelisk bazel wrapper, bazel --version will match /revoy-open/.bazelversion
buildah run $container wget -q "https://github.com/bazelbuild/bazelisk/releases/download/v1.22.0/bazelisk-linux-amd64"
buildah run $container mv bazelisk-linux-amd64 /usr/bin/bazel
buildah run $container chmod +x-w /usr/bin/bazel

# bazel buildifier (formatting) 
buildah run $container wget -q "https://github.com/bazelbuild/buildtools/releases/download/5.1.0/buildifier-linux-amd64"
buildah run $container mv buildifier-linux-amd64 /usr/bin/buildifier
buildah run $container chmod +x-w /usr/bin/buildifier

# python black (formatting)
buildah run $container pip -q install black

# used for making the requirement.lock files for bazel
buildah run $container pip -q install pip-tools
 
# commit
buildah config --author $author --created-by $created_by --label name=$image_name $container
buildah commit --rm $container $image_name:$tag
