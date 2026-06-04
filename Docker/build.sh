#!/bin/sh
set -ex

export DOCKER_BUILDKIT=0

docker build \
    --file \
    Dockerfile \
    --build-arg=UID=$(id -u) \
    --build-arg=GID=$(id -g) \
    --tag=docker3dsensors \
    .
