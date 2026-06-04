#!/bin/sh
set -ex

script_path=$(readlink -e "$(dirname "$0")")

cd "${script_path}"
export DOCKER_BUILDKIT=0
./build.sh
./run.sh
