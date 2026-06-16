#!/usr/bin/env bash
set -Eeuo pipefail

# Safer ORB-SLAM3 build script for laptop development.
# Default to 2 parallel jobs to avoid RAM crashes on VMs and laptops.
# Override with: JOBS=4 ./build.sh

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS="${JOBS:-2}"
CMAKE_POLICY_FLAG="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"

build_cmake_project() {
  local project_dir="$1"
  local project_name="$2"
  shift 2

  echo ""
  echo "Configuring and building ${project_name} ..."
  cd "${project_dir}"
  mkdir -p build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release ${CMAKE_POLICY_FLAG} "$@"
  make -j"${JOBS}"
}

build