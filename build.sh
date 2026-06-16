#!/usr/bin/env bash
set -Eeuo pipefail

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

build_cmake_project "${ROOT_DIR}/Thirdparty/DBoW2" "Thirdparty/DBoW2"
build_cmake_project "${ROOT_DIR}/Thirdparty/g2o" "Thirdparty/g2o"
build_cmake_project "${ROOT_DIR}/Thirdparty/Sophus" "Thirdparty/Sophus" -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF

echo ""
echo "Uncompressing vocabulary if needed ..."
cd "${ROOT_DIR}/Vocabulary"
if [ ! -f ORBvoc.txt ]; then
  tar -xf ORBvoc.txt.tar.gz
fi

build_cmake_project "${ROOT_DIR}" "ORB_SLAM3"

echo ""
echo "Build completed successfully."
echo "Library: ${ROOT_DIR}/lib/libORB_SLAM3.so"
echo "Examples: ${ROOT_DIR}/Examples"
