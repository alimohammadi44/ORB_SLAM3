#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "Checking ORB-SLAM3 mono_slam installation ..."
echo "Root: ${ROOT_DIR}"

missing=0

check_path() {
  local path="$1"
  local label="$2"
  if [ -e "${path}" ]; then
    echo "OK: ${label}"
  else
    echo "MISSING: ${label} -> ${path}"
    missing=1
  fi
}

check_path "${ROOT_DIR}/Vocabulary/ORBvoc.txt" "ORB vocabulary"
check_path "${ROOT_DIR}/lib/libORB_SLAM3.so" "ORB-SLAM3 shared library"
check_path "${ROOT_DIR}/Examples/Monocular/mono_euroc" "EuRoC monocular executable"
check_path "${ROOT_DIR}/Examples/Monocular-Inertial/mono_inertial_euroc" "EuRoC mono-inertial executable"

if [ "${missing}" -ne 0 ]; then
  echo "Installation is incomplete. Run: ./build.sh 2>&1 | tee build_log.txt"
  exit 1
fi

echo "Installation check passed."
