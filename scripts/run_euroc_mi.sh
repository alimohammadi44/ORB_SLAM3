#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 /path/to/EuRoC_sequence"
  exit 1
fi

SEQ_DIR="$1"
"${ROOT_DIR}/Examples/Monocular-Inertial/mono_inertial_euroc" \
  "${ROOT_DIR}/Vocabulary/ORBvoc.txt" \
  "${ROOT_DIR}/Examples/Monocular-Inertial/EuRoC.yaml" \
  "${SEQ_DIR}" \
  "${ROOT_DIR}/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt"
