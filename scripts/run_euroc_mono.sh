#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 /path/to/EuRoC_sequence"
  echo "Example: $0 ~/slam_ws/datasets/euroc/MH_01_easy/mav0"
  exit 1
fi

SEQ_DIR="$1"
VOCAB="${ROOT_DIR}/Vocabulary/ORBvoc.txt"
SETTINGS="${ROOT_DIR}/Examples/Monocular/EuRoC.yaml"
TIMES="${ROOT_DIR}/Examples/Monocular/EuRoC_TimeStamps/MH01.txt"
EXE="${ROOT_DIR}/Examples/Monocular/mono_euroc"

"${EXE}" "${VOCAB}" "${SETTINGS}" "${SEQ_DIR}" "${TIMES}"
