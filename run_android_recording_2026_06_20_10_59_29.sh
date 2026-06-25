#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
export LD_LIBRARY_PATH="$PWD/lib:$PWD/Thirdparty/DBoW2/lib:$PWD/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

./Examples/Monocular-Inertial/mono_inertial_euroc \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular-Inertial/Android_recording_2026_06_20_10_59_29.yaml \
  ../datasets/android/recording_2026_06_20_10_59_29 \
  Examples/Monocular-Inertial/Android_TimeStamps/recording_2026_06_20_10_59_29.txt \
  android_recording_2026_06_20_10_59_29
