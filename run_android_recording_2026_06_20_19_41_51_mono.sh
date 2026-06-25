#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
export LD_LIBRARY_PATH="$PWD/lib:$PWD/Thirdparty/DBoW2/lib:$PWD/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"

./Examples/Monocular/mono_euroc \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/Android_recording_2026_06_20_19_41_51.yaml \
  ../datasets/android/recording_2026_06_20_19_41_51 \
  Examples/Monocular-Inertial/Android_TimeStamps/recording_2026_06_20_19_41_51.txt \
  android_recording_2026_06_20_19_41_51_mono
