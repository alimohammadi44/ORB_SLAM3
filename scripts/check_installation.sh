#!/usr/bin/env bash
set -e

cd "$(dirname "$0")/.."

echo "Checking ORB-SLAM3 installation..."
echo

[ -f Vocabulary/ORBvoc.txt ] && echo "[OK] Vocabulary/ORBvoc.txt" || echo "[MISSING] Vocabulary/ORBvoc.txt"
[ -f lib/libORB_SLAM3.so ] && echo "[OK] lib/libORB_SLAM3.so" || echo "[MISSING] lib/libORB_SLAM3.so"

echo
echo "Monocular:"
ls -lh Examples/Monocular/mono_* 2>/dev/null || true

echo
echo "Monocular-Inertial:"
ls -lh Examples/Monocular-Inertial/mono_* 2>/dev/null || true
