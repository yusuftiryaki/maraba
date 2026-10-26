#!/usr/bin/env bash
# Build kit_camera_shim against Insta360's Camera SDK (robot_kit/camera.py,
# docs/kit-soketi.md "360° kamera"). The SDK comes by application and never
# goes into this repo; point this at the unpacked package for THIS machine's
# architecture (Pi: an aarch64 package, e.g. the gcc-arm-11.2 one):
#
#   ./scripts/build_camera_shim.sh ~/sdk/CameraSDK-2.1.8-...-aarch64-...
#
# Installs ~/.robot/sdk/kit_camera_shim and ~/.robot/sdk/lib/libCameraSDK.so
# (rpath $ORIGIN/lib), which is where kit_camera looks by default.
set -euo pipefail

SDK="${1:?usage: $0 <unpacked CameraSDK directory>}"
OUT="${2:-$HOME/.robot/sdk}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${REPO}/ros2/src/robot_kit/shim/kit_camera_shim.cpp"

[[ -f "${SDK}/include/camera/camera.h" ]] || { echo "no include/camera/camera.h under ${SDK}" >&2; exit 1; }
[[ -f "${SDK}/lib/libCameraSDK.so" ]] || { echo "no lib/libCameraSDK.so under ${SDK}" >&2; exit 1; }

# The ELF header's e_machine (bytes 18-19): 3e 00 = x86-64, b7 00 = aarch64.
want="$(uname -m)"
have="$(od -An -tx1 -j18 -N2 "${SDK}/lib/libCameraSDK.so" | tr -d ' \n')"
case "${want}:${have}" in
  x86_64:3e00|aarch64:b700) ;;
  *) echo "libCameraSDK.so is machine ${have}, this machine is ${want}: wrong SDK package" >&2; exit 1 ;;
esac

mkdir -p "${OUT}/lib"
cp "${SDK}/lib/libCameraSDK.so" "${OUT}/lib/"
g++ -std=c++14 -O2 -Wall -o "${OUT}/kit_camera_shim" "${SRC}" \
    -I"${SDK}/include" -L"${OUT}/lib" -lCameraSDK -pthread -Wl,-rpath,'$ORIGIN/lib'
echo "built ${OUT}/kit_camera_shim"
echo "smoke test (no camera -> 'no camera found'):"
echo '{"id": 1, "cmd": "connect"}' | timeout 20 "${OUT}/kit_camera_shim" || true
