#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
rid="${1:-}"
case "$(uname -s)" in
  Darwin)
    if [[ -z "$rid" ]]; then
      [[ "$(uname -m)" == "arm64" ]] && rid="osx-arm64" || rid="osx-x64"
    fi
  ;;
  Linux) rid="${rid:-linux-x64}" ;;
  MINGW*|MSYS*|CYGWIN*) rid="${rid:-win-x64}" ;;
  *) echo "Unsupported OS"; exit 1 ;;
esac

build_dir="$root/native/build-stub-$rid"
cmake -S "$root/native" -B "$build_dir" -DMOTUS_USE_OMPL=OFF -DMOTUS_USE_FCL=OFF
cmake --build "$build_dir" --config Release

dest="$root/src/Motus.Native/runtimes/$rid/native"
mkdir -p "$dest"
case "$rid" in
  win-x64) cp "$build_dir/Release/motus_native.dll" "$dest/" 2>/dev/null || cp "$build_dir/motus_native.dll" "$dest/" ;;
  osx-*) cp "$build_dir/libmotus_native.dylib" "$dest/" ;;
  linux-x64) cp "$build_dir/libmotus_native.so" "$dest/" ;;
esac
echo "Stub -> $dest"
