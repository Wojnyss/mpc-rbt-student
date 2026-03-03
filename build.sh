#!/usr/bin/env bash
set -euo pipefail

# Always run from repo root (where this script lives)
cd "$(dirname "$0")"

if [[ -f CMakeLists.txt ]]; then
  mkdir -p build
  cmake -S . -B build
  cmake --build build -j
elif [[ -f Makefile || -f makefile ]]; then
  make -j
else
  echo "ERROR: No CMakeLists.txt or Makefile found in repo root."
  exit 1
fi

echo "Build finished."
