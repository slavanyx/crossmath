#!/usr/bin/env bash
# Build the Fortran core and run all tests.
set -euo pipefail
cd "$(dirname "$0")"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
echo "Done. Run the GUI with:  cd python && PYTHONPATH=. python -m bladecam.viewer"
