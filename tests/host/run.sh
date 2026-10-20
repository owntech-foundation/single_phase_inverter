#!/bin/sh
# Builds the library and its control_library dependencies for the host and
# runs the tests. CMSIS-DSP and Zephyr logging are replaced by shims.
# usage: tests/host/run.sh [path to control_library]
set -e
D=$(cd "$(dirname "$0")" && pwd)
LIB=$(cd "$D/../.." && pwd)
CL=${1:-${CONTROL_LIBRARY:-$LIB/../control_library}}
CL=$(cd "$CL" && pwd)
OUT=$(mktemp -d)
g++ -std=c++17 -O0 -g -Wall -Wno-unused-variable \
    -I"$D/shim" -I"$LIB/src" -I"$CL/src" \
    "$CL/src/trigo.cpp" "$CL/src/transform.cpp" "$CL/src/pid.cpp" \
    "$LIB/src/sogi.cpp" "$LIB/src/singlePhaseInverter.cpp" \
    "$D/test_inverter.cpp" -lm -o "$OUT/test_inverter"
"$OUT/test_inverter"; R=$?
rm -rf "$OUT"
exit $R
