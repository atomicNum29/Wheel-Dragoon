#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d /tmp/wheel-dragoon-tests.XXXXXX)
cxx=${CXX:-g++}
flags=(-std=c++11 -Wall -Wextra -Werror -pedantic -Iinclude)
"$cxx" "${flags[@]}" test/test_md200t_can.cpp src/md200t_can.cpp -o "$build_dir/can"
"$cxx" "${flags[@]}" -std=c++14 -Itest/flexcan_fakes test/test_flexcan_rx.cpp -o "$build_dir/flexcan_rx"
"$cxx" "${flags[@]}" -Itest/fakes test/test_can_rx_queue_integration.cpp src/md200t_can.cpp -o "$build_dir/can_rx_queue"
"$cxx" "${flags[@]}" -Itest/fakes test/test_firmware_odometry.cpp src/md200t_can.cpp -o "$build_dir/firmware"
"$cxx" "${flags[@]}" -Itest/fakes test/test_timing_diagnostics.cpp src/md200t_can.cpp -o "$build_dir/timing"
"$build_dir/can"
"$build_dir/flexcan_rx"
"$build_dir/can_rx_queue"
"$build_dir/firmware"
"$build_dir/timing" "$build_dir/timing_fixture.bin"
python3 test/check_timing_fixture.py "$build_dir/timing_fixture.bin"
python3 -m unittest discover -s test
printf 'All host tests passed. Test executables: %s\n' "$build_dir"
