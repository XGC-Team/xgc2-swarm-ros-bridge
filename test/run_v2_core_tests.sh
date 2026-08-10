#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

required_commands=(g++ python3 clang-format diff shellcheck clang-tidy)
for required_command in "${required_commands[@]}"; do
  if ! command -v "$required_command" >/dev/null 2>&1; then
    echo "required v2 core gate tool is unavailable: $required_command" >&2
    exit 1
  fi
done

bridge_v2_test_dir="$(mktemp -d /tmp/swarm-ros-bridge-v2-tests.XXXXXX)"

cleanup() {
  case "$bridge_v2_test_dir" in
    /tmp/swarm-ros-bridge-v2-tests.*)
      find "$bridge_v2_test_dir" -depth -delete
      ;;
    *)
      echo "refusing to clean unexpected test directory: $bridge_v2_test_dir" >&2
      ;;
  esac
}
trap cleanup EXIT

common_flags=(
  "-std=c++17"
  "-Wall"
  "-Wextra"
  "-Wpedantic"
  "-Werror"
  "-Wconversion"
  "-Wsign-conversion"
  -I"$repo_dir/include"
  "$repo_dir/src/v2/protocol.cpp"
  "$repo_dir/test/v2_protocol_test.cpp"
)

g++ "${common_flags[@]}" -O2 -o "$bridge_v2_test_dir/v2_protocol_test"
"$bridge_v2_test_dir/v2_protocol_test"
python3 "$repo_dir/test/v2_reference_encoder_test.py" \
  "$bridge_v2_test_dir/v2_protocol_test" \
  "$repo_dir/test/golden_v2_vectors.txt"

g++ "${common_flags[@]}" -O1 -g -fno-omit-frame-pointer \
  -fsanitize=address,undefined \
  -o "$bridge_v2_test_dir/v2_protocol_sanitized_test"
ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  "$bridge_v2_test_dir/v2_protocol_sanitized_test"

format_files=(
  "$repo_dir/include/swarm_ros_bridge/v2/protocol.hpp"
  "$repo_dir/include/swarm_ros_bridge/v2/ros1_codec.hpp"
  "$repo_dir/src/v2/protocol.cpp"
  "$repo_dir/src/v2/ros1_codec.cpp"
  "$repo_dir/test/v2_protocol_test.cpp"
  "$repo_dir/test/v2_ros1_codec_test.cpp"
)
format_index=0
for format_file in "${format_files[@]}"; do
  formatted_file="$bridge_v2_test_dir/clang-format-${format_index}"
  clang-format "$format_file" >"$formatted_file"
  diff -u "$format_file" "$formatted_file"
  format_index=$((format_index + 1))
done

python3 - "$repo_dir/test/v2_reference_encoder_test.py" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
compile(path.read_text(encoding="utf-8"), str(path), "exec")
PY

shellcheck \
  "$repo_dir/test/run_v2_core_tests.sh" \
  "$repo_dir/test/v2_release_contract_test.sh" \
  "$repo_dir/.xgc2/scripts/package_debs.sh" \
  "$repo_dir/.xgc2/scripts/check_installed_packages.sh" \
  "$repo_dir/.xgc2/scripts/check_noetic_catkin_in_docker.sh" \
  "$repo_dir/.xgc2/scripts/build_debs_in_docker.sh" \
  "$repo_dir/.xgc2/scripts/install_scout_msgs_dependency.sh"
if command -v cppcheck >/dev/null 2>&1; then
  cppcheck --enable=warning,performance,portability --error-exitcode=1 \
    --std=c++17 --suppress=missingIncludeSystem \
    -I"$repo_dir/include" \
    "$repo_dir/src/v2/protocol.cpp" \
    "$repo_dir/test/v2_protocol_test.cpp"
fi
clang-tidy "$repo_dir/src/v2/protocol.cpp" \
  --checks='-*,clang-analyzer-*,bugprone-*,performance-*' \
  --warnings-as-errors='clang-analyzer-*,bugprone-*' \
  -- -std=c++17 -I"$repo_dir/include"

bash "$repo_dir/test/v2_release_contract_test.sh"

echo "swarm_ros_bridge v2 ROS-free core gates passed"
