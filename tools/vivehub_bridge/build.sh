#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 2 ]]; then
  echo 'Usage: bash build.sh /absolute/VIVEHub-Linux/sdk /absolute/output/vt-vivehub-reader' >&2
  exit 2
fi
sdk=$(realpath "$1")
output=$2
[[ "$output" = /* ]] || { echo 'Output must be absolute' >&2; exit 2; }
[[ -f "$sdk/include/vut/vut_client.h" && -f "$sdk/lib/libvut_sdk.a" ]]
mkdir -p "$(dirname "$output")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I "$sdk/include" "$(dirname "$0")/sdk_reader.cpp" \
  "$sdk/lib/libvut_sdk.a" -lpthread -o "$output"
echo "Built $output (external HTC SDK; do not redistribute the linked binary without permission)"
