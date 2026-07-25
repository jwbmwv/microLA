#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <build-directory>" >&2
    exit 1
fi

build_dir="$1"
size_tool="${MICROLA_SIZE_TOOL:-arm-none-eabi-size}"
report_file="$build_dir/microla-target-resource-report.txt"

if ! command -v "$size_tool" >/dev/null 2>&1; then
    echo "ERROR: size tool not found: $size_tool" >&2
    exit 1
fi

mapfile -t object_files < <(find "$build_dir" -type f \
    -path '*CMakeFiles/microla_target_smoke.dir/*' \( -name '*.o' -o -name '*.obj' \) -print | sort)
mapfile -t stack_files < <(find "$build_dir" -type f \
    -path '*CMakeFiles/microla_target_smoke.dir/*' -name '*.su' -print | sort)

if [[ ${#object_files[@]} -eq 0 ]]; then
    echo "ERROR: no target smoke object files found in $build_dir" >&2
    exit 1
fi

{
    echo "MicroLA target resource report"
    echo "Build directory: $build_dir"
    echo "Size tool: $size_tool"
    echo
    echo "Object section sizes (not a linked firmware image):"
    "$size_tool" "${object_files[@]}"
    echo
    echo "Compiler stack-usage reports (bytes per emitted function):"
    if [[ ${#stack_files[@]} -eq 0 ]]; then
        echo "No .su files were produced by this compiler."
    else
        for stack_file in "${stack_files[@]}"; do
            echo "--- ${stack_file#$build_dir/}"
            cat "$stack_file"
        done
    fi
} | tee "$report_file"
