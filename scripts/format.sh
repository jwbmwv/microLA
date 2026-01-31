#!/bin/bash
# Format all C++ source files with clang-format
# Uses clang-format-18 for consistency with CI (Ubuntu 24.04 default)

set -e

CLANG_FORMAT=${CLANG_FORMAT:-clang-format-18}

if ! command -v "$CLANG_FORMAT" &>/dev/null; then
    echo "Error: $CLANG_FORMAT not found. Install with: sudo apt-get install clang-format-18" >&2
    exit 1
fi

echo "Formatting C++ files with $CLANG_FORMAT..."

# Find and format all .hpp and .cpp files
find include tests examples benchmarks -name '*.hpp' -o -name '*.cpp' | while read -r file; do
    echo "Formatting: $file"
    "$CLANG_FORMAT" -i "$file"
done

echo "Formatting complete!"
