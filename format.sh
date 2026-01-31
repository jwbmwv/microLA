#!/bin/bash
# Format all C++ source files with clang-format

set -e

echo "Formatting C++ files with clang-format..."

# Find and format all .hpp and .cpp files
find include tests examples benchmarks -name '*.hpp' -o -name '*.cpp' | while read -r file; do
    echo "Formatting: $file"
    clang-format -i "$file"
done

echo "Formatting complete!"
