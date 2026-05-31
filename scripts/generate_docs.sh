#!/bin/bash
# Script to generate Doxygen documentation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$PROJECT_ROOT"

echo "=================================="
echo "Doxygen Documentation Generation"
echo "=================================="
echo ""

# Check if doxygen is installed
if ! command -v doxygen &> /dev/null; then
    echo "Error: doxygen is not installed"
    echo "Install with: sudo apt-get install doxygen graphviz"
    exit 1
fi

# Display doxygen version
echo "Doxygen version:"
doxygen --version
echo ""

# Clean previous output
if [ -d "docs/doxygen" ]; then
    echo "Cleaning previous output..."
    rm -rf docs/doxygen
fi

# Generate documentation
echo "Generating documentation..."
doxygen Doxyfile

exit_code=$?

if [ $exit_code -eq 0 ]; then
    echo ""
    echo "✓ Documentation generated successfully"
    echo "  Output: docs/doxygen/html/index.html"
    
    # Display statistics
    if [ -f "docs/doxygen/html/index.html" ]; then
        html_files=$(find docs/doxygen/html -name "*.html" | wc -l)
        echo "  Files generated: $html_files HTML files"
    fi
else
    echo ""
    echo "✗ Documentation generation failed"
fi

exit $exit_code
