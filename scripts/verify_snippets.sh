#!/bin/bash
# Script to verify code snippets in documentation compile successfully

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$PROJECT_ROOT"

echo "=================================="
echo "Code Snippet Verification"
echo "=================================="
echo ""

# Run the Python script
python3 "$SCRIPT_DIR/verify_snippets.py" "$@"

exit_code=$?

if [ $exit_code -eq 0 ]; then
    echo ""
    echo "✓ Code snippet verification passed"
else
    echo ""
    echo "✗ Code snippet verification failed"
fi

exit $exit_code
