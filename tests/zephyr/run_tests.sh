#!/bin/bash
# SPDX-License-Identifier: MIT
# Quick test runner script for MicroLA Zephyr tests

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}MicroLA Zephyr Test Runner${NC}"
echo -e "${BLUE}================================${NC}"
echo ""

# Default board
BOARD="${1:-native_posix}"

echo -e "${GREEN}Building tests for board: ${BOARD}${NC}"
echo ""

# Clean previous build if requested
if [ "$2" == "--clean" ] || [ "$2" == "-c" ]; then
    echo "Cleaning previous build..."
    rm -rf build
fi

# Build
echo "Building..."
if west build -b ${BOARD} -p auto; then
    echo -e "${GREEN}Build successful!${NC}"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Running tests...${NC}"
echo "================================"
echo ""

# Run tests
if west build -t run; then
    echo ""
    echo "================================"
    echo -e "${GREEN}Tests completed successfully!${NC}"
else
    echo ""
    echo "================================"
    echo -e "${RED}Tests failed!${NC}"
    exit 1
fi
