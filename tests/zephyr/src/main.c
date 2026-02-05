// SPDX-License-Identifier: MIT
/// @file main.c
/// @brief Main entry point for MicroLA Zephyr tests
/// @copyright Copyright (c) 2026 James Baldwin

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

/**
 * @brief Main function - runs all test suites
 * 
 * This file serves as the entry point for the Zephyr test framework.
 * All test suites are automatically discovered and executed by ztest.
 * 
 * Test suites included:
 * - microla_vector: Vector operations tests
 * - microla_matrix: Matrix operations tests
 * - microla_quaternion: Quaternion operations tests
 * - microla_integration: Integration and combined functionality tests
 */

// The ztest framework will automatically discover and run all test suites
// No additional code needed here - test suites are registered in their respective files
