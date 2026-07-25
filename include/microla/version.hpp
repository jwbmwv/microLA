// SPDX-License-Identifier: Apache-2.0
/// @file version.hpp
/// @brief MicroLA version information and API
/// @details Provides compile-time and runtime access to MicroLA version numbers.
/// @copyright Copyright (c) 2026 James Baldwin. AI-assisted — see NOTICE.
/// @author James Baldwin

#pragma once

/// @brief MicroLA major version number
#define MICROLA_VERSION_MAJOR 0

/// @brief MicroLA minor version number
#define MICROLA_VERSION_MINOR 0

/// @brief MicroLA patch version number
#define MICROLA_VERSION_PATCH 3

// Expand a version component before converting it to a string literal.
#define MICROLA_VERSION_STRINGIFY_IMPL(value) #value
#define MICROLA_VERSION_STRINGIFY(value) MICROLA_VERSION_STRINGIFY_IMPL(value)

/// @brief MicroLA version string
#define MICROLA_VERSION_STRING                       \
    MICROLA_VERSION_STRINGIFY(MICROLA_VERSION_MAJOR) \
    "." MICROLA_VERSION_STRINGIFY(MICROLA_VERSION_MINOR) "." MICROLA_VERSION_STRINGIFY(MICROLA_VERSION_PATCH)

/// @brief Combined version number (MAJOR * 10000 + MINOR * 100 + PATCH)
#define MICROLA_VERSION_NUMBER ((MICROLA_VERSION_MAJOR * 10000) + (MICROLA_VERSION_MINOR * 100) + MICROLA_VERSION_PATCH)

namespace microla
{

/// @brief Version information structure
struct Version
{
    int major;  ///< Major version number
    int minor;  ///< Minor version number
    int patch;  ///< Patch version number

    /// @brief Get the version as a string
    /// @return Version string in the format "major.minor.patch"
    static const char* string() { return MICROLA_VERSION_STRING; }

    /// @brief Get the combined version number
    /// @return Version number (major * 10000 + minor * 100 + patch)
    static int number() { return MICROLA_VERSION_NUMBER; }

    /// @brief Get the major version number
    /// @return Major version
    static int get_major() { return MICROLA_VERSION_MAJOR; }

    /// @brief Get the minor version number
    /// @return Minor version
    static int get_minor() { return MICROLA_VERSION_MINOR; }

    /// @brief Get the patch version number
    /// @return Patch version
    static int get_patch() { return MICROLA_VERSION_PATCH; }
};

/// @brief Get the library version as a string
/// @return Version string
inline const char* get_version_string()
{
    return Version::string();
}

/// @brief Get the library version as a number
/// @return Version number
inline int get_version_number()
{
    return Version::number();
}

/// @brief Check if the library version is at least the specified version
/// @param major Required major version
/// @param minor Required minor version
/// @param patch Required patch version
/// @return true if library version >= required version
inline bool version_at_least(int major, int minor, int patch)
{
    return MICROLA_VERSION_NUMBER >= ((major * 10000) + (minor * 100) + patch);
}

}  // namespace microla
