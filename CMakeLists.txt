cmake_minimum_required(VERSION 3.13.1)

project(microla VERSION 1.0.0 LANGUAGES CXX)

# Interface library (header-only)
add_library(microla INTERFACE)
add_library(microla::microla ALIAS microla)

# Include directories
target_include_directories(microla INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Require C++11
target_compile_features(microla INTERFACE cxx_std_11)

# Embedded profile (small footprint defaults)
option(MICROLA_EMBEDDED "Enable embedded-friendly defaults" OFF)

# Autodetect SIMD where available
option(MICROLA_AUTODETECT_SIMD "Autodetect SIMD features (NEON/MVE/AVX)" ON)
if(MICROLA_AUTODETECT_SIMD)
    target_compile_definitions(microla INTERFACE MICROLA_AUTODETECT_SIMD)
endif()

# Exceptions / RTTI controls
option(MICROLA_DISABLE_EXCEPTIONS "Disable C++ exceptions" OFF)
option(MICROLA_DISABLE_RTTI "Disable RTTI" OFF)

if(MICROLA_EMBEDDED)
    target_compile_definitions(microla INTERFACE MICROLA_EMBEDDED)
    set(MICROLA_DISABLE_EXCEPTIONS ON)
    set(MICROLA_DISABLE_RTTI ON)
endif()

if(MICROLA_DISABLE_EXCEPTIONS)
    target_compile_definitions(microla INTERFACE MICROLA_NO_EXCEPTIONS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "IAR")
        target_compile_options(microla INTERFACE --no_exceptions)
    elseif(MSVC)
        target_compile_options(microla INTERFACE /EHs- /EHc-)
    else()
        target_compile_options(microla INTERFACE -fno-exceptions)
    endif()
endif()

if(MICROLA_DISABLE_RTTI)
    target_compile_definitions(microla INTERFACE MICROLA_NO_RTTI)
    if(CMAKE_CXX_COMPILER_ID MATCHES "IAR")
        target_compile_options(microla INTERFACE --no_rtti)
    elseif(MSVC)
        target_compile_options(microla INTERFACE /GR-)
    else()
        target_compile_options(microla INTERFACE -fno-rtti)
    endif()
endif()

# Compiler-specific settings
if(CMAKE_CXX_COMPILER_ID MATCHES "IAR")
    # IAR Embedded Workbench for ARM
    # Enable C++11 support
    target_compile_options(microla INTERFACE --c++ --cpp_init_routine=_Z41__static_initialization_and_destruction_0ii)
    # Suppress specific warnings if needed
    # target_compile_options(microla INTERFACE --diag_suppress=Pe111,Pe177)
endif()

# Optional NEON support (ARM Cortex-A, Apple Silicon, ARM64)
option(MICROLA_ENABLE_NEON "Enable ARM NEON optimizations" OFF)
if(MICROLA_ENABLE_NEON)
    target_compile_definitions(microla INTERFACE CONFIG_MICROLA_NEON)
    # Enable NEON on ARM platforms
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)")
        if(CMAKE_CXX_COMPILER_ID MATCHES "IAR")
            # IAR: NEON is typically enabled via project settings or --fpu=VFPv4_sp
            target_compile_options(microla INTERFACE --fpu=VFPv4_sp)
        else()
            target_compile_options(microla INTERFACE -mfpu=neon)
        endif()
    endif()
endif()

# Optional CMSIS-DSP support (ARM Cortex-M)
option(MICROLA_ENABLE_CMSIS "Enable CMSIS-DSP optimizations" OFF)
if(MICROLA_ENABLE_CMSIS)
    target_compile_definitions(microla INTERFACE CONFIG_MICROLA_CMSIS)
    # User must link CMSIS::DSP in their project
endif()

# Optional: Build examples
option(MICROLA_LINEAR_BUILD_EXAMPLES "Build example programs" OFF)
if(MICROLA_LINEAR_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Optional: Build tests
option(MICROLA_LINEAR_BUILD_TESTS "Build test programs" OFF)
if(MICROLA_LINEAR_BUILD_TESTS)
    enable_testing()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
        add_subdirectory(tests)
    endif()
endif()

# Optional: Build benchmarks
option(MICROLA_LINEAR_BUILD_BENCHMARKS "Build benchmark programs" OFF)
if(MICROLA_LINEAR_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

# Optional: Build documentation
option(MICROLA_LINEAR_BUILD_DOCS "Build Doxygen documentation" OFF)
if(MICROLA_LINEAR_BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        message(STATUS "Doxygen found, documentation will be built")
        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
    else()
        message(WARNING "Doxygen not found, documentation will not be built")
    endif()
endif()

# Installation
include(GNUInstallDirs)
install(DIRECTORY include/microla
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS microla
    EXPORT microla-targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT microla-targets
    FILE microla-targets.cmake
    NAMESPACE microla::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/microla
)

# Generate config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/microla-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/microla-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/microla
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/microla-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/microla-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/microla-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/microla
)
