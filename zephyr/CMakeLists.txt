# Zephyr-specific CMake configuration for MicroLA

if(CONFIG_MICROLA)
    zephyr_interface_library_named(microla)

    # Add include directory
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

    # Add compile definitions if debug mode is enabled
    if(CONFIG_MICROLA_DEBUG)
        zephyr_compile_definitions(MICROLA_DEBUG)
    endif()

    # Add compile definitions if NEON is enabled
    if(CONFIG_MICROLA_NEON)
        zephyr_compile_definitions(CONFIG_MICROLA_NEON)
        # Compatibility alias for non-Zephyr build switches
        zephyr_compile_definitions(MICROLA_ENABLE_NEON)
    endif()

    # Add compile definitions if CMSIS is enabled
    if(CONFIG_MICROLA_CMSIS)
        zephyr_compile_definitions(CONFIG_MICROLA_CMSIS)
        # Compatibility alias for non-Zephyr build switches
        zephyr_compile_definitions(MICROLA_ENABLE_CMSIS)
        # Link CMSIS-DSP if available in Zephyr
        if(CONFIG_CMSIS_DSP)
            zephyr_link_libraries(CMSIS)
        endif()
    endif()

    # Add compile definitions if MVE is enabled
    if(CONFIG_MICROLA_MVE)
        zephyr_compile_definitions(CONFIG_MICROLA_MVE)
        # Compatibility alias for non-Zephyr build switches
        zephyr_compile_definitions(MICROLA_ENABLE_MVE)
    endif()

    # Propagate long-double intermediate preference from Kconfig
    if(CONFIG_MICROLA_USE_LONG_DOUBLE_INTERMEDIATES)
        zephyr_compile_definitions(MICROLA_USE_LONG_DOUBLE_INTERMEDIATES=1)
    else()
        zephyr_compile_definitions(MICROLA_USE_LONG_DOUBLE_INTERMEDIATES=0)
    endif()

    # Link interface library to app
    target_link_libraries(microla INTERFACE zephyr_interface)
endif()
