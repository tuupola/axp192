cmake_minimum_required(VERSION 3.10)
project(axp192 VERSION 0.7.0 LANGUAGES C)

if (ESP_PLATFORM)

    idf_component_register(
        SRCS "axp192.c"
        INCLUDE_DIRS "."
    )
    add_definitions("-DAXP192_INCLUDE_SDKCONFIG_H")

else()

    add_library(axp192 axp192.c)
    target_include_directories(axp192 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    # Tests (off by default)
    option(AXP192_BUILD_TESTS "Build tests" OFF)
    if (AXP192_BUILD_TESTS)
        enable_testing()
        add_subdirectory(tests)
    endif()

endif()