if (ESP_PLATFORM)
    idf_component_register(
        SRCS "axp192.c"
        INCLUDE_DIRS "."
    )
endif()
