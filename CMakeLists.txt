cmake_minimum_required(VERSION 3.22)

if(ESP_PLATFORM)
    file(GLOB_RECURSE YOBA_SOURCES
        "src/*.*"
    )

    idf_component_register(
        SRCS ${YOBA_SOURCES}
        INCLUDE_DIRS src
        REQUIRES
            esp_timer
            esp_driver_i2c
            esp_driver_gpio
    )

    target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++26)

    return()
endif()