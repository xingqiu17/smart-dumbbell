# BMM150 component for ESP-IDF
idf_component_register(
    SRCS
        "bmm150.c"
        "port/bmm150_esp32_port.c"
    INCLUDE_DIRS
        "."
    PRIV_INCLUDE_DIRS
        "port"
    REQUIRES
        driver
)
