# BMI270 component for ESP-IDF
idf_component_register(
    SRCS
        "bmi2.c"
        "bmi270.c"
        "port/bmi270_esp32_port.c"
        # 如需 OIS 功能再解开下一行
        # "bmi2_ois.c"
    INCLUDE_DIRS
        "."
    PRIV_INCLUDE_DIRS
        "port"
    REQUIRES
        driver        # ESP-IDF 自带 I2C/SPI 驱动
)
