# components/bmi270/CMakeLists.txt
idf_component_register(
    SRCS
        "bmi2.c"
        "bmi270.c"
        "port/bmi270_esp32_port.c"
        # 如果要用 OIS，就把下面两行取消注释
        # "bmi2_ois.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "port"
    REQUIRES driver  # ESP-IDF 自带的 I2C / SPI 驱动
)

# 如想保持灵活，也能用 GLOB，但官方更推荐显式列文件
