idf_component_register(
    SRCS
        "crc.c"
        "buffer.c"
        "bldc_interface_can.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
