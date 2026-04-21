idf_component_register(
    SRCS
        "crc.c"
        "packet.c"
        "buffer.c"
        "bldc_interface.c"
        "bldc_interface_can.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
