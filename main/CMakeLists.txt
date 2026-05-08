idf_component_register(
    SRCS
        "main.c"
        "ble.c"
        "throttle.c"
        "failsafe.c"
        "led.c"
        "bms.c"
        "aux_output.c"
        "usb_serial.c"
    INCLUDE_DIRS "."
    REQUIRES vesc esp_partition nvs_flash esp_timer bt
)
