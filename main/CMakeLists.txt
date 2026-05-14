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
        "wifi_ap.c"
    INCLUDE_DIRS "."
    REQUIRES vesc esp_partition nvs_flash esp_timer bt
             esp_wifi esp_netif esp_event esp_http_server json
             app_update
)
