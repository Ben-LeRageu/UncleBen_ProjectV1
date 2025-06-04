idf_component_register(SRCS "Cuisson.c" "test_touch-buttons.c" "main.c" "http_server.c" "wifi_setup.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver is31fl3731 ht16k33 esp_http_server cjson esp_netif esp_event esp_wifi nvs_flash
                    )
