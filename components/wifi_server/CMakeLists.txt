idf_component_register(SRCS "wifi_server.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi esp_http_server esp_netif nvs_flash espressif__cjson)
