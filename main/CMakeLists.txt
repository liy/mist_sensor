idf_component_register(SRCS "main.c"
    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_adc
    INCLUDE_DIRS "."
    REQUIRES mist_messages mist_comm sht4x sgp40 sed
)
