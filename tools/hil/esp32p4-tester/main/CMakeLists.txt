idf_component_register(
    SRCS "p4_hil_main.c"
    PRIV_REQUIRES bt driver esp_driver_i2c esp_driver_i2s esp_driver_pcnt esp_eth esp_event esp_hosted esp_netif esp_timer esp_wifi
                  nvs_flash spi_flash
    INCLUDE_DIRS ".")
