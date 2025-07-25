idf_component_register(
    SRCS "src/as5600.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer nvs_flash
)
