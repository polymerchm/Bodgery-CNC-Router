idf_component_register(SRCS "main.c"
        REQUIRES  esp_driver_ledc
                    PRIV_REQUIRES esp_adc
                    INCLUDE_DIRS ".")
