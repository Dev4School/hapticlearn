idf_component_register(SRCS "wakeword_main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2s esp-sr)