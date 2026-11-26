idf_component_register(
    SRCS "main.cpp" "haptic.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)