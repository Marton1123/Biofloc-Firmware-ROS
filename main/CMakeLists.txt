# ==============================================================================
# Biofloc Firmware - Main Component CMakeLists.txt
# ==============================================================================

idf_component_register(
    SRCS 
        "main.c"
        "sensors.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        micro_ros_espidf_component
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_adc
        esp_timer
        json
)
