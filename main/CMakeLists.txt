# ==============================================================================
# Biofloc Firmware v4.1.2 - Main Component CMakeLists.txt
# Arquitectura modular con separación de responsabilidades
# ==============================================================================

idf_component_register(
    SRCS 
        # Core
        "main.c"
        "core/app_state.c"
        
        # Drivers
        "drivers/wifi_manager.c"
        
        # Middleware
        "middleware/uros/uros_manager.c"
        "middleware/data_aggregator.c"
        "middleware/config_manager.c"
        "middleware/calibration/calibration_handler.c"
        
        # HAL - Hardware Abstraction Layer
        "hal/sensors/sensors.c"
        
        # Application Layer (CORRECCIÓN: Ruta ajustada para el Linker)
        "app/sensor_task.c"
        
    INCLUDE_DIRS 
        "."
        "core"
        "hal/sensors"
        "hal/storage"
        "drivers"
        "middleware/uros"
        "middleware/calibration"
        "app"
        
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
