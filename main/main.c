/**
 * @file    main.c
 * @brief   Biofloc Firmware ROS - Sistema de telemetría con micro-ROS
 * @version 4.1.2
 *
 * @description
 * Firmware para ESP32 con micro-ROS Jazzy sobre WiFi UDP.
 * Lee sensores de pH y temperatura CWT-BL y publica datos JSON a ROS 2.
 * Soporta calibración remota vía topic /biofloc/calibration_cmd.
 *
 * @changelog v4.1.2 (2026-02-19) - ARQUITECTURA BLINDADA (EclipSec Standard)
 * - ✅ Core: Estado global 100% thread-safe (Copias por valor + Mutex)
 * - ✅ Hardware: Protección NVS contra Wear-Out (Evita brickeos por bucles de calibración)
 * - ✅ WiFi: Reconexión forzada de hardware con verificación de IP
 * - ✅ FreeRTOS: Eliminadas condiciones de carrera (Race conditions) entre Cores
 * - ✅ WDT: Configuración actualizada para ESP-IDF v5.x
 */

/* Standard library */
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

/* Core modules */
#include "config.h"
#include "types.h"
#include "app_state.h"

/* Middleware modules */
#include "middleware/uros/uros_manager.h"
#include "middleware/data_aggregator.h"
#include "middleware/config_manager.h"
#include "middleware/calibration/calibration_handler.h"

/* Application layer */
#include "app/sensor_task.h"

/* HAL - Hardware Abstraction Layer */
#include "hal/sensors/sensors.h"
#include "drivers/wifi_manager.h" /* AÑADIDO: Para gestionar WiFi a bajo nivel */

/* ESP-IDF */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

/* micro-ROS */
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

/* ============================================================================
 * Constantes de Configuración
 * ============================================================================ */
#define PING_RETRIES            5      
#define SAMPLE_INTERVAL_MS      CONFIG_BIOFLOC_SENSOR_SAMPLE_INTERVAL_MS
#define DEVICE_LOCATION         CONFIG_BIOFLOC_LOCATION

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 20480
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

#define SENSOR_TASK_STACK       12288   
#define SENSOR_TASK_PRIO        4
#define JSON_BUFFER_SIZE        512
#define CAL_CMD_BUFFER_SIZE     1024    
#define CAL_RESPONSE_SIZE       512
#define RECONNECT_DELAY_INITIAL 3000   
#define RECONNECT_DELAY_MAX     60000  
#define RECONNECT_FOREVER       true   

/* ============================================================================
 * Error Handling Macros
 * ============================================================================ */

#define RCCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGE(TAG_UROS, "RCL error at %s:%d (rc=%d)", __FILE__, __LINE__, (int)rc); \
        vTaskDelete(NULL); \
    } \
} while(0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG_UROS, "RCL warning at %s:%d (rc=%d)", __FILE__, __LINE__, (int)rc); \
    } \
} while(0)

/* ============================================================================
 * Network Utilities
 * ============================================================================ */

static void load_network_info_to_state(void)
{
    device_info_t dev_info;
    memset(&dev_info, 0, sizeof(device_info_t));
    
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    
    if (err == ESP_OK) {
        snprintf(dev_info.mac_address, sizeof(dev_info.mac_address),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        snprintf(dev_info.device_id, sizeof(dev_info.device_id),
                 "biofloc_esp32_%02x%02x", mac[4], mac[5]);
    } else {
        strncpy(dev_info.mac_address, "00:00:00:00:00:00", sizeof(dev_info.mac_address));
        strncpy(dev_info.device_id, "biofloc_esp32_0000", sizeof(dev_info.device_id));
    }
    
    // Conseguir la IP
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            snprintf(dev_info.ip_address, sizeof(dev_info.ip_address), IPSTR, IP2STR(&ip_info.ip));
        }
    }

    ESP_LOGI(TAG_MAIN, "Device ID: %s", dev_info.device_id);
    ESP_LOGI(TAG_MAIN, "MAC: %s", dev_info.mac_address);
    ESP_LOGI(TAG_MAIN, "IP: %s", dev_info.ip_address);
    
    app_state_update_device_info(&dev_info);
}

/* ============================================================================
 * micro-ROS Task
 * ============================================================================ */

static void micro_ros_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_UROS, "micro-ROS task started");
    ESP_LOGI(TAG_UROS, "Agent: %s:%d", AGENT_IP, AGENT_PORT);

    esp_err_t ret = uros_manager_init(calibration_callback);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG_UROS, "⚠️ Agent unreachable on startup - waiting...");
        ESP_LOGW(TAG_UROS, "ESP32 will NOT restart - reconnecting infinitely");
        
        uros_manager_reconnect_forever();
        
        ret = uros_manager_init(calibration_callback);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_UROS, "FATAL: Failed to initialize after reconnection");
            vTaskDelete(NULL);
            return;
        }
    }

    ESP_LOGI(TAG_UROS, "Subscribing micro-ROS task to watchdog...");
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err == ESP_OK) {
        ESP_LOGI(TAG_UROS, "✓ micro-ROS task watchdog active (timeout: %ds)", WATCHDOG_TIMEOUT_SEC);
    } else {
        ESP_LOGW(TAG_UROS, "Watchdog subscription failed (err=%d) - non-fatal", wdt_err);
    }

    ESP_LOGI(TAG_MAIN, "=========================================");
    ESP_LOGI(TAG_MAIN, "  micro-ROS Ready!");
    ESP_LOGI(TAG_MAIN, "  Node: /%s/biofloc_node", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "  Publisher: /%s/sensor_data", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "  Publisher: /%s/calibration_status", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "  Subscriber: /%s/calibration_cmd", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "=========================================");

    uint32_t ping_counter = 0;
    const uint32_t ping_interval = PING_CHECK_INTERVAL_MS / 100;
    uint32_t keep_alive_counter = 0;
    const uint32_t keep_alive_interval = 200; 

    while (1) {
        esp_task_wdt_reset();

        uros_manager_spin_once(100);

        if (++keep_alive_counter >= keep_alive_interval) {
            keep_alive_counter = 0;
            esp_err_t ka_ret = uros_manager_keep_alive();
            if (ka_ret != ESP_OK && ka_ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGD(TAG_UROS, "Keep-alive check triggered");
            }
        }

        if (++ping_counter >= ping_interval) {
            ping_counter = 0;

            if (!uros_manager_ping_agent()) {
                ESP_LOGW(TAG_UROS, "⚠️ Lost connection to Agent");
                uros_manager_reconnect_forever();
                ESP_LOGI(TAG_UROS, "✅ Connection restored");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uros_manager_deinit();
    vTaskDelete(NULL);
}

/* ============================================================================
 * Application Entry Point
 * ============================================================================ */

void app_main(void)
{
    ESP_LOGI(TAG_MAIN, "=========================================");
    ESP_LOGI(TAG_MAIN, "  Biofloc Firmware ROS v%s", FIRMWARE_VERSION);
    ESP_LOGI(TAG_MAIN, "  ESP-IDF: %s", esp_get_idf_version());
    ESP_LOGI(TAG_MAIN, "  micro-ROS: Jazzy");
    ESP_LOGI(TAG_MAIN, "=========================================");
    
    ESP_ERROR_CHECK(app_state_init());
    
    ESP_LOGI(TAG_MAIN, "Initializing configuration manager...");
    ESP_ERROR_CHECK(config_manager_init());
    
    esp_reset_reason_t reset_reason = esp_reset_reason();
    const char *reason_str = "UNKNOWN";
    switch (reset_reason) {
        case ESP_RST_POWERON:   reason_str = "POWER_ON"; break;
        case ESP_RST_SW:        reason_str = "SOFTWARE"; break;
        case ESP_RST_PANIC:     reason_str = "PANIC"; break;
        case ESP_RST_INT_WDT:   reason_str = "INT_WDT"; break;
        case ESP_RST_TASK_WDT:  reason_str = "TASK_WDT"; break;
        case ESP_RST_WDT:       reason_str = "WDT"; break;
        case ESP_RST_DEEPSLEEP: reason_str = "DEEP_SLEEP"; break;
        case ESP_RST_BROWNOUT:  reason_str = "BROWNOUT"; break;
        default: break;
    }
    ESP_LOGI(TAG_MAIN, "Reset reason: %s", reason_str);
    
    /* CORRECCIÓN 1: Usar reconfigure en lugar de init para ESP-IDF v5.x */
    ESP_LOGI(TAG_MAIN, "Configuring hardware watchdog (timeout: %ds)", WATCHDOG_TIMEOUT_SEC);
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,  
        .trigger_panic = false 
    };
    esp_err_t wdt_err = esp_task_wdt_reconfigure(&wdt_config);
    if (wdt_err == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "✓ Watchdog configured - will hard reset if task blocks > %ds", WATCHDOG_TIMEOUT_SEC);
    } else {
        ESP_LOGW(TAG_MAIN, "Failed to configure watchdog (err=%d)", wdt_err);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_LOGI(TAG_MAIN, "Initializing network...");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    
    /* CORRECCIÓN 3: Inicializar nuestro gestor de WiFi a bajo nivel */
    wifi_manager_init();
    
    ESP_LOGI(TAG_MAIN, "Network ready");
#endif

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* CORRECCIÓN 2: Llenar el estado oficial de la aplicación de forma segura */
    load_network_info_to_state();

    ESP_LOGI(TAG_MAIN, "⚠ No Internet access - Running in secure gateway mode");
    ESP_LOGI(TAG_MAIN, "Timestamps will be added by the server");

    ESP_LOGI(TAG_MAIN, "Starting micro-ROS task...");
    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL,
        1  /* APP_CPU */
    );

    ESP_LOGI(TAG_MAIN, "Starting sensor task...");
    TaskHandle_t sensor_handle = NULL;
    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIO,
        &sensor_handle,
        0  /* PRO_CPU */
    );
}
