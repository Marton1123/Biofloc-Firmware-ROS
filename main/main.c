/**
 * @file    main.c
 * @brief   Biofloc Firmware ROS - Sistema de telemetría con micro-ROS
 * @version 4.0.0
 *
 * @description
 *   Firmware para ESP32 con micro-ROS Jazzy sobre WiFi UDP.
 *   Lee sensores de pH y temperatura CWT-BL y publica datos JSON a ROS 2.
 *   Soporta calibración remota vía topic /biofloc/calibration_cmd.
 *
 * @changelog v4.0.0 (2026-02-18) - REFACTORIZACIÓN COMPLETA
 *   - 🏗️ Arquitectura modular con separación de responsabilidades
 *   - ✅ Core: config.h, types.h, app_state.c/.h
 *   - ✅ Clean Code + SOLID + Security best practices
 *   - ✅ Estado global thread-safe con mutex
 *   - ✅ Zero magic numbers, todo en config.h
 * 
 * @changelog v3.6.5 (2026-02-18)
 *   - ✅ Pausa de sensor_data durante calibración para evitar crash
 *   - ✅ g_calibrating flag para controlar publicación
 * 
 * @changelog v3.6.4 (2026-02-17)
 *   - ✅ Stack de sensor_task aumentado a 12KB
 *   - ✅ Buffer cleanup entre calibraciones
 *   - ✅ Monitoreo de ambos stacks
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
 * NOTA: Las constantes ahora están en core/config.h
 * ============================================================================ */
#define PING_RETRIES            5      /* Aumentado de 3 a 5 reintentos */
#define SAMPLE_INTERVAL_MS      CONFIG_BIOFLOC_SENSOR_SAMPLE_INTERVAL_MS
#define DEVICE_LOCATION         CONFIG_BIOFLOC_LOCATION

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 20480
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

#define SENSOR_TASK_STACK       12288   /* Increased from 8KB to 12KB (v3.6.4) to prevent stack overflow during rcl_publish() + cJSON */
#define SENSOR_TASK_PRIO        4
#define JSON_BUFFER_SIZE        512
#define CAL_CMD_BUFFER_SIZE     1024    /* Subscriber buffer: must be > largest incoming JSON */
#define CAL_RESPONSE_SIZE       512
/* PING_CHECK_INTERVAL_MS definido en core/config.h */
#define RECONNECT_DELAY_INITIAL 3000   /* Delay inicial: 3s */
#define RECONNECT_DELAY_MAX     60000  /* Max delay: 60s (aumentado de 30s) */
#define RECONNECT_FOREVER       true   /* CRÍTICO: Nunca reiniciar, reconectar infinitamente */

/* ============================================================================
 * Logging Tags - Ahora en core/config.h
 * ============================================================================ */
// TAG_MAIN, TAG_UROS, TAG_SENSOR definidos en config.h

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
 * Application State
 * ============================================================================ */

typedef struct {
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_publisher_t sensor_publisher;
    rcl_publisher_t calibration_response_publisher;
    rcl_subscription_t calibration_subscriber;
    std_msgs__msg__String sensor_msg;
    std_msgs__msg__String calibration_cmd_msg;
    std_msgs__msg__String calibration_response_msg;
    bool initialized;
} microros_context_t;

// TODO FASE 2: Refactorizar para usar app_state_*() API completamente
typedef struct {
    char device_id[24];
    char mac_address[18];
    char ip_address[16];
    bool microros_ready;
} local_app_state_t;

// v4.0.0: g_uros_ctx eliminated - now managed by uros_manager module
static local_app_state_t g_app_state = {0};  // TEMPORAL - migrar a app_state API

/* ============================================================================
 * Network Utilities
 * ============================================================================ */

static void get_network_info(void)
{
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    
    if (err == ESP_OK) {
        snprintf(g_app_state.mac_address, sizeof(g_app_state.mac_address),
                 "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        
        snprintf(g_app_state.device_id, sizeof(g_app_state.device_id),
                 "biofloc_esp32_%02x%02x", mac[4], mac[5]);
    } else {
        strncpy(g_app_state.mac_address, "00:00:00:00:00:00", sizeof(g_app_state.mac_address));
        strncpy(g_app_state.device_id, "biofloc_esp32_0000", sizeof(g_app_state.device_id));
    }

    ESP_LOGI(TAG_MAIN, "Device ID: %s", g_app_state.device_id);
    ESP_LOGI(TAG_MAIN, "MAC: %s", g_app_state.mac_address);
}

/* ============================================================================
 * micro-ROS Task
 * ============================================================================ */

static void micro_ros_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_UROS, "micro-ROS task started");
    ESP_LOGI(TAG_UROS, "Agent: %s:%d", AGENT_IP, AGENT_PORT);

    // v4.0.0: Initialize micro-ROS via uros_manager (callback antes de executor)
    esp_err_t ret = uros_manager_init(calibration_callback);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG_UROS, "⚠️ Agent unreachable on startup - waiting...");
        ESP_LOGW(TAG_UROS, "ESP32 will NOT restart - reconnecting infinitely");
        
        // Wait forever for agent
        uros_manager_reconnect_forever();
        
        // Retry initialization
        ret = uros_manager_init(calibration_callback);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_UROS, "FATAL: Failed to initialize after reconnection");
            vTaskDelete(NULL);
            return;
        }
    }

    /* Subscribe micro-ROS task to watchdog (CRITICAL: prevents silent deadlocks) */
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
    const uint32_t keep_alive_interval = 200;  // Llamar keep_alive cada ~2 segundos (200 * 10ms)

    // Main spin loop with connection monitoring
    while (1) {
        // Feed watchdog FIRST
        esp_task_wdt_reset();

        // Spin executor
        uros_manager_spin_once(100);

        // Keep-alive automático: mantiene sesión viva contra timeouts
        if (++keep_alive_counter >= keep_alive_interval) {
            keep_alive_counter = 0;
            esp_err_t ka_ret = uros_manager_keep_alive();
            if (ka_ret != ESP_OK && ka_ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGD(TAG_UROS, "Keep-alive check triggered");
            }
        }

        // Periodic Agent ping
        if (++ping_counter >= ping_interval) {
            ping_counter = 0;

            if (!uros_manager_ping_agent()) {
                ESP_LOGW(TAG_UROS, "⚠️ Lost connection to Agent");
                
                // Reconnect forever
                uros_manager_reconnect_forever();
                
                ESP_LOGI(TAG_UROS, "✅ Connection restored");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Cleanup (unreachable but good practice)
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
    
    /* v4.0.0: Initialize application state manager */
    ESP_ERROR_CHECK(app_state_init());
    
    /* v4.0.0: Initialize configuration manager with defaults */
    ESP_LOGI(TAG_MAIN, "Initializing configuration manager...");
    ESP_ERROR_CHECK(config_manager_init());
    
    /* Log reset reason (for diagnostics) */
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
    
    /* Initialize Hardware Task Watchdog Timer (CRITICAL for zombie state prevention) */
    ESP_LOGI(TAG_MAIN, "Initializing hardware watchdog (timeout: %ds)", WATCHDOG_TIMEOUT_SEC);
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,  /* Don't watch idle tasks */
        .trigger_panic = false  /* Just reset, don't panic */
    };
    esp_err_t wdt_err = esp_task_wdt_init(&wdt_config);
    if (wdt_err == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "✓ Watchdog initialized - will hard reset if task blocks > %ds", WATCHDOG_TIMEOUT_SEC);
    } else {
        ESP_LOGW(TAG_MAIN, "Failed to initialize watchdog (err=%d)", wdt_err);
    }

    /* Initial delay for power stabilization */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Initialize network interface */
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_LOGI(TAG_MAIN, "Initializing network...");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_LOGI(TAG_MAIN, "Network ready");
#endif

    /* Wait for WiFi to stabilize */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Get device network info */
    get_network_info();

    ESP_LOGI(TAG_MAIN, "⚠ No Internet access - Running in secure gateway mode");
    ESP_LOGI(TAG_MAIN, "Timestamps will be added by the server");

    /* Create micro-ROS task on APP_CPU */
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

    /* Create sensor task on PRO_CPU */
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
