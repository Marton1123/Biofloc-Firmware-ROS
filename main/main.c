/**
 * @file    main.c
 * @brief   Biofloc Firmware ROS - Sistema de telemetría con micro-ROS
 * @version 2.2.0 - Added pH calibration system
 *
 * @description
 *   Firmware para ESP32 con micro-ROS Jazzy sobre WiFi UDP.
 *   Lee sensores de pH y temperatura CWT-BL y publica datos JSON a ROS 2.
 *
 * @author  @Marton1123 (https://github.com/Marton1123)
 * @date    2026
 */

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "sensors.h"

/* ============================================================================
 * Configuration Constants
 * ============================================================================ */

#define FIRMWARE_VERSION        "2.1.0"

#define AGENT_IP                CONFIG_BIOFLOC_AGENT_IP
#define AGENT_PORT              CONFIG_BIOFLOC_AGENT_PORT
#define ROS_NAMESPACE           CONFIG_BIOFLOC_ROS_NAMESPACE
#define PING_TIMEOUT_MS         CONFIG_BIOFLOC_PING_TIMEOUT_MS
#define PING_RETRIES            CONFIG_BIOFLOC_PING_RETRIES
#define SAMPLE_INTERVAL_MS      CONFIG_BIOFLOC_SENSOR_SAMPLE_INTERVAL_MS
#define DEVICE_LOCATION         CONFIG_BIOFLOC_LOCATION

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 16000
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

#define SENSOR_TASK_STACK       8192
#define SENSOR_TASK_PRIO        4
#define JSON_BUFFER_SIZE        512
#define PING_CHECK_INTERVAL_MS  30000
#define RECONNECT_ATTEMPTS      10
#define RECONNECT_DELAY_MS      2000
#define RECONNECT_BACKOFF_MAX   30000  /* Max 30s between retries */
#define RESTART_DELAY_MS        3000

/* ============================================================================
 * Logging Tags
 * ============================================================================ */

static const char *TAG_MAIN   = "BIOFLOC";
static const char *TAG_UROS   = "UROS";
static const char *TAG_SENSOR = "SENSOR";

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
    std_msgs__msg__String sensor_msg;
    bool initialized;
} microros_context_t;

typedef struct {
    char device_id[24];
    char mac_address[18];
    char ip_address[16];
    bool microros_ready;
} app_state_t;

static microros_context_t g_uros_ctx = {0};
static app_state_t g_app_state = {0};
static char g_agent_port_str[8];

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
 * micro-ROS Agent Connection
 * ============================================================================ */

static bool ping_agent(rmw_init_options_t *rmw_options, int timeout_ms, int retries)
{
    for (int attempt = 1; attempt <= retries; attempt++) {
        ESP_LOGI(TAG_UROS, "Ping attempt %d/%d", attempt, retries);

        if (rmw_uros_ping_agent_options(timeout_ms, 1, rmw_options) == RMW_RET_OK) {
            ESP_LOGI(TAG_UROS, "Agent is ONLINE");
            return true;
        }

        if (attempt < retries) {
            ESP_LOGW(TAG_UROS, "Ping failed, retrying in 2s...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    return false;
}

static bool try_reconnect(void)
{
    uint32_t delay_ms = RECONNECT_DELAY_MS;
    
    for (int i = 0; i < RECONNECT_ATTEMPTS; i++) {
        ESP_LOGI(TAG_UROS, "Reconnection attempt %d/%d (delay: %lums)", 
                 i + 1, RECONNECT_ATTEMPTS, delay_ms);
        
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
            ESP_LOGI(TAG_UROS, "✅ Reconnected successfully");
            return true;
        }
        
        /* Exponential backoff: 2s, 4s, 8s, 16s, 30s, 30s... */
        delay_ms = (delay_ms * 2 > RECONNECT_BACKOFF_MAX) 
                   ? RECONNECT_BACKOFF_MAX 
                   : delay_ms * 2;
    }
    
    ESP_LOGE(TAG_UROS, "❌ Failed to reconnect after %d attempts", RECONNECT_ATTEMPTS);
    return false;
}

/* ============================================================================
 * Sensor Task
 * ============================================================================ */

static void sensor_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG_SENSOR, "Sensor task started");
    ESP_LOGI(TAG_SENSOR, "Sample interval: %d ms", SAMPLE_INTERVAL_MS);
    ESP_LOGI(TAG_SENSOR, "Location: %s", DEVICE_LOCATION);

    esp_err_t ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "Failed to initialize sensors (err=%d)", ret);
        vTaskDelete(NULL);
        return;
    }

    /* Apply pH calibration (3-point calibration with pH 4.01, 6.86, 9.18) */
    ret = sensors_calibrate_ph_manual(2.559823f, 0.469193f);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_SENSOR, "✓ pH calibration applied: R²=0.9997, max_err=0.049pH");
    } else {
        ESP_LOGW(TAG_SENSOR, "Failed to apply pH calibration");
    }

    sensors_get_device_info(g_app_state.device_id, 
                            g_app_state.mac_address, 
                            g_app_state.ip_address);

    char json_buffer[JSON_BUFFER_SIZE];
    sensors_data_t sensor_data;

    while (!g_app_state.microros_ready) {
        ESP_LOGD(TAG_SENSOR, "Waiting for micro-ROS...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG_SENSOR, "Starting sensor readings");

    while (1) {
        ret = sensors_read_all(&sensor_data);

        if (ret == ESP_OK) {
            int json_len = sensors_to_json(&sensor_data, json_buffer,
                                           sizeof(json_buffer),
                                           g_app_state.device_id,
                                           DEVICE_LOCATION);

            if (json_len > 0 && g_uros_ctx.initialized) {
                ESP_LOGI(TAG_SENSOR, "pH: %.2f (%.3fV) | Temp: %.1f C (%.3fV)",
                         sensor_data.ph.value, sensor_data.ph.voltage,
                         sensor_data.temperature.value, sensor_data.temperature.voltage);

                g_uros_ctx.sensor_msg.data.data = json_buffer;
                g_uros_ctx.sensor_msg.data.size = (size_t)json_len;
                g_uros_ctx.sensor_msg.data.capacity = JSON_BUFFER_SIZE;

                rcl_ret_t pub_ret = rcl_publish(&g_uros_ctx.sensor_publisher,
                                                &g_uros_ctx.sensor_msg, NULL);
                if (pub_ret != RCL_RET_OK) {
                    ESP_LOGW(TAG_SENSOR, "Failed to publish (rc=%d)", (int)pub_ret);
                }
            }
        } else {
            ESP_LOGW(TAG_SENSOR, "Sensor read failed (err=%d)", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

/* ============================================================================
 * micro-ROS Task
 * ============================================================================ */

static void micro_ros_task(void *arg)
{
    (void)arg;

    snprintf(g_agent_port_str, sizeof(g_agent_port_str), "%d", AGENT_PORT);

    ESP_LOGI(TAG_UROS, "micro-ROS task started");
    ESP_LOGI(TAG_UROS, "Agent: %s:%s", AGENT_IP, g_agent_port_str);

    g_uros_ctx.allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, g_uros_ctx.allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, g_agent_port_str, rmw_options));

    ESP_LOGI(TAG_UROS, "Pinging Agent (timeout: %dms, retries: %d)",
             PING_TIMEOUT_MS, PING_RETRIES);

    if (!ping_agent(rmw_options, PING_TIMEOUT_MS, PING_RETRIES)) {
        ESP_LOGE(TAG_UROS, "Agent unreachable - restarting");
        vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
        esp_restart();
    }
#endif

    ESP_LOGI(TAG_UROS, "Initializing micro-ROS support");
    RCCHECK(rclc_support_init_with_options(&g_uros_ctx.support, 0, NULL,
                                           &init_options, &g_uros_ctx.allocator));

    ESP_LOGI(TAG_UROS, "Creating node: /%s/biofloc_node", ROS_NAMESPACE);
    RCCHECK(rclc_node_init_default(&g_uros_ctx.node, "biofloc_node",
                                   ROS_NAMESPACE, &g_uros_ctx.support));

    ESP_LOGI(TAG_UROS, "Creating publisher: /%s/sensor_data", ROS_NAMESPACE);
    RCCHECK(rclc_publisher_init_default(
        &g_uros_ctx.sensor_publisher,
        &g_uros_ctx.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "sensor_data"
    ));

    RCCHECK(rclc_executor_init(&g_uros_ctx.executor,
                               &g_uros_ctx.support.context, 1,
                               &g_uros_ctx.allocator));

    g_uros_ctx.initialized = true;
    g_app_state.microros_ready = true;

    ESP_LOGI(TAG_MAIN, "=========================================");
    ESP_LOGI(TAG_MAIN, "  micro-ROS Ready!");
    ESP_LOGI(TAG_MAIN, "  Node: /%s/biofloc_node", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "  Topic: /%s/sensor_data", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "=========================================");

    uint32_t ping_counter = 0;
    const uint32_t ping_interval = PING_CHECK_INTERVAL_MS / 100;

    while (1) {
        rclc_executor_spin_some(&g_uros_ctx.executor, RCL_MS_TO_NS(100));

        if (++ping_counter >= ping_interval) {
            ping_counter = 0;

            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
                ESP_LOGW(TAG_UROS, "Lost connection to Agent");

                if (!try_reconnect()) {
                    ESP_LOGE(TAG_UROS, "Reconnection failed - restarting");
                    vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
                    esp_restart();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Cleanup (unreachable but good practice) */
    RCSOFTCHECK(rcl_publisher_fini(&g_uros_ctx.sensor_publisher, &g_uros_ctx.node));
    RCSOFTCHECK(rclc_executor_fini(&g_uros_ctx.executor));
    RCSOFTCHECK(rcl_node_fini(&g_uros_ctx.node));
    RCSOFTCHECK(rclc_support_fini(&g_uros_ctx.support));
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
    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIO,
        NULL,
        0  /* PRO_CPU */
    );
}
