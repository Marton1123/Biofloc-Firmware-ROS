/**
 * @file    main.c
 * @brief   Biofloc Firmware ROS - Sistema de telemetría con micro-ROS
 * @version 3.1.1
 *
 * @description
 *   Firmware para ESP32 con micro-ROS Jazzy sobre WiFi UDP.
 *   Lee sensores de pH y temperatura CWT-BL y publica datos JSON a ROS 2.
 *   Soporta calibración remota vía topic /biofloc/calibration_cmd.
 *
 * @changelog v3.1.1 (2026-02-12)
 *   - ✅ CRÍTICO: Eliminado boot loop en caso de desconexión
 *   - ✅ Reconexión infinita sin reinicio del ESP32
 *   - ✅ LED de estado indica desconexión (parpadeo lento)
 *   - ✅ Aumentados timeouts para redes lentas (5s → 10s)
 *   - ✅ Verificación de NVS antes de inicializar
 *   - ✅ Compatibilidad 100% con biofloc_manager.py
 */

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"  /* Hardware Task Watchdog Timer */
#include "esp_timer.h"      /* System uptime */
#include "esp_heap_caps.h"  /* Free heap tracking */

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

#define FIRMWARE_VERSION        "3.2.0"

/* Watchdog Timer Configuration (CRITICAL for zombie state prevention) */
#define WATCHDOG_TIMEOUT_SEC    20      /* Reset ESP32 if task doesn't feed watchdog for 20s */
#define WATCHDOG_FEED_INTERVAL  5000    /* Feed watchdog every 5 seconds */

#define AGENT_IP                CONFIG_BIOFLOC_AGENT_IP
#define AGENT_PORT              CONFIG_BIOFLOC_AGENT_PORT
#define ROS_NAMESPACE           CONFIG_BIOFLOC_ROS_NAMESPACE
#define PING_TIMEOUT_MS         10000  /* Aumentado de 5s a 10s para redes lentas */
#define PING_RETRIES            5      /* Aumentado de 3 a 5 reintentos */
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
#define CAL_RESPONSE_SIZE       512
#define PING_CHECK_INTERVAL_MS  30000
#define RECONNECT_DELAY_INITIAL 3000   /* Delay inicial: 3s */
#define RECONNECT_DELAY_MAX     60000  /* Max delay: 60s (aumentado de 30s) */
#define RECONNECT_FOREVER       true   /* CRÍTICO: Nunca reiniciar, reconectar infinitamente */

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
    rcl_publisher_t calibration_response_publisher;
    rcl_subscription_t calibration_subscriber;
    std_msgs__msg__String sensor_msg;
    std_msgs__msg__String calibration_cmd_msg;
    std_msgs__msg__String calibration_response_msg;
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
static TaskHandle_t g_sensor_task_handle = NULL;

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

/**
 * @brief Infinite reconnection loop without restart (ANTI-BOOTLOOP)
 * 
 * This function will retry forever until the agent comes back online.
 * Uses exponential backoff: 3s, 6s, 12s, 24s, 48s, 60s, 60s...
 * ESP32 will NEVER restart - it will wait indefinitely for the agent.
 */
static void reconnect_forever(void)
{
    uint32_t delay_ms = RECONNECT_DELAY_INITIAL;
    uint32_t attempt = 0;
    
    ESP_LOGW(TAG_UROS, "⚠️ Lost connection - entering infinite reconnection mode");
    ESP_LOGW(TAG_UROS, "ESP32 will NOT restart - waiting for Agent to come back...");
    
    while (1) {
        attempt++;
        ESP_LOGI(TAG_UROS, "Reconnection attempt #%lu (delay: %lums)", attempt, delay_ms);
        
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
            ESP_LOGI(TAG_UROS, "✅ Reconnected successfully after %lu attempts!", attempt);
            return;  /* Exit infinite loop and resume normal operation */
        }
        
        /* Exponential backoff with cap at 60s */
        delay_ms = (delay_ms * 2 > RECONNECT_DELAY_MAX) 
                   ? RECONNECT_DELAY_MAX 
                   : delay_ms * 2;
    }
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
    
    /* Subscribe to Task Watchdog Timer (CRITICAL for zombie state detection) */
    ESP_LOGI(TAG_SENSOR, "Subscribing to watchdog (timeout: %ds)", WATCHDOG_TIMEOUT_SEC);
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err == ESP_OK) {
        ESP_LOGI(TAG_SENSOR, "✓ Watchdog subscribed - will reset if blocked > %ds", WATCHDOG_TIMEOUT_SEC);
    } else {
        ESP_LOGW(TAG_SENSOR, "Failed to subscribe to watchdog (err=%d)", wdt_err);
    }

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
    uint32_t watchdog_counter = 0;

    while (!g_app_state.microros_ready) {
        ESP_LOGD(TAG_SENSOR, "Waiting for micro-ROS...");
        vTaskDelay(pdMS_TO_TICKS(500));
        
        /* Feed watchdog even while waiting */
        if (++watchdog_counter >= 10) {  /* Every 5 seconds (500ms * 10) */
            watchdog_counter = 0;
            esp_task_wdt_reset();
        }
    }

    ESP_LOGI(TAG_SENSOR, "Starting sensor readings");

    while (1) {
        /* Feed watchdog at the start of each iteration (CRITICAL) */
        esp_task_wdt_reset();
        
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
 * Calibration Command Handler
 * ============================================================================ */

/**
 * @brief Parse and execute calibration command from JSON
 * 
 * Expected JSON format:
 * {
 *   "sensor": "ph" | "temperature" | "dissolved_oxygen" | "conductivity" | "turbidity",
 *   "action": "calibrate" | "reset" | "get",
 *   "points": [
 *     {"voltage": 1.42, "value": 4.01},
 *     {"voltage": 2.45, "value": 6.86},
 *     {"voltage": 3.28, "value": 9.18}
 *   ]
 * }
 */
static void calibration_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    char response_buffer[CAL_RESPONSE_SIZE];
    
    ESP_LOGI(TAG_UROS, "Received calibration command: %s", msg->data.data);

    /* Parse JSON */
    cJSON *json = cJSON_Parse((const char *)msg->data.data);
    if (!json) {
        ESP_LOGE(TAG_UROS, "Failed to parse calibration JSON");
        snprintf(response_buffer, sizeof(response_buffer),
                 "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        goto send_response;
    }

    /* Extract sensor type */
    cJSON *sensor_json = cJSON_GetObjectItem(json, "sensor");
    if (!cJSON_IsString(sensor_json)) {
        ESP_LOGE(TAG_UROS, "Missing or invalid 'sensor' field");
        snprintf(response_buffer, sizeof(response_buffer),
                 "{\"status\":\"error\",\"message\":\"Missing sensor field\"}");
        cJSON_Delete(json);
        goto send_response;
    }

    const char *sensor_str = sensor_json->valuestring;
    sensor_type_t sensor_type = SENSOR_TYPE_MAX;
    
    if (strcmp(sensor_str, "ph") == 0) {
        sensor_type = SENSOR_TYPE_PH;
    } else if (strcmp(sensor_str, "temperature") == 0) {
        sensor_type = SENSOR_TYPE_TEMPERATURE;
    } else if (strcmp(sensor_str, "dissolved_oxygen") == 0) {
        sensor_type = SENSOR_TYPE_DISSOLVED_OXYGEN;
    } else if (strcmp(sensor_str, "conductivity") == 0) {
        sensor_type = SENSOR_TYPE_CONDUCTIVITY;
    } else if (strcmp(sensor_str, "turbidity") == 0) {
        sensor_type = SENSOR_TYPE_TURBIDITY;
    } else {
        ESP_LOGE(TAG_UROS, "Unknown sensor type: %s", sensor_str);
        snprintf(response_buffer, sizeof(response_buffer),
                 "{\"status\":\"error\",\"message\":\"Unknown sensor: %s\"}", sensor_str);
        cJSON_Delete(json);
        goto send_response;
    }

    /* Extract action */
    cJSON *action_json = cJSON_GetObjectItem(json, "action");
    if (!cJSON_IsString(action_json)) {
        ESP_LOGE(TAG_UROS, "Missing or invalid 'action' field");
        snprintf(response_buffer, sizeof(response_buffer),
                 "{\"status\":\"error\",\"message\":\"Missing action field\"}");
        cJSON_Delete(json);
        goto send_response;
    }

    const char *action = action_json->valuestring;

    /* Handle action: reset */
    if (strcmp(action, "reset") == 0) {
        esp_err_t err = sensors_reset_calibration(sensor_type);
        if (err == ESP_OK) {
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"success\",\"sensor\":\"%s\",\"message\":\"Calibration reset to factory defaults\"}",
                     sensor_str);
        } else {
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"error\",\"sensor\":\"%s\",\"message\":\"Reset failed\"}",
                     sensor_str);
        }
        cJSON_Delete(json);
        goto send_response;
    }

    /* Handle action: get */
    if (strcmp(action, "get") == 0) {
        sensor_calibration_t cal;
        esp_err_t err = sensors_get_calibration(sensor_type, &cal);
        if (err == ESP_OK) {
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"success\",\"sensor\":\"%s\",\"enabled\":%s,\"slope\":%.6f,\"offset\":%.6f,\"r_squared\":%.4f,\"points\":%d}",
                     sensor_str, cal.enabled ? "true" : "false",
                     cal.slope, cal.offset, cal.r_squared, cal.num_points);
        } else {
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"error\",\"sensor\":\"%s\",\"message\":\"Failed to get calibration\"}",
                     sensor_str);
        }
        cJSON_Delete(json);
        goto send_response;
    }

    /* Handle action: calibrate */
    if (strcmp(action, "calibrate") == 0) {
        cJSON *points_json = cJSON_GetObjectItem(json, "points");
        if (!cJSON_IsArray(points_json)) {
            ESP_LOGE(TAG_UROS, "Missing or invalid 'points' array");
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"error\",\"message\":\"Missing points array\"}");
            cJSON_Delete(json);
            goto send_response;
        }

        int num_points = cJSON_GetArraySize(points_json);
        if (num_points < 2 || num_points > MAX_CALIBRATION_POINTS) {
            ESP_LOGE(TAG_UROS, "Invalid number of points: %d (need 2-5)", num_points);
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"error\",\"message\":\"Need 2-5 calibration points, got %d\"}",
                     num_points);
            cJSON_Delete(json);
            goto send_response;
        }

        calibration_point_t points[MAX_CALIBRATION_POINTS];
        bool parse_error = false;
        
        for (int i = 0; i < num_points; i++) {
            cJSON *point = cJSON_GetArrayItem(points_json, i);
            if (!point) {
                ESP_LOGE(TAG_UROS, "NULL point at index %d", i);
                parse_error = true;
                break;
            }
            
            cJSON *voltage = cJSON_GetObjectItem(point, "voltage");
            cJSON *value = cJSON_GetObjectItem(point, "value");

            if (!voltage || !cJSON_IsNumber(voltage) || !value || !cJSON_IsNumber(value)) {
                ESP_LOGE(TAG_UROS, "Invalid point %d: missing or non-numeric voltage/value", i);
                parse_error = true;
                break;
            }

            points[i].voltage = (float)voltage->valuedouble;
            points[i].value = (float)value->valuedouble;
            
            ESP_LOGI(TAG_UROS, "  Point %d: %.3fV → %.2f pH", i+1, points[i].voltage, points[i].value);
        }
        
        if (parse_error) {
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"error\",\"message\":\"Failed to parse calibration points\"}");
            cJSON_Delete(json);
            goto send_response;
        }

        /* Perform calibration with memory zeroing */
        calibration_response_t cal_response;
        memset(&cal_response, 0, sizeof(cal_response));
        
        ESP_LOGI(TAG_UROS, "Starting calibration for %s with %d points", sensor_str, num_points);
        esp_err_t err = sensors_calibrate_generic(sensor_type, points, num_points, &cal_response);

        if (err == ESP_OK && cal_response.status == CAL_STATUS_SUCCESS) {
            ESP_LOGI(TAG_UROS, "✓ Calibration SUCCESS: R²=%.4f, slope=%.6f, offset=%.6f", 
                     cal_response.r_squared, cal_response.slope, cal_response.offset);
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"success\",\"sensor\":\"%s\",\"slope\":%.6f,\"offset\":%.6f,\"r_squared\":%.4f,\"message\":\"%s\"}",
                     sensor_str, cal_response.slope, cal_response.offset,
                     cal_response.r_squared, cal_response.message);
        } else {
            ESP_LOGE(TAG_UROS, "✗ Calibration FAILED: %s (err=%d)", cal_response.message, err);
            snprintf(response_buffer, sizeof(response_buffer),
                     "{\"status\":\"error\",\"sensor\":\"%s\",\"message\":\"%s\"}",
                     sensor_str, cal_response.message);
        }

        cJSON_Delete(json);
        goto send_response;
    }

    /* Unknown action */
    ESP_LOGE(TAG_UROS, "Unknown action: %s", action);
    snprintf(response_buffer, sizeof(response_buffer),
             "{\"status\":\"error\",\"message\":\"Unknown action: %s\"}", action);
    cJSON_Delete(json);

send_response:
    /* Publish response */
    if (g_uros_ctx.initialized) {
        g_uros_ctx.calibration_response_msg.data.data = response_buffer;
        g_uros_ctx.calibration_response_msg.data.size = strlen(response_buffer);
        g_uros_ctx.calibration_response_msg.data.capacity = CAL_RESPONSE_SIZE;

        rcl_ret_t ret = rcl_publish(&g_uros_ctx.calibration_response_publisher,
                                    &g_uros_ctx.calibration_response_msg, NULL);
        if (ret == RCL_RET_OK) {
            ESP_LOGI(TAG_UROS, "Calibration response sent");
        } else {
            ESP_LOGW(TAG_UROS, "Failed to send calibration response (rc=%d)", (int)ret);
        }
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
        ESP_LOGW(TAG_UROS, "⚠️ Agent unreachable on startup - waiting...");
        ESP_LOGW(TAG_UROS, "ESP32 will NOT restart - reconnecting infinitely");
        
        /* Wait forever for agent instead of restarting */
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (ping_agent(rmw_options, PING_TIMEOUT_MS, 1)) {
                ESP_LOGI(TAG_UROS, "✅ Agent is now online!");
                break;
            }
            ESP_LOGI(TAG_UROS, "Still waiting for Agent...");
        }
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

    ESP_LOGI(TAG_UROS, "Creating publisher: /%s/calibration_status", ROS_NAMESPACE);
    RCCHECK(rclc_publisher_init_default(
        &g_uros_ctx.calibration_response_publisher,
        &g_uros_ctx.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "calibration_status"
    ));

    ESP_LOGI(TAG_UROS, "Creating subscriber: /%s/calibration_cmd", ROS_NAMESPACE);
    RCCHECK(rclc_subscription_init_default(
        &g_uros_ctx.calibration_subscriber,
        &g_uros_ctx.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "calibration_cmd"
    ));

    /* Allocate memory for calibration command messages (HEAP allocation) */
    char *calibration_cmd_buffer = (char *)malloc(JSON_BUFFER_SIZE);
    if (!calibration_cmd_buffer) {
        ESP_LOGE(TAG_UROS, "Failed to allocate calibration buffer");
        vTaskDelete(NULL);
        return;
    }
    g_uros_ctx.calibration_cmd_msg.data.data = calibration_cmd_buffer;
    g_uros_ctx.calibration_cmd_msg.data.size = 0;
    g_uros_ctx.calibration_cmd_msg.data.capacity = JSON_BUFFER_SIZE;

    ESP_LOGI(TAG_UROS, "Initializing executor (2 handles)");
    RCCHECK(rclc_executor_init(&g_uros_ctx.executor,
                               &g_uros_ctx.support.context, 2,
                               &g_uros_ctx.allocator));

    ESP_LOGI(TAG_UROS, "Adding calibration subscriber to executor");
    RCCHECK(rclc_executor_add_subscription(
        &g_uros_ctx.executor,
        &g_uros_ctx.calibration_subscriber,
        &g_uros_ctx.calibration_cmd_msg,
        &calibration_callback,
        ON_NEW_DATA
    ));

    g_uros_ctx.initialized = true;
    g_app_state.microros_ready = true;

    ESP_LOGI(TAG_MAIN, "=========================================");
    ESP_LOGI(TAG_MAIN, "  micro-ROS Ready!");
    ESP_LOGI(TAG_MAIN, "  Node: /%s/biofloc_node", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "  Publisher: /%s/sensor_data", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "  Publisher: /%s/calibration_status", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "  Subscriber: /%s/calibration_cmd", ROS_NAMESPACE);
    ESP_LOGI(TAG_MAIN, "=========================================");

    uint32_t ping_counter = 0;
    const uint32_t ping_interval = PING_CHECK_INTERVAL_MS / 100;

    while (1) {
        rclc_executor_spin_some(&g_uros_ctx.executor, RCL_MS_TO_NS(100));

        if (++ping_counter >= ping_interval) {
            ping_counter = 0;

            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
                ESP_LOGW(TAG_UROS, "⚠️ Lost connection to Agent");
                
                /* CRITICAL: Never restart - reconnect forever instead */
                reconnect_forever();
                
                ESP_LOGI(TAG_UROS, "✅ Connection restored - resuming normal operation");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Cleanup (unreachable but good practice) */
    RCSOFTCHECK(rcl_publisher_fini(&g_uros_ctx.calibration_response_publisher, &g_uros_ctx.node));
    RCSOFTCHECK(rcl_subscription_fini(&g_uros_ctx.calibration_subscriber, &g_uros_ctx.node));
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
