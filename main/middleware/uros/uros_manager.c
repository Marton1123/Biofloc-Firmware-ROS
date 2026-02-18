/**
 * @file uros_manager.c
 * @brief Implementación del gestor de micro-ROS
 * @version 4.0.0
 * 
 * Responsabilidades:
 * - Inicializar micro-ROS (support, node, allocator)
 * - Crear publishers (sensor_data, calibration_status, config_status)
 * - Crear subscribers (calibration_cmd, config_cmd)
 * - Gestionar executor y spin
 * - Ping Agent y reconexión
 */

#include "middleware/uros/uros_manager.h"
#include "middleware/config_manager.h"
#include "core/config.h"
#include "core/app_state.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

/* ============================================================================
 * ERROR HANDLING MACRO
 * ============================================================================ */

#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if (temp_rc != RCL_RET_OK) { \
        ESP_LOGE(TAG_UROS, "Failed: " #fn " (rc=%d)", (int)temp_rc); \
        return ESP_FAIL; \
    } \
}

/* ============================================================================
 * ESTADO PRIVADO
 * ============================================================================ */

typedef struct {
    // Core micro-ROS
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    
    // Publishers
    rcl_publisher_t sensor_data_pub;
    rcl_publisher_t calibration_status_pub;
    rcl_publisher_t config_status_pub;
    
    // Subscribers
    rcl_subscription_t calibration_cmd_sub;
    rcl_subscription_t config_cmd_sub;
    
    // Messages (static buffers)
    std_msgs__msg__String sensor_data_msg;
    std_msgs__msg__String calibration_status_msg;
    std_msgs__msg__String config_status_msg;
    std_msgs__msg__String calibration_cmd_msg;
    std_msgs__msg__String config_cmd_msg;
    
    // State
    bool initialized;
    uros_state_t state;
    uros_calibration_callback_t calibration_callback;
    
} uros_manager_state_t;

static uros_manager_state_t s_uros = {0};

/* ============================================================================
 * HELPERS PRIVADOS
 * ============================================================================ */

/**
 * @brief Wrapper interno para callback de calibración
 */
static void internal_calibration_callback(const void *msgin)
{
    if (s_uros.calibration_callback) {
        s_uros.calibration_callback(msgin);
    } else {
        ESP_LOGW(TAG_UROS, "Calibration command received but no callback registered");
    }
}

/**
 * @brief Wrapper interno para callback de configuración
 */
static void internal_config_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    ESP_LOGI(TAG_MAIN, "Config command received (%zu bytes)", msg->data.size);
    
    // Parsear y aplicar configuración
    config_manager_command_callback(msgin);
    
    // TODO: Enviar respuesta en config_status_pub
}

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

esp_err_t uros_manager_init(uros_calibration_callback_t calibration_callback)
{
    if (s_uros.initialized) {
        ESP_LOGW(TAG_UROS, "uROS manager already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG_UROS, "Initializing uROS manager...");
    
    // Registrar callback ANTES de crear executor (evita race condition)
    s_uros.calibration_callback = calibration_callback;
    if (calibration_callback) {
        ESP_LOGI(TAG_UROS, "✓ Calibration callback registered");
    }
    
    // Configurar estado
    s_uros.state = UROS_STATE_CONNECTING;
    app_state_set_uros(UROS_STATE_CONNECTING);
    
    // Allocator
    s_uros.allocator = rcl_get_default_allocator();
    
    // Init options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, s_uros.allocator));
    
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    // Configurar Agent
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", AGENT_PORT);
    
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, port_str, rmw_options));
    
    ESP_LOGI(TAG_UROS, "Pinging Agent %s:%s (timeout: %dms, retries: %d)",
             AGENT_IP, port_str, PING_TIMEOUT_MS, PING_RETRIES);
    
    // Ping Agent
    if (!uros_manager_ping_agent()) {
        ESP_LOGW(TAG_UROS, "⚠️ Agent unreachable - will reconnect");
        s_uros.state = UROS_STATE_DISCONNECTED;
        app_state_set_uros(UROS_STATE_DISCONNECTED);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG_UROS, "✓ Agent online");
#endif
    
    // Support
    ESP_LOGI(TAG_UROS, "Initializing support...");
    RCCHECK(rclc_support_init_with_options(&s_uros.support, 0, NULL,
                                           &init_options, &s_uros.allocator));
    
    // Node
    ESP_LOGI(TAG_UROS, "Creating node: /%s/biofloc_node", ROS_NAMESPACE);
    RCCHECK(rclc_node_init_default(&s_uros.node, "biofloc_node",
                                   ROS_NAMESPACE, &s_uros.support));
    
    // Initialize message structs BEFORE using them in publishers/subscribers
    ESP_LOGI(TAG_UROS, "Initializing message structures...");
    std_msgs__msg__String__init(&s_uros.sensor_data_msg);
    std_msgs__msg__String__init(&s_uros.calibration_status_msg);
    std_msgs__msg__String__init(&s_uros.config_status_msg);
    std_msgs__msg__String__init(&s_uros.calibration_cmd_msg);
    std_msgs__msg__String__init(&s_uros.config_cmd_msg);
    
    // Publishers
    ESP_LOGI(TAG_UROS, "Creating publishers...");
    
    RCCHECK(rclc_publisher_init_default(
        &s_uros.sensor_data_pub,
        &s_uros.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "sensor_data"
    ));
    
    RCCHECK(rclc_publisher_init_default(
        &s_uros.calibration_status_pub,
        &s_uros.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "calibration_status"
    ));
    
    RCCHECK(rclc_publisher_init_default(
        &s_uros.config_status_pub,
        &s_uros.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "config_status"
    ));
    
    // Subscribers
    ESP_LOGI(TAG_UROS, "Creating subscribers...");
    
    RCCHECK(rclc_subscription_init_default(
        &s_uros.calibration_cmd_sub,
        &s_uros.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "calibration_cmd"
    ));
    
    RCCHECK(rclc_subscription_init_default(
        &s_uros.config_cmd_sub,
        &s_uros.node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "config_cmd"
    ));
    
    // Executor
    ESP_LOGI(TAG_UROS, "Creating executor (2 handles)...");
    RCCHECK(rclc_executor_init(&s_uros.executor, &s_uros.support.context,
                               2, &s_uros.allocator));
    
    // Agregar subscribers al executor
    RCCHECK(rclc_executor_add_subscription(
        &s_uros.executor,
        &s_uros.calibration_cmd_sub,
        &s_uros.calibration_cmd_msg,
        &internal_calibration_callback,
        ON_NEW_DATA
    ));
    
    RCCHECK(rclc_executor_add_subscription(
        &s_uros.executor,
        &s_uros.config_cmd_sub,
        &s_uros.config_cmd_msg,
        &internal_config_callback,
        ON_NEW_DATA
    ));
    
    // Marcar como inicializado
    s_uros.initialized = true;
    s_uros.state = UROS_STATE_CONNECTED;
    app_state_set_uros(UROS_STATE_CONNECTED);
    
    ESP_LOGI(TAG_UROS, "✓ uROS manager initialized successfully");
    ESP_LOGI(TAG_UROS, "  Publishers: sensor_data, calibration_status, config_status");
    ESP_LOGI(TAG_UROS, "  Subscribers: calibration_cmd, config_cmd");
    
    return ESP_OK;
}

esp_err_t uros_manager_set_calibration_callback(uros_calibration_callback_t callback)
{
    if (!callback) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_uros.calibration_callback = callback;
    ESP_LOGI(TAG_UROS, "✓ Calibration callback registered");
    
    return ESP_OK;
}

esp_err_t uros_manager_publish_sensor_data(const char *json_data, size_t json_len)
{
    if (!s_uros.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!json_data || json_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Use rosidl_runtime_c__String__assign() to properly copy data
    if (!rosidl_runtime_c__String__assign(&s_uros.sensor_data_msg.data, json_data)) {
        ESP_LOGW(TAG_UROS, "Failed to assign sensor_data message");
        return ESP_FAIL;
    }
    
    rcl_ret_t ret = rcl_publish(&s_uros.sensor_data_pub,
                                &s_uros.sensor_data_msg, NULL);
    
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG_UROS, "Failed to publish sensor_data (rc=%d)", (int)ret);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t uros_manager_publish_calibration_status(const char *json_response, size_t response_len)
{
    if (!s_uros.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!json_response || response_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Use rosidl_runtime_c__String__assign() to properly copy data
    if (!rosidl_runtime_c__String__assign(&s_uros.calibration_status_msg.data, json_response)) {
        ESP_LOGW(TAG_UROS, "Failed to assign calibration_status message");
        return ESP_FAIL;
    }
    
    rcl_ret_t ret = rcl_publish(&s_uros.calibration_status_pub,
                                &s_uros.calibration_status_msg, NULL);
    
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG_UROS, "Failed to publish calibration_status (rc=%d)", (int)ret);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG_UROS, "✓ Calibration ACK sent (%zu bytes)", response_len);
    
    return ESP_OK;
}

uros_state_t uros_manager_get_state(void)
{
    return s_uros.state;
}

bool uros_manager_ping_agent(void)
{
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_ret_t ret = rcl_init_options_init(&init_options, s_uros.allocator);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", AGENT_PORT);
    rmw_uros_options_set_udp_address(AGENT_IP, port_str, rmw_options);
    
    int attempts = 0;
    while (attempts < PING_RETRIES) {
        rmw_ret_t ping_ret = rmw_uros_ping_agent(PING_TIMEOUT_MS, 1);
        if (ping_ret == RMW_RET_OK) {
            return true;
        }
        attempts++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    return false;
#else
    return true;  // Sin middleware, asumir conectado
#endif
}

void uros_manager_reconnect_forever(void)
{
    ESP_LOGW(TAG_UROS, "Reconnecting to Agent...");
    
    s_uros.state = UROS_STATE_CONNECTING;
    app_state_set_uros(UROS_STATE_CONNECTING);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        if (uros_manager_ping_agent()) {
            ESP_LOGI(TAG_UROS, "✓ Agent reconnected!");
            // CRÍTICO: Limpiar initialized flag para permitir reinit completo
            s_uros.initialized = false;
            break;
        }
        
        ESP_LOGI(TAG_UROS, "Still waiting for Agent...");
    }
}

void uros_manager_spin_once(uint32_t timeout_ms)
{
    if (!s_uros.initialized) {
        return;
    }
    
    rclc_executor_spin_some(&s_uros.executor, RCL_MS_TO_NS(timeout_ms));
}

void uros_manager_deinit(void)
{
    if (!s_uros.initialized) {
        return;
    }
    
    ESP_LOGI(TAG_UROS, "Deinitializing uROS manager...");
    
    // Cleanup (orden inverso a inicialización)
    rcl_publisher_fini(&s_uros.config_status_pub, &s_uros.node);
    rcl_publisher_fini(&s_uros.calibration_status_pub, &s_uros.node);
    rcl_publisher_fini(&s_uros.sensor_data_pub, &s_uros.node);
    rcl_subscription_fini(&s_uros.config_cmd_sub, &s_uros.node);
    rcl_subscription_fini(&s_uros.calibration_cmd_sub, &s_uros.node);
    rclc_executor_fini(&s_uros.executor);
    rcl_node_fini(&s_uros.node);
    rclc_support_fini(&s_uros.support);
    
    memset(&s_uros, 0, sizeof(uros_manager_state_t));
    
    ESP_LOGI(TAG_UROS, "✓ uROS manager deinitialized");
}
