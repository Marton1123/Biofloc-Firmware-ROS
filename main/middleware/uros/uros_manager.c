/**
 * @file uros_manager.c
 * @brief Implementación del gestor de micro-ROS
 * @version 4.0.1 - Fixed dead code in init guard, memory leak in ping_agent()
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
#include "drivers/wifi_manager.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

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
    
    // Messages - Circular buffer con malloc directo (NUNCA liberar)
    // CRÍTICO: lwip accede buffers asíncronamente, NO podemos reutilizar memoria
    // SOLUCIÓN: Circular buffer de PUNTEROS, cada uno con malloc independiente
    #define MAX_SENSOR_MSGS 20  // 20 mensajes × 4s = 80s de buffer
    std_msgs__msg__String sensor_data_msgs[MAX_SENSOR_MSGS];
    char* sensor_data_buffers[MAX_SENSOR_MSGS];
    uint8_t sensor_data_msg_index;
    
    std_msgs__msg__String calibration_status_msg;
    #define MAX_CALIB_STATUS_MSGS 5
    char* calibration_status_buffers[MAX_CALIB_STATUS_MSGS];
    uint8_t calibration_status_msg_index;
    
    std_msgs__msg__String config_status_msg;
    std_msgs__msg__String calibration_cmd_msg;
    std_msgs__msg__String config_cmd_msg;
    
    // State
    bool initialized;
    uros_state_t state;
    uros_calibration_callback_t calibration_callback;
    
    // Keep-alive: timestamp de última actividad (ms desde boot)
    uint32_t last_activity_ms;
    
} uros_manager_state_t;

static uros_manager_state_t s_uros = {0};

/* ============================================================================
 * FORWARD DECLARATIONS - HELPERS PRIVADOS
 * ============================================================================ */

static bool uros_manager_ping_agent_with_options(rmw_init_options_t *rmw_options);

/* ============================================================================
 * HELPERS PRIVADOS
 * ============================================================================ */

static void internal_calibration_callback(const void *msgin)
{
    if (s_uros.calibration_callback) {
        s_uros.calibration_callback(msgin);
    } else {
        ESP_LOGW(TAG_UROS, "Calibration command received but no callback registered");
    }
}

static void internal_config_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    ESP_LOGI(TAG_MAIN, "Config command received (%zu bytes)", msg->data.size);
    
    config_manager_command_callback(msgin);
}

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

esp_err_t uros_manager_init(uros_calibration_callback_t calibration_callback)
{
    /* FIX v4.0.1: La versión anterior tenía dos checks de s_uros.initialized
     * consecutivos. El primero retornaba ESP_OK si ya estaba inicializado, por
     * lo que el segundo (que limpiaba recursos) era dead code y NUNCA ejecutaba.
     * Esto impedía limpiar recursos de una init fallida en reconexiones,
     * causando el error "rcl_init_options_init() rc=1".
     *
     * CORRECTO: Retornar si ya está inicializado correctamente, o limpiar
     * si hay recursos de una inicialización fallida previa. */
    if (s_uros.initialized) {
        ESP_LOGW(TAG_UROS, "uROS manager already initialized");
        return ESP_OK;
    }

    /* Limpiar recursos de una init fallida previa (s_uros.initialized == false
     * pero pueden haber recursos parcialmente inicializados).
     * Llamar deinit es seguro: verifica internamente si hay algo que limpiar. */
    ESP_LOGD(TAG_UROS, "Cleaning up any previous partial resources before init");
    uros_manager_deinit();
    
    ESP_LOGI(TAG_UROS, "Initializing uROS manager...");
    
    // Registrar callback ANTES de crear executor (evita race condition)
    s_uros.calibration_callback = calibration_callback;
    if (calibration_callback) {
        ESP_LOGI(TAG_UROS, "✓ Calibration callback registered");
    }
    
    // CRITICAL: Initialize allocator FIRST (needed for all subsequent operations)
    s_uros.allocator = rcl_get_default_allocator();
    
    // Configurar estado
    s_uros.state = UROS_STATE_CONNECTING;
    app_state_set_uros(UROS_STATE_CONNECTING);

    // Init options
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, s_uros.allocator));
    
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", AGENT_PORT);
    
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, port_str, rmw_options));
    
    ESP_LOGI(TAG_UROS, "Pinging Agent %s:%s (timeout: %dms, retries: %d)",
             AGENT_IP, port_str, PING_TIMEOUT_MS, PING_RETRIES);
    
    if (!uros_manager_ping_agent_with_options(rmw_options)) {
        ESP_LOGW(TAG_UROS, "⚠️ Agent unreachable - will reconnect");
        s_uros.state = UROS_STATE_DISCONNECTED;
        app_state_set_uros(UROS_STATE_DISCONNECTED);
        rcl_init_options_fini(&init_options);  // Liberar antes de salir
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
    
    // Publishers
    ESP_LOGI(TAG_UROS, "Creating publishers...");
    
    RCCHECK(rclc_publisher_init_best_effort(
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
    
    // LÍMITE ARQUITECTÓNICO: El ESP32 con micro-ROS solo soporta 2 publishers máximo.
    // El 3º publisher (config_status_pub) causa error rc=1 en rclc_publisher_init_default().
    // RCCHECK(rclc_publisher_init_default(
    //     &s_uros.config_status_pub, ...));
    
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
    
    // CRÍTICO: Inicializar buffers para mensajes ANTES del executor
    ESP_LOGI(TAG_UROS, "Initializing message buffers...");
    
    s_uros.sensor_data_msg_index = 0;
    for (int i = 0; i < MAX_SENSOR_MSGS; i++) {
        s_uros.sensor_data_buffers[i] = (char*)malloc(2048);
        if (!s_uros.sensor_data_buffers[i]) {
            ESP_LOGE(TAG_UROS, "Failed to allocate sensor_data_buffer[%d]", i);
            return ESP_ERR_NO_MEM;
        }
        
        s_uros.sensor_data_msgs[i].data.data = s_uros.sensor_data_buffers[i];
        s_uros.sensor_data_msgs[i].data.size = 0;
        s_uros.sensor_data_msgs[i].data.capacity = 2048;
    }
    
    s_uros.calibration_status_msg_index = 0;
    for (int i = 0; i < MAX_CALIB_STATUS_MSGS; i++) {
        s_uros.calibration_status_buffers[i] = (char*)malloc(512);
        if (!s_uros.calibration_status_buffers[i]) {
            ESP_LOGE(TAG_UROS, "Failed to allocate calibration_status_buffer[%d]", i);
            return ESP_ERR_NO_MEM;
        }
    }
    
    static char calibration_buffer[1024];
    static char config_buffer[1024];
    
    s_uros.calibration_cmd_msg.data.data = calibration_buffer;
    s_uros.calibration_cmd_msg.data.size = 0;
    s_uros.calibration_cmd_msg.data.capacity = sizeof(calibration_buffer);
    
    s_uros.config_cmd_msg.data.data = config_buffer;
    s_uros.config_cmd_msg.data.size = 0;
    s_uros.config_cmd_msg.data.capacity = sizeof(config_buffer);
    
    ESP_LOGI(TAG_UROS, "✓ Message buffers initialized (1024 bytes each)");
    
    // Executor
    ESP_LOGI(TAG_UROS, "Creating executor (2 handles)...");
    RCCHECK(rclc_executor_init(&s_uros.executor, &s_uros.support.context,
                               2, &s_uros.allocator));
    
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
    ESP_LOGI(TAG_UROS, "  Publishers: sensor_data, calibration_status");
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
    
    if (json_len > 2048) {
        ESP_LOGE(TAG_UROS, "Sensor data too large: %zu > 2048", json_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    std_msgs__msg__String *msg = &s_uros.sensor_data_msgs[s_uros.sensor_data_msg_index];
    char *buffer = s_uros.sensor_data_buffers[s_uros.sensor_data_msg_index];
    
    s_uros.sensor_data_msg_index = (s_uros.sensor_data_msg_index + 1) % MAX_SENSOR_MSGS;
    
    if (json_len >= msg->data.capacity) {
        ESP_LOGE(TAG_UROS, "Sensor data exceeds buffer: %zu >= %zu", json_len, msg->data.capacity);
        return ESP_ERR_INVALID_SIZE;
    }
    
    memcpy(buffer, json_data, json_len);
    buffer[json_len] = '\0';
    msg->data.size = json_len;
    
    static uint32_t pub_count = 0;
    pub_count++;
    
    uint32_t free_heap = esp_get_free_heap_size();
    
    ESP_LOGI(TAG_UROS, "[PUB #%lu] ATTEMPTING publish sensor_data (%zu bytes, heap: %lu)", 
             pub_count, json_len, free_heap);
    
    rcl_ret_t ret = rcl_publish(&s_uros.sensor_data_pub, msg, NULL);
    
    if (ret != RCL_RET_OK) {
        ESP_LOGE(TAG_UROS, "[PUB #%lu] ❌ FAILED rcl_publish (rc=%d)", pub_count, (int)ret);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG_UROS, "[PUB #%lu] ✅ SUCCESS rcl_publish (heap after: %lu)", 
             pub_count, esp_get_free_heap_size());
    
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
    
    if (response_len > 1024) {
        ESP_LOGE(TAG_UROS, "Calibration status too large: %zu > 1024", response_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    char *buffer = s_uros.calibration_status_buffers[s_uros.calibration_status_msg_index];
    s_uros.calibration_status_msg_index = (s_uros.calibration_status_msg_index + 1) % MAX_CALIB_STATUS_MSGS;
    
    if (response_len >= 512) {
        ESP_LOGE(TAG_UROS, "Calibration response exceeds buffer: %zu >= 512", response_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    memcpy(buffer, json_response, response_len);
    buffer[response_len] = '\0';
    
    s_uros.calibration_status_msg.data.data = buffer;
    s_uros.calibration_status_msg.data.size = response_len;
    s_uros.calibration_status_msg.data.capacity = 512;
    
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

static bool uros_manager_ping_agent_with_options(rmw_init_options_t *rmw_options)
{
    for (int attempt = 1; attempt <= PING_RETRIES; attempt++) {
        ESP_LOGD(TAG_UROS, "Ping attempt %d/%d", attempt, PING_RETRIES);
        
        rmw_ret_t ret = rmw_uros_ping_agent_options(PING_TIMEOUT_MS, 1, rmw_options);
        if (ret == RMW_RET_OK) {
            return true;
        }
        
        if (attempt < PING_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
    
    return false;
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
    
    bool result = false;
    for (int attempts = 0; attempts < PING_RETRIES; attempts++) {
        rmw_ret_t ping_ret = rmw_uros_ping_agent_options(PING_TIMEOUT_MS, 1, rmw_options);
        if (ping_ret == RMW_RET_OK) {
            result = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    /* FIX v4.0.1: Liberar init_options para evitar memory leak.
     * La versión anterior retornaba directamente sin llamar rcl_init_options_fini(),
     * acumulando una fuga de memoria en cada llamada a ping durante reconexión. */
    rcl_init_options_fini(&init_options);
    
    return result;
#else
    return true;
#endif
}

void uros_manager_reconnect_forever(void)
{
    ESP_LOGW(TAG_UROS, "Reconnecting to Agent...");
    
    s_uros.state = UROS_STATE_CONNECTING;
    app_state_set_uros(UROS_STATE_CONNECTING);
    
    ESP_LOGI(TAG_UROS, "Step 1/2: Verifying WiFi physical connection...");
    wifi_manager_reconnect_forever();
    ESP_LOGI(TAG_UROS, "✓ WiFi physical layer restored");
    
    /* NOTA: El timeout real de este bucle depende de cuántos reintentos internos
     * hace uros_manager_ping_agent() (PING_RETRIES × 500ms por intento interno).
     * Con MAX_RECONNECT_ATTEMPTS=30 y PING_RETRIES=3, el tiempo máximo real es
     * 30 × (3 × 0.5s + 1s) = ~75 segundos, no 30 segundos como indica el comentario. */
    ESP_LOGI(TAG_UROS, "Step 2/2: Waiting for ROS2 Agent (max 30 outer attempts)...");
    uint32_t retry_count = 0;
    const uint32_t MAX_RECONNECT_ATTEMPTS = 30;
    
    while (retry_count < MAX_RECONNECT_ATTEMPTS) {
        esp_task_wdt_reset();
        
        if (uros_manager_ping_agent()) {
            ESP_LOGI(TAG_UROS, "✓ Agent ping successful after %lu retries", (unsigned long)retry_count);
            break;
        }
        
        retry_count++;
        if (retry_count % 10 == 0) {
            ESP_LOGI(TAG_UROS, "Still waiting for Agent... (attempt %lu/%lu)", 
                     (unsigned long)retry_count, (unsigned long)MAX_RECONNECT_ATTEMPTS);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (retry_count >= MAX_RECONNECT_ATTEMPTS) {
        ESP_LOGE(TAG_UROS, "❌ Failed to reconnect after max attempts - RESTARTING ESP32");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
    
    ESP_LOGI(TAG_UROS, "✓ Reconnection sequence complete - ready for re-init");
}

void uros_manager_spin_once(uint32_t timeout_ms)
{
    if (!s_uros.initialized) {
        return;
    }
    
    rclc_executor_spin_some(&s_uros.executor, RCL_MS_TO_NS(timeout_ms));
    s_uros.last_activity_ms = esp_timer_get_time() / 1000;
}

esp_err_t uros_manager_keep_alive(void)
{
    if (!s_uros.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t current_ms = esp_timer_get_time() / 1000;
    uint32_t elapsed_ms = current_ms - s_uros.last_activity_ms;
    
    if (elapsed_ms > 20000) {
        ESP_LOGD(TAG_UROS, "🔄 Keep-alive: %lums sin actividad, enviando ping silencioso", (unsigned long)elapsed_ms);
        
        if (uros_manager_ping_agent()) {
            s_uros.last_activity_ms = current_ms;
            return ESP_OK;
        } else {
            ESP_LOGW(TAG_UROS, "⚠️ Keep-alive ping falló - posible desconexión");
            return ESP_ERR_TIMEOUT;
        }
    }
    
    return ESP_OK;
}

void uros_manager_deinit(void)
{
    if (!s_uros.initialized) {
        return;
    }
    
    ESP_LOGI(TAG_UROS, "Deinitializing uROS manager...");
    
    // CRÍTICO: NO liberar buffers malloc aquí.
    // lwip puede estar accediendo asíncronamente durante transmisión UDP.
    // Los buffers se liberan automáticamente al reiniciar ESP32.
    
    rcl_publisher_fini(&s_uros.calibration_status_pub, &s_uros.node);
    rcl_publisher_fini(&s_uros.sensor_data_pub, &s_uros.node);
    rcl_subscription_fini(&s_uros.config_cmd_sub, &s_uros.node);
    rcl_subscription_fini(&s_uros.calibration_cmd_sub, &s_uros.node);
    rclc_executor_fini(&s_uros.executor);
    rcl_node_fini(&s_uros.node);
    rclc_support_fini(&s_uros.support);
    
    s_uros.initialized = false;
    s_uros.state = UROS_STATE_DISCONNECTED;
    s_uros.sensor_data_msg_index = 0;
    s_uros.calibration_status_msg_index = 0;
    
    ESP_LOGI(TAG_UROS, "✓ uROS manager deinitialized");
}
