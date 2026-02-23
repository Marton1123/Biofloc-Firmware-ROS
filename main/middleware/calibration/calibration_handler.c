/**
 * @file calibration_handler.c
 * @brief Implementación del manejador de calibración remota (Thread-Safe v4.1.3)
 * @version 4.1.3 - Fixed dangling pointer: action string copied before cJSON_Delete()
 * 
 * Arquitectura SRP (Single Responsibility Principle):
 * - Cada función tiene una responsabilidad única
 * - Validación exhaustiva en cada paso
 * - No causa PANIC: todos los paths retornan gracefully
 *
 * INVARIANTE: Este código NUNCA crashea.
 */

#include "calibration_handler.h"
#include "core/app_state.h"
#include "core/config.h"
#include "core/types.h"
#include "middleware/uros/uros_manager.h"
#include "hal/sensors/sensors.h"

#include <string.h>
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <std_msgs/msg/string.h>

/* ============================================================================
 * CONSTANTES PRIVADAS
 * ============================================================================ */

#define CAL_CMD_BUFFER_SIZE  1024  /* Buffer para JSON entrante */
#define CAL_RESPONSE_SIZE    512   /* Buffer para respuesta ACK */
#define CAL_ACTION_SIZE      32    /* FIX v4.1.3: Buffer para copiar action string */

/* ============================================================================
 * FUNCIONES PRIVADAS - PROTOTIPOS
 * ============================================================================ */

static int safe_receive_msg(const std_msgs__msg__String *msg,
                            char *buffer, size_t buf_size);

static cJSON *parse_calibration_json_safe(const char *json_str,
                                          char *resp, size_t resp_size,
                                          sensor_type_t *out_type,
                                          char *out_action, size_t action_size);

static void execute_calibration_action(cJSON *root,
                                       sensor_type_t sensor_type,
                                       const char *sensor_str,
                                       const char *action,
                                       char *resp, size_t resp_size);

static void send_calibration_ack(const char *response);

/* ============================================================================
 * IMPLEMENTACIÓN - FUNCIONES PRIVADAS
 * ============================================================================ */

/**
 * @brief Copia segura de mensaje micro-ROS a buffer con terminador nulo
 */
static int safe_receive_msg(const std_msgs__msg__String *msg,
                            char *buffer, size_t buf_size)
{
    if (!msg || !msg->data.data || msg->data.size == 0) {
        ESP_LOGE(TAG_CALIBRATION, "SAFETY: NULL or empty calibration message");
        return -1;
    }

    size_t copy_len = msg->data.size;
    if (copy_len >= buf_size) {
        copy_len = buf_size - 1;
        ESP_LOGW(TAG_CALIBRATION, "Message truncated: %d → %d bytes",
                 (int)msg->data.size, (int)copy_len);
    }

    memcpy(buffer, msg->data.data, copy_len);
    buffer[copy_len] = '\0';
    return (int)copy_len;
}

/**
 * @brief Parsea y valida estructura JSON de calibración.
 *
 * FIX v4.1.3: El parámetro out_action ahora es un buffer propio (char*, size_t)
 * en lugar de un puntero al string interno de cJSON (const char**).
 * 
 * Razón: si el caller llama cJSON_Delete(root) antes de usar out_action,
 * el puntero queda colgado (use-after-free). Al copiar el string aquí,
 * out_action es válido incluso después de destruir el árbol cJSON.
 */
static cJSON *parse_calibration_json_safe(const char *json_str,
                                          char *resp, size_t resp_size,
                                          sensor_type_t *out_type,
                                          char *out_action, size_t action_size)
{
    app_state_t state_copy;
    if (app_state_get(&state_copy) != ESP_OK) return NULL;

    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        const char *err_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG_CALIBRATION, "JSON parse failed near: %.40s", err_ptr ? err_ptr : "(null)");
        
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Invalid JSON\"}",
                 state_copy.device_info.device_id);
        return NULL;
    }

    /* Validar campo "sensor" */
    cJSON *sensor_json = cJSON_GetObjectItem(root, "sensor");
    if (!cJSON_IsString(sensor_json) || !sensor_json->valuestring) {
        ESP_LOGE(TAG_CALIBRATION, "Missing or invalid 'sensor' field");
        
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Missing 'sensor' field\"}",
                 state_copy.device_info.device_id);
        cJSON_Delete(root);
        return NULL;
    }

    const char *sensor_str = sensor_json->valuestring;
    *out_type = SENSOR_TYPE_MAX;

    if (strcmp(sensor_str, "ph") == 0)                   *out_type = SENSOR_TYPE_PH;
    else if (strcmp(sensor_str, "temperature") == 0)      *out_type = SENSOR_TYPE_TEMPERATURE;
    else if (strcmp(sensor_str, "dissolved_oxygen") == 0) *out_type = SENSOR_TYPE_DISSOLVED_OXYGEN;
    else if (strcmp(sensor_str, "conductivity") == 0)     *out_type = SENSOR_TYPE_CONDUCTIVITY;
    else if (strcmp(sensor_str, "turbidity") == 0)        *out_type = SENSOR_TYPE_TURBIDITY;

    if (*out_type == SENSOR_TYPE_MAX) {
        ESP_LOGE(TAG_CALIBRATION, "Unknown sensor type: %s", sensor_str);
        
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Unknown sensor: %s\"}",
                 state_copy.device_info.device_id, sensor_str);
        cJSON_Delete(root);
        return NULL;
    }

    /* Validar campo "action" */
    cJSON *action_json = cJSON_GetObjectItem(root, "action");
    if (!cJSON_IsString(action_json) || !action_json->valuestring) {
        ESP_LOGE(TAG_CALIBRATION, "Missing or invalid 'action' field");
        
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Missing 'action' field\"}",
                 state_copy.device_info.device_id);
        cJSON_Delete(root);
        return NULL;
    }

    /* FIX v4.1.3: Copiar el string de action a un buffer propio del caller.
     * Antes se devolvía un puntero directo a action_json->valuestring, que
     * pertenece al árbol cJSON. Si el caller llamaba cJSON_Delete(root) antes
     * de usar el puntero, resultaba en use-after-free. */
    strncpy(out_action, action_json->valuestring, action_size - 1);
    out_action[action_size - 1] = '\0';

    return root;  /* Caller DEBE llamar cJSON_Delete(root) al terminar */
}

/**
 * @brief Ejecuta una acción de calibración (reset, get, o calibrate)
 */
static void execute_calibration_action(cJSON *root,
                                       sensor_type_t sensor_type,
                                       const char *sensor_str,
                                       const char *action,
                                       char *resp, size_t resp_size)
{
    app_state_t state_copy;
    if (app_state_get(&state_copy) != ESP_OK) return;
    
    /* ---- ACCIÓN: reset ---- */
    if (strcmp(action, "reset") == 0) {
        esp_err_t err = sensors_reset_calibration(sensor_type);
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"%s\",\"sensor\":\"%s\","
                 "\"message\":\"%s\"}",
                 state_copy.device_info.device_id,
                 (err == ESP_OK) ? "success" : "error",
                 sensor_str,
                 (err == ESP_OK) ? "Calibration reset to factory defaults"
                                 : "Reset failed");
        return;
    }

    /* ---- ACCIÓN: get ---- */
    if (strcmp(action, "get") == 0) {
        sensor_calibration_t cal;
        esp_err_t err = sensors_get_calibration(sensor_type, &cal);
        if (err == ESP_OK) {
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"success\",\"sensor\":\"%s\","
                     "\"enabled\":%s,\"slope\":%.6f,\"offset\":%.6f,"
                     "\"r_squared\":%.4f,\"points\":%d}",
                     state_copy.device_info.device_id, sensor_str,
                     cal.enabled ? "true" : "false",
                     cal.slope, cal.offset, cal.r_squared, cal.num_points);
        } else {
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\",\"sensor\":\"%s\","
                     "\"message\":\"Failed to get calibration\"}",
                     state_copy.device_info.device_id, sensor_str);
        }
        return;
    }

    /* ---- ACCIÓN: calibrate ---- */
    if (strcmp(action, "calibrate") == 0) {
        ESP_LOGI(TAG_CALIBRATION, "  → Calibrate action");
        
        cJSON *points_json = cJSON_GetObjectItem(root, "points");
        if (!cJSON_IsArray(points_json)) {
            ESP_LOGE(TAG_CALIBRATION, "Missing or invalid 'points' array");
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\","
                     "\"message\":\"Missing 'points' array\"}",
                     state_copy.device_info.device_id);
            return;
        }

        int num_points = cJSON_GetArraySize(points_json);
        ESP_LOGI(TAG_CALIBRATION, "  Points in JSON: %d", num_points);
        
        if (num_points < 2 || num_points > MAX_CALIBRATION_POINTS) {
            ESP_LOGE(TAG_CALIBRATION, "Invalid point count: %d (need 2-%d)", num_points, MAX_CALIBRATION_POINTS);
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\","
                     "\"message\":\"Need 2-%d calibration points, got %d\"}",
                     state_copy.device_info.device_id, MAX_CALIBRATION_POINTS, num_points);
            return;
        }

        calibration_point_t points[MAX_CALIBRATION_POINTS];
        ESP_LOGI(TAG_CALIBRATION, "  Parsing calibration points...");

        for (int i = 0; i < num_points; i++) {
            cJSON *pt = cJSON_GetArrayItem(points_json, i);
            if (!pt || !cJSON_IsObject(pt)) {
                ESP_LOGE(TAG_CALIBRATION, "Point %d: not an object", i);
                snprintf(resp, resp_size,
                         "{\"device_id\":\"%s\",\"status\":\"error\","
                         "\"message\":\"Point %d is not an object\"}",
                         state_copy.device_info.device_id, i);
                return;
            }

            cJSON *v = cJSON_GetObjectItem(pt, "voltage");
            cJSON *val = cJSON_GetObjectItem(pt, "value");

            if (!v || !cJSON_IsNumber(v) || !val || !cJSON_IsNumber(val)) {
                ESP_LOGE(TAG_CALIBRATION, "Point %d: missing or non-numeric voltage/value", i);
                snprintf(resp, resp_size,
                         "{\"device_id\":\"%s\",\"status\":\"error\","
                         "\"message\":\"Point %d: invalid voltage/value\"}",
                         state_copy.device_info.device_id, i);
                return;
            }

            points[i].voltage = (float)v->valuedouble;
            points[i].value   = (float)val->valuedouble;
            ESP_LOGI(TAG_CALIBRATION, "  Point %d: %.3fV → %.2f %s",
                     i + 1, points[i].voltage, points[i].value, sensor_str);
        }

        ESP_LOGI(TAG_CALIBRATION, "  ✓ All points parsed successfully");
        ESP_LOGI(TAG_CALIBRATION, "  Calling sensors_calibrate_generic()...");
        
        calibration_response_t cal_resp;
        memset(&cal_resp, 0, sizeof(cal_resp));

        esp_err_t err = sensors_calibrate_generic(sensor_type, points,
                                                   (uint8_t)num_points, &cal_resp);

        ESP_LOGI(TAG_CALIBRATION, "  sensors_calibrate_generic() returned: err=%d, status=%d",
                 err, cal_resp.status);

        if (err == ESP_OK && cal_resp.status == CAL_STATUS_SUCCESS) {
            ESP_LOGI(TAG_CALIBRATION, "✓ Calibration SUCCESS: R²=%.4f slope=%.6f offset=%.6f",
                     cal_resp.r_squared, cal_resp.slope, cal_resp.offset);
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"success\",\"sensor\":\"%s\","
                     "\"slope\":%.6f,\"offset\":%.6f,\"r_squared\":%.4f,"
                     "\"message\":\"%s\"}",
                     state_copy.device_info.device_id, sensor_str,
                     cal_resp.slope, cal_resp.offset, cal_resp.r_squared,
                     cal_resp.message);
        } else {
            ESP_LOGE(TAG_CALIBRATION, "✗ Calibration FAILED: %s (err=%d)", cal_resp.message, err);
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\",\"sensor\":\"%s\","
                     "\"message\":\"%s\"}",
                     state_copy.device_info.device_id, sensor_str, cal_resp.message);
        }
        return;
    }

    /* ---- Acción desconocida ---- */
    ESP_LOGE(TAG_CALIBRATION, "Unknown action: %s", action);
    snprintf(resp, resp_size,
             "{\"device_id\":\"%s\",\"status\":\"error\","
             "\"message\":\"Unknown action: %s\"}",
             state_copy.device_info.device_id, action);
}

/**
 * @brief Publica respuesta ACK de calibración a /biofloc/calibration_status
 */
static void send_calibration_ack(const char *response)
{
    uros_manager_publish_calibration_status(response, strlen(response));
    ESP_LOGI(TAG_CALIBRATION, "✓ ACK sent (%d bytes)", (int)strlen(response));
}

/* ============================================================================
 * API PÚBLICA - IMPLEMENTACIÓN
 * ============================================================================ */

void calibration_callback(const void *msgin)
{
    esp_task_wdt_reset();
    
    app_state_enter_calibration();
    
    static char safe_buffer[CAL_CMD_BUFFER_SIZE];
    static char response[CAL_RESPONSE_SIZE];
    /* FIX v4.1.3: Buffer propio para action, desacoplado del árbol cJSON */
    static char action_buf[CAL_ACTION_SIZE];
    
    memset(safe_buffer, 0, sizeof(safe_buffer));
    memset(response, 0, sizeof(response));
    memset(action_buf, 0, sizeof(action_buf));

    ESP_LOGI(TAG_CALIBRATION, "════════════════════════════════════════");
    ESP_LOGI(TAG_CALIBRATION, "  CALIBRATION CALLBACK INVOKED (v4.1.3)");
    ESP_LOGI(TAG_CALIBRATION, "════════════════════════════════════════");
    ESP_LOGI(TAG_CALIBRATION, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG_CALIBRATION, "Free stack (micro_ros_task): %u bytes", uxTaskGetStackHighWaterMark(NULL));

    /* Paso 1: Recepción segura */
    ESP_LOGI(TAG_CALIBRATION, "[1/4] Receiving message...");
    int len = safe_receive_msg((const std_msgs__msg__String *)msgin,
                               safe_buffer, sizeof(safe_buffer));
    if (len < 0) {
        ESP_LOGE(TAG_CALIBRATION, "✗ safe_receive_msg FAILED");
        
        app_state_t state_copy;
        if (app_state_get(&state_copy) == ESP_OK) {
            snprintf(response, sizeof(response),
                     "{\"device_id\":\"%s\",\"status\":\"error\","
                     "\"message\":\"NULL or empty message\"}",
                     state_copy.device_info.device_id);
            send_calibration_ack(response);
        }
        
        app_state_exit_calibration();
        ESP_LOGI(TAG_CALIBRATION, "════════════════════════════════════════");
        return;
    }

    ESP_LOGI(TAG_CALIBRATION, "✓ Message received (%d bytes)", len);
    ESP_LOGI(TAG_CALIBRATION, "Payload (first 200 chars): %.200s%s",
             safe_buffer, len > 200 ? "..." : "");

    /* Paso 2: Parsear + validar JSON */
    ESP_LOGI(TAG_CALIBRATION, "[2/4] Parsing JSON...");
    sensor_type_t sensor_type;

    /* FIX v4.1.3: Se pasa action_buf (buffer propio) en lugar de &action (puntero).
     * parse_calibration_json_safe() copia el string de action aquí, así action_buf
     * es válido aunque llamemos cJSON_Delete(root) antes de usarlo. */
    cJSON *root = parse_calibration_json_safe(safe_buffer, response,
                                              sizeof(response),
                                              &sensor_type,
                                              action_buf, sizeof(action_buf));
    if (!root) {
        ESP_LOGE(TAG_CALIBRATION, "✗ parse_calibration_json_safe FAILED");
        send_calibration_ack(response);
        app_state_exit_calibration();
        ESP_LOGI(TAG_CALIBRATION, "════════════════════════════════════════");
        return;
    }

    /* sensor_str: puntero temporal al árbol cJSON, solo se usa ANTES de cJSON_Delete */
    const char *sensor_str = cJSON_GetObjectItem(root, "sensor")->valuestring;

    ESP_LOGI(TAG_CALIBRATION, "✓ JSON parsed successfully");
    ESP_LOGI(TAG_CALIBRATION, "  Action: %s | Sensor: %s | Type: %d",
             action_buf, sensor_str, sensor_type);

    /* Paso 3: Ejecutar acción — action_buf es seguro, root sigue vivo */
    ESP_LOGI(TAG_CALIBRATION, "[3/4] Executing calibration action...");
    execute_calibration_action(root, sensor_type, sensor_str, action_buf,
                               response, sizeof(response));
    ESP_LOGI(TAG_CALIBRATION, "✓ Action executed");
    
    esp_task_wdt_reset();

    /* Paso 4: Cleanup + enviar ACK.
     * IMPORTANTE: cJSON_Delete(root) va ANTES de send_calibration_ack().
     * action_buf y sensor_str ya no se usan después de execute_calibration_action(),
     * por lo que este orden es seguro y libera memoria lo antes posible. */
    ESP_LOGI(TAG_CALIBRATION, "[4/4] Sending ACK...");
    cJSON_Delete(root);
    send_calibration_ack(response);

    app_state_exit_calibration();

    ESP_LOGI(TAG_CALIBRATION, "✓ Calibration command processed successfully");
    ESP_LOGI(TAG_CALIBRATION, "════════════════════════════════════════");
}
