/**
 * @file calibration_handler.c
 * @brief Implementación del manejador de calibración remota
 * @version 4.0.0
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

/* ============================================================================
 * FUNCIONES PRIVADAS - PROTOTIPOS
 * ============================================================================ */

static int safe_receive_msg(const std_msgs__msg__String *msg,
                            char *buffer, size_t buf_size);

static cJSON *parse_calibration_json_safe(const char *json_str,
                                          char *resp, size_t resp_size,
                                          sensor_type_t *out_type,
                                          const char **out_action);

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
 *
 * micro-ROS XRCE-DDS llena msg->data.data con EXACTAMENTE msg->data.size bytes
 * pero NO garantiza un terminador '\0'. cJSON_Parse() lee hasta '\0' → segfault.
 *
 * @param msg      Mensaje entrante micro-ROS
 * @param buffer   Buffer destino (debe ser >= buf_size bytes)
 * @param buf_size Tamaño del buffer destino
 * @return Bytes copiados (>0), o -1 en error
 */
static int safe_receive_msg(const std_msgs__msg__String *msg,
                            char *buffer, size_t buf_size)
{
    if (!msg || !msg->data.data || msg->data.size == 0) {
        ESP_LOGE(TAG_UROS, "SAFETY: NULL or empty calibration message");
        return -1;
    }

    size_t copy_len = msg->data.size;
    if (copy_len >= buf_size) {
        copy_len = buf_size - 1;
        ESP_LOGW(TAG_UROS, "Message truncated: %d → %d bytes",
                 (int)msg->data.size, (int)copy_len);
    }

    memcpy(buffer, msg->data.data, copy_len);
    buffer[copy_len] = '\0';
    return (int)copy_len;
}

/**
 * @brief Parsea y valida estructura JSON de calibración
 *
 * Valida: root es objeto, "sensor" es string conocido,
 * "action" es string conocido. En cualquier fallo, llena buffer
 * de respuesta y retorna NULL.
 *
 * @param json_str   String JSON terminado en nulo
 * @param resp       Buffer de respuesta (se llena en error)
 * @param resp_size  Tamaño del buffer de respuesta
 * @param out_type   Output: tipo de sensor parseado (enum)
 * @param out_action Output: puntero a string de acción (dentro de árbol cJSON)
 * @return cJSON root en éxito (caller DEBE hacer cJSON_Delete), NULL en error
 */
static cJSON *parse_calibration_json_safe(const char *json_str,
                                          char *resp, size_t resp_size,
                                          sensor_type_t *out_type,
                                          const char **out_action)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        const char *err_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG_UROS, "JSON parse failed near: %.40s", err_ptr ? err_ptr : "(null)");
        
        const app_state_t *state = app_state_get();
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Invalid JSON\"}",
                 state->device_info.device_id);
        return NULL;
    }

    /* Validar campo "sensor" */
    cJSON *sensor_json = cJSON_GetObjectItem(root, "sensor");
    if (!cJSON_IsString(sensor_json) || !sensor_json->valuestring) {
        ESP_LOGE(TAG_UROS, "Missing or invalid 'sensor' field");
        
        const app_state_t *state = app_state_get();
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Missing 'sensor' field\"}",
                 state->device_info.device_id);
        cJSON_Delete(root);
        return NULL;
    }

    const char *sensor_str = sensor_json->valuestring;
    *out_type = SENSOR_TYPE_MAX;  /* sentinel = desconocido */

    if (strcmp(sensor_str, "ph") == 0)                  *out_type = SENSOR_TYPE_PH;
    else if (strcmp(sensor_str, "temperature") == 0)     *out_type = SENSOR_TYPE_TEMPERATURE;
    else if (strcmp(sensor_str, "dissolved_oxygen") == 0) *out_type = SENSOR_TYPE_DISSOLVED_OXYGEN;
    else if (strcmp(sensor_str, "conductivity") == 0)    *out_type = SENSOR_TYPE_CONDUCTIVITY;
    else if (strcmp(sensor_str, "turbidity") == 0)       *out_type = SENSOR_TYPE_TURBIDITY;

    if (*out_type == SENSOR_TYPE_MAX) {
        ESP_LOGE(TAG_UROS, "Unknown sensor type: %s", sensor_str);
        
        const app_state_t *state = app_state_get();
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Unknown sensor: %s\"}",
                 state->device_info.device_id, sensor_str);
        cJSON_Delete(root);
        return NULL;
    }

    /* Validar campo "action" */
    cJSON *action_json = cJSON_GetObjectItem(root, "action");
    if (!cJSON_IsString(action_json) || !action_json->valuestring) {
        ESP_LOGE(TAG_UROS, "Missing or invalid 'action' field");
        
        const app_state_t *state = app_state_get();
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"error\",\"message\":\"Missing 'action' field\"}",
                 state->device_info.device_id);
        cJSON_Delete(root);
        return NULL;
    }

    *out_action = action_json->valuestring;
    return root;  /* Caller DEBE llamar cJSON_Delete(root) al terminar */
}

/**
 * @brief Ejecuta una acción de calibración (reset, get, o calibrate)
 *
 * Encapsula toda la lógica que toca NVS. Al retornar,
 * `resp` siempre contiene una respuesta JSON válida.
 *
 * @param root       Root cJSON parseado (para extraer "points")
 * @param sensor_type Tipo de sensor validado
 * @param sensor_str  String nombre del sensor (para mensajes de respuesta)
 * @param action      String de acción ("reset", "get", o "calibrate")
 * @param resp        Buffer de respuesta
 * @param resp_size   Tamaño del buffer de respuesta
 */
static void execute_calibration_action(cJSON *root,
                                       sensor_type_t sensor_type,
                                       const char *sensor_str,
                                       const char *action,
                                       char *resp, size_t resp_size)
{
    const app_state_t *state = app_state_get();
    
    /* ---- ACCIÓN: reset ---- */
    if (strcmp(action, "reset") == 0) {
        esp_err_t err = sensors_reset_calibration(sensor_type);
        snprintf(resp, resp_size,
                 "{\"device_id\":\"%s\",\"status\":\"%s\",\"sensor\":\"%s\","
                 "\"message\":\"%s\"}",
                 state->device_info.device_id,
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
                     state->device_info.device_id, sensor_str,
                     cal.enabled ? "true" : "false",
                     cal.slope, cal.offset, cal.r_squared, cal.num_points);
        } else {
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\",\"sensor\":\"%s\","
                     "\"message\":\"Failed to get calibration\"}",
                     state->device_info.device_id, sensor_str);
        }
        return;
    }

    /* ---- ACCIÓN: calibrate ---- */
    if (strcmp(action, "calibrate") == 0) {
        ESP_LOGI(TAG_UROS, "  → Calibrate action");
        
        /* Validar array "points" */
        cJSON *points_json = cJSON_GetObjectItem(root, "points");
        if (!cJSON_IsArray(points_json)) {
            ESP_LOGE(TAG_UROS, "Missing or invalid 'points' array");
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\","
                     "\"message\":\"Missing 'points' array\"}",
                     state->device_info.device_id);
            return;
        }

        int num_points = cJSON_GetArraySize(points_json);
        ESP_LOGI(TAG_UROS, "  Points in JSON: %d", num_points);
        
        if (num_points < 2 || num_points > MAX_CALIBRATION_POINTS) {
            ESP_LOGE(TAG_UROS, "Invalid point count: %d (need 2-%d)", num_points, MAX_CALIBRATION_POINTS);
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\","
                     "\"message\":\"Need 2-%d calibration points, got %d\"}",
                     state->device_info.device_id, MAX_CALIBRATION_POINTS, num_points);
            return;
        }

        /* Parsear cada punto de calibración con validación exhaustiva */
        calibration_point_t points[MAX_CALIBRATION_POINTS];
        ESP_LOGI(TAG_UROS, "  Parsing calibration points...");

        for (int i = 0; i < num_points; i++) {
            cJSON *pt = cJSON_GetArrayItem(points_json, i);
            if (!pt || !cJSON_IsObject(pt)) {
                ESP_LOGE(TAG_UROS, "Point %d: not an object", i);
                snprintf(resp, resp_size,
                         "{\"device_id\":\"%s\",\"status\":\"error\","
                         "\"message\":\"Point %d is not an object\"}",
                         state->device_info.device_id, i);
                return;
            }

            cJSON *v = cJSON_GetObjectItem(pt, "voltage");
            cJSON *val = cJSON_GetObjectItem(pt, "value");

            if (!v || !cJSON_IsNumber(v) || !val || !cJSON_IsNumber(val)) {
                ESP_LOGE(TAG_UROS, "Point %d: missing or non-numeric voltage/value", i);
                snprintf(resp, resp_size,
                         "{\"device_id\":\"%s\",\"status\":\"error\","
                         "\"message\":\"Point %d: invalid voltage/value\"}",
                         state->device_info.device_id, i);
                return;
            }

            points[i].voltage = (float)v->valuedouble;
            points[i].value   = (float)val->valuedouble;
            ESP_LOGI(TAG_UROS, "  Point %d: %.3fV → %.2f %s",
                     i + 1, points[i].voltage, points[i].value, sensor_str);
        }

        ESP_LOGI(TAG_UROS, "  ✓ All points parsed successfully");
        ESP_LOGI(TAG_UROS, "  Calling sensors_calibrate_generic()...");
        
        /* Ejecutar calibración + guardar en NVS */
        calibration_response_t cal_resp;
        memset(&cal_resp, 0, sizeof(cal_resp));

        ESP_LOGI(TAG_UROS, "  Free heap before calibration: %lu", (unsigned long)esp_get_free_heap_size());
        
        esp_err_t err = sensors_calibrate_generic(sensor_type, points,
                                                   (uint8_t)num_points, &cal_resp);

        ESP_LOGI(TAG_UROS, "  sensors_calibrate_generic() returned: err=%d, status=%d",
                 err, cal_resp.status);
        ESP_LOGI(TAG_UROS, "  Free heap after calibration: %lu", (unsigned long)esp_get_free_heap_size());

        if (err == ESP_OK && cal_resp.status == CAL_STATUS_SUCCESS) {
            ESP_LOGI(TAG_UROS, "✓ Calibration SUCCESS: R²=%.4f slope=%.6f offset=%.6f",
                     cal_resp.r_squared, cal_resp.slope, cal_resp.offset);
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"success\",\"sensor\":\"%s\","
                     "\"slope\":%.6f,\"offset\":%.6f,\"r_squared\":%.4f,"
                     "\"message\":\"%s\"}",
                     state->device_info.device_id, sensor_str,
                     cal_resp.slope, cal_resp.offset, cal_resp.r_squared,
                     cal_resp.message);
        } else {
            ESP_LOGE(TAG_UROS, "✗ Calibration FAILED: %s (err=%d)", cal_resp.message, err);
            snprintf(resp, resp_size,
                     "{\"device_id\":\"%s\",\"status\":\"error\",\"sensor\":\"%s\","
                     "\"message\":\"%s\"}",
                     state->device_info.device_id, sensor_str, cal_resp.message);
        }
        return;
    }

    /* ---- Acción desconocida ---- */
    ESP_LOGE(TAG_UROS, "Unknown action: %s", action);
    snprintf(resp, resp_size,
             "{\"device_id\":\"%s\",\"status\":\"error\","
             "\"message\":\"Unknown action: %s\"}",
             state->device_info.device_id, action);
}

/**
 * @brief Publica respuesta ACK de calibración a /biofloc/calibration_status
 *
 * Siempre publica, incluso en error — el lado Python necesita saber qué pasó.
 *
 * v4.0.0: Simplificado para usar API de uros_manager.
 *
 * @param response  String JSON de respuesta terminado en nulo
 */
static void send_calibration_ack(const char *response)
{
    uros_manager_publish_calibration_status(response, strlen(response));
    ESP_LOGI(TAG_UROS, "✓ ACK sent (%d bytes)", (int)strlen(response));
}

/* ============================================================================
 * API PÚBLICA - IMPLEMENTACIÓN
 * ============================================================================ */

/**
 * @brief Callback principal de calibración — orquesta funciones SRP
 *
 * Llamado por el executor rclc cuando llega mensaje en calibration_cmd.
 * Esta función NUNCA crashea: cada path de fallo loggea + envía ACK + retorna.
 * 
 * v3.5.0: LOGGING EXHAUSTIVO para diagnóstico de crasheos
 * v3.6.4: Buffers estáticos fuera del stack para reducir presión
 * v3.6.5/v4.0.0: PAUSA de publicación sensor_data durante calibración
 */
void calibration_callback(const void *msgin)
{
    /* CRÍTICO: Alimentar watchdog al inicio para prevenir timeout durante calibración */
    esp_task_wdt_reset();
    
    /* v3.6.5/v4.0.0: PAUSAR publicación sensor_data durante calibración para prevenir crash */
    app_state_enter_calibration();
    
    /* Buffers estáticos — fuera del stack para reducir presión en stack de 20KB */
    static char safe_buffer[CAL_CMD_BUFFER_SIZE];
    static char response[CAL_RESPONSE_SIZE];
    
    /* v3.6.4: Limpiar buffers para prevenir corrupción de datos de calibraciones previas */
    memset(safe_buffer, 0, sizeof(safe_buffer));
    memset(response, 0, sizeof(response));

    ESP_LOGI(TAG_UROS, "════════════════════════════════════════");
    ESP_LOGI(TAG_UROS, "  CALIBRATION CALLBACK INVOKED (v4.0.0)");
    ESP_LOGI(TAG_UROS, "════════════════════════════════════════");
    ESP_LOGI(TAG_UROS, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG_UROS, "Free stack (micro_ros_task): %u bytes", uxTaskGetStackHighWaterMark(NULL));

    /* Paso 1: Recepción segura con terminador nulo */
    ESP_LOGI(TAG_UROS, "[1/4] Receiving message...");
    int len = safe_receive_msg((const std_msgs__msg__String *)msgin,
                               safe_buffer, sizeof(safe_buffer));
    if (len < 0) {
        ESP_LOGE(TAG_UROS, "✗ safe_receive_msg FAILED");
        const app_state_t *state = app_state_get();
        snprintf(response, sizeof(response),
                 "{\"device_id\":\"%s\",\"status\":\"error\","
                 "\"message\":\"NULL or empty message\"}",
                 state->device_info.device_id);
        send_calibration_ack(response);
        app_state_exit_calibration();  /* v3.6.5/v4.0.0: Resumir publicación en error */
        ESP_LOGI(TAG_UROS, "════════════════════════════════════════");
        return;
    }

    ESP_LOGI(TAG_UROS, "✓ Message received (%d bytes)", len);
    ESP_LOGI(TAG_UROS, "Payload (first 200 chars): %.200s%s",
             safe_buffer, len > 200 ? "..." : "");

    /* Paso 2: Parsear + validar JSON */
    ESP_LOGI(TAG_UROS, "[2/4] Parsing JSON...");
    sensor_type_t sensor_type;
    const char *action = NULL;

    cJSON *root = parse_calibration_json_safe(safe_buffer, response,
                                              sizeof(response),
                                              &sensor_type, &action);
    if (!root) {
        ESP_LOGE(TAG_UROS, "✗ parse_calibration_json_safe FAILED");
        /* response ya fue llenado por la función parse */
        send_calibration_ack(response);
        app_state_exit_calibration();  /* v3.6.5/v4.0.0: Resumir publicación en error */
        ESP_LOGI(TAG_UROS, "════════════════════════════════════════");
        return;
    }

    /* Extraer string de sensor para logging/respuesta (seguro: ya validado arriba) */
    const char *sensor_str = cJSON_GetObjectItem(root, "sensor")->valuestring;

    ESP_LOGI(TAG_UROS, "✓ JSON parsed successfully");
    ESP_LOGI(TAG_UROS, "  Action: %s | Sensor: %s | Type: %d", action, sensor_str, sensor_type);

    /* Paso 3: Ejecutar acción */
    ESP_LOGI(TAG_UROS, "[3/4] Executing calibration action...");
    execute_calibration_action(root, sensor_type, sensor_str, action,
                               response, sizeof(response));
    ESP_LOGI(TAG_UROS, "✓ Action executed");
    
    /* CRÍTICO: Alimentar watchdog después de ejecución de calibración */
    esp_task_wdt_reset();

    /* Paso 4: Cleanup + enviar ACK (SIEMPRE, incluso en error) */
    ESP_LOGI(TAG_UROS, "[4/4] Sending ACK...");
    cJSON_Delete(root);
    send_calibration_ack(response);

    /* v3.6.5/v4.0.0: RESUMIR publicación sensor_data después de calibración */
    app_state_exit_calibration();

    ESP_LOGI(TAG_UROS, "✓ Calibration command processed successfully");
    ESP_LOGI(TAG_UROS, "════════════════════════════════════════");
}
