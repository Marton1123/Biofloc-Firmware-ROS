/**
 * @file uros_manager.h
 * @brief micro-ROS manager - Clean API for ROS2 communication
 * @version 4.0.0
 * 
 * Responsabilidades:
 * - Inicialización micro-ROS (support, node, allocator)
 * - Creación de publishers/subscribers
 * - Gestión de executor
 * - Ping al Agent
 * - Reconexión automática
 */

#ifndef UROS_MANAGER_H
#define UROS_MANAGER_H

#include "esp_err.h"
#include "core/types.h"
#include <rcl/rcl.h>
#include <std_msgs/msg/string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * TIPOS
 * ============================================================================ */

/**
 * @brief Callback para procesamiento de mensajes de calibración
 */
typedef void (*uros_calibration_callback_t)(const void *msgin);

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Inicializa subsistema micro-ROS y conecta al Agent
 * 
 * @param calibration_callback Callback para comandos de calibración (puede ser NULL)
 * @return ESP_OK si micro-ROS inicializado y conectado
 * @return ESP_FAIL si falla inicialización o conexión
 * 
 * @note Realiza ping al Agent antes de continuar
 * @note Configura node, publishers y subscribers
 * @note Callback se registra ANTES de crear executor (evita race condition)
 */
esp_err_t uros_manager_init(uros_calibration_callback_t calibration_callback);

/**
 * @brief Registra callback para comandos de calibración
 * 
 * @deprecated Usar parámetro en uros_manager_init() en su lugar
 * @param callback Función a llamar cuando se recibe comando
 * @return ESP_OK si callback registrado exitosamente
 */
esp_err_t uros_manager_set_calibration_callback(uros_calibration_callback_t callback);

/**
 * @brief Publica datos de sensores al topic /biofloc/sensor_data
 * 
 * @param json_data JSON string con datos de sensores
 * @param json_len Longitud del string JSON
 * @return ESP_OK si publicación exitosa
 */
esp_err_t uros_manager_publish_sensor_data(const char *json_data, size_t json_len);

/**
 * @brief Publica estado de calibración al topic /biofloc/calibration_status
 * 
 * @param json_response JSON string con resultado de calibración
 * @param response_len Longitud del string JSON
 * @return ESP_OK si publicación exitosa
 */
esp_err_t uros_manager_publish_calibration_status(const char *json_response, size_t response_len);

/**
 * @brief Obtiene estado actual de micro-ROS
 * 
 * @return uros_state_t Estado actual (DISCONNECTED, CONNECTING, CONNECTED, ERROR)
 */
uros_state_t uros_manager_get_state(void);

/**
 * @brief Verifica conectividad con Agent mediante ping
 * 
 * @return true si Agent responde
 * @return false si Agent no responde
 */
bool uros_manager_ping_agent(void);

/**
 * @brief Reconecta al Agent indefinidamente
 * 
 * @note Bloqueante - retorna solo cuando reconectado exitosamente
 */
void uros_manager_reconnect_forever(void);

/**
 * @brief Ejecuta un ciclo del executor (procesa callbacks)
 * 
 * @param timeout_ms Timeout en milisegundos para spin_some
 */
void uros_manager_spin_once(uint32_t timeout_ms);

/**
 * @brief Deinicializa subsistema micro-ROS y libera recursos
 */
void uros_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // UROS_MANAGER_H
