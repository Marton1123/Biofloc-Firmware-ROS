/**
 * @file calibration_handler.h
 * @brief Manejador de calibración remota de sensores
 * @version 4.0.0
 * 
 * Responsabilidades:
 * - Procesar comandos de calibración desde ROS2
 * - Ejecutar acciones: reset, get, calibrate_low, calibrate_mid, calibrate_high
 * - Validar JSON y parámetros de forma segura
 * - Enviar ACK/NACK a ROS2
 * 
 * Arquitectura SRP (Single Responsibility Principle):
 * - safe_receive_msg()             → Buffer seguro
 * - parse_calibration_json_safe()  → Validación JSON
 * - execute_calibration_action()   → Dispatch de acción
 * - send_calibration_ack()         → Respuesta ROS2
 * - calibration_callback()         → Orquestador principal
 * 
 * @note Thread-safe: usa app_state mutex internamente
 * @note No causa PANIC: validación exhaustiva en cada paso
 */

#ifndef CALIBRATION_HANDLER_H
#define CALIBRATION_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Callback principal para comandos de calibración desde ROS2
 * 
 * @param msgin Puntero a std_msgs__msg__String con comando JSON
 * 
 * @note Llamado desde executor micro-ROS (micro_ros_task)
 * @note Pausa sensor_data durante ejecución
 * @note SIEMPRE envía ACK (incluso en error)
 * 
 * Formato JSON esperado:
 * {
 *   "device_id": "biofloc_esp32_c8e0",
 *   "sensor": "ph",                    // "ph" o "temperature"
 *   "action": "calibrate_mid",         // reset/get/calibrate_low/mid/high
 *   "value": 7.0                       // Solo para calibrate_*
 * }
 */
void calibration_callback(const void *msgin);

#ifdef __cplusplus
}
#endif

#endif // CALIBRATION_HANDLER_H
