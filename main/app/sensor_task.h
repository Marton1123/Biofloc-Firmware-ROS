/**
 * @file sensor_task.h
 * @brief Tarea de muestreo de sensores (Thread-Safe)
 * @version 4.1.2
 * * Responsabilidades:
 * - Muestrear sensores pH y temperatura cada sample_interval_ms
 * - Agregar muestras en buffer (data_aggregator)
 * - Publicar datos agregados cada publish_interval_ms
 * - Pausar de forma segura durante la calibración (protegiendo el ADC y bus I2C)
 * - Respetar configuración dinámica mediante copias atómicas del estado
 * * @note Thread-safe: usa copias locales de app_state_get() para evitar colisiones de memoria
 * @note Se suspende automáticamente durante calibración cediendo el procesador
 */

#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Crea y lanza la tarea de muestreo de sensores
 * * @param[out] task_handle Puntero donde guardar el handle de la tarea
 * @return pdPASS si tarea creada exitosamente
 * * @note Stack: 12KB (incrementado en v3.6.4 para prevenir overflow)
 * @note Prioridad: 4 (menor que micro_ros_task)
 * * Uso:
 * ```c
 * TaskHandle_t sensor_handle = NULL;
 * xTaskCreate(sensor_task, "sensor_task", 12288, NULL, 4, &sensor_handle);
 * ```
 */
void sensor_task(void *arg);

/**
 * @brief Obtiene el handle de la tarea de sensores (para monitoreo de stack)
 * * @return TaskHandle_t o NULL si no iniciado
 */
TaskHandle_t sensor_task_get_handle(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_TASK_H
