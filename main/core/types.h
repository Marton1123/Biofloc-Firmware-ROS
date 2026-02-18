/**
 * @file types.h
 * @brief Tipos de datos compartidos en todo el firmware
 * 
 * Define estructuras y enumeraciones compartidas entre módulos.
 * Principio: Single Source of Truth para tipos de datos.
 * 
 * @version 4.0.0
 */

#ifndef BIOFLOC_TYPES_H
#define BIOFLOC_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* ============================================================================
 * ENUMERACIONES
 * ============================================================================ */

/**
 * @brief Tipos de sensores soportados
 */
typedef enum {
    SENSOR_TYPE_PH = 0,
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_DISSOLVED_OXYGEN,
    SENSOR_TYPE_CONDUCTIVITY,
    SENSOR_TYPE_TURBIDITY,
    SENSOR_TYPE_MAX  /* Número total de sensores */
} sensor_type_t;

/**
 * @brief Estados del sistema WiFi
 */
typedef enum {
    WIFI_STATE_DISCONNECTED = 0,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_ERROR
} wifi_state_t;

/**
 * @brief Estados del micro-ROS
 */
typedef enum {
    UROS_STATE_DISCONNECTED = 0,
    UROS_STATE_CONNECTING,
    UROS_STATE_CONNECTED,
    UROS_STATE_ERROR
} uros_state_t;

/**
 * @brief Modo de agregación de datos
 */
typedef enum {
    DATA_MODE_INSTANT = 0,      /* Envío inmediato (4s) */
    DATA_MODE_AVERAGE,          /* Promedio de N muestras */
    DATA_MODE_MEDIAN,           /* Mediana de N muestras */
    DATA_MODE_MIN_MAX,          /* Min y max de N muestras */
    DATA_MODE_LAST              /* Última muestra del periodo */
} data_aggregation_mode_t;

/* ============================================================================
 * ESTRUCTURAS DE DATOS
 * ============================================================================ */

/**
 * @brief Lectura de un sensor individual
 */
typedef struct {
    float value;            /* Valor calibrado (pH, °C, etc.) */
    float voltage;          /* Voltaje del sensor (antes del divisor, 0-5V) */
    float voltage_adc;      /* Voltaje ADC (después del divisor) */
    int raw_adc;            /* Valor crudo del ADC (0-4095) */
    bool valid;             /* True si la lectura es válida */
    const char *unit;       /* Unidad de medida ("pH", "C") */
} sensor_reading_t;

/**
 * @brief Datos de todos los sensores
 */
typedef struct {
    sensor_reading_t ph;
    sensor_reading_t temperature;
    int64_t timestamp_ms;                       /* Unix timestamp en ms */
    char timestamp_iso[32];                     /* Timestamp ISO8601 */
} sensors_data_t;

/**
 * @brief Punto de calibración (voltaje, valor conocido)
 */
typedef struct {
    float voltage;      /* Voltaje medido (V) */
    float value;        /* Valor conocido (pH, °C, etc.) - renombrado para compatibilidad con sensors.h */
} calibration_point_t;

/**
 * @brief Estado de una calibración
 */
typedef enum {
    CAL_STATUS_SUCCESS = 0,
    CAL_STATUS_INVALID_SENSOR,
    CAL_STATUS_INVALID_POINTS,
    CAL_STATUS_INSUFFICIENT_POINTS,
    CAL_STATUS_NVS_ERROR,
    CAL_STATUS_NOT_INITIALIZED
} calibration_status_t;

/**
 * @brief Parámetros de calibración (resultado de regresión lineal)
 */
typedef struct {
    float slope;        /* Pendiente (m) */
    float offset;       /* Ordenada (b) */
    float r_squared;    /* Coeficiente de determinación */
    calibration_status_t status;
    uint32_t timestamp; /* Cuándo se calibró */
} calibration_params_t;

/**
 * @brief Información del dispositivo
 */
typedef struct {
    char device_id[32];      /* ID único del dispositivo */
    char mac_address[18];    /* Dirección MAC */
    char ip_address[16];     /* Dirección IP */
    char location[64];       /* Ubicación física */
    uint32_t uptime_sec;     /* Tiempo de funcionamiento */
    uint32_t free_heap;      /* Heap libre */
    const char *reset_reason; /* Razón del último reset */
} device_info_t;

/**
 * @brief Configuración dinámica de muestreo de sensores
 */
typedef struct {
    uint32_t sample_interval_ms;        /* Intervalo de lectura interna (default: 4000ms) */
    uint32_t publish_interval_ms;       /* Intervalo de publicación (default: 4000ms) */
    data_aggregation_mode_t mode;       /* Modo de agregación */
    uint16_t samples_per_publish;       /* Número de muestras a agregar (default: 1) */
    bool enabled;                       /* Publicación habilitada */
} sensor_config_t;

/**
 * @brief Estado global de la aplicación
 */
typedef struct {
    /* Información del dispositivo */
    device_info_t device_info;
    
    /* Estados de conectividad */
    wifi_state_t wifi_state;
    uros_state_t uros_state;
    
    /* Flags de operación */
    volatile bool calibrating;      /* True durante calibración */
    volatile bool sensor_task_ready;
    volatile bool uros_ready;
    
    /* Configuración dinámica de sensores */
    sensor_config_t sensor_config;
    
    /* Contadores */
    uint32_t sensor_publish_count;
    uint32_t calibration_count;
    uint32_t reconnect_count;
} app_state_t;

/* ============================================================================
 * CÓDIGOS DE ERROR PERSONALIZADOS
 * ============================================================================ */

#define ESP_ERR_SENSOR_BASE         0x8000
#define ESP_ERR_SENSOR_NOT_INIT     (ESP_ERR_SENSOR_BASE + 1)
#define ESP_ERR_SENSOR_READ_FAILED  (ESP_ERR_SENSOR_BASE + 2)
#define ESP_ERR_SENSOR_INVALID_DATA (ESP_ERR_SENSOR_BASE + 3)
#define ESP_ERR_CAL_INVALID_POINTS  (ESP_ERR_SENSOR_BASE + 4)
#define ESP_ERR_CAL_MATH_ERROR      (ESP_ERR_SENSOR_BASE + 5)

#endif /* BIOFLOC_TYPES_H */
