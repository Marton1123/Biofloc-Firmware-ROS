/**
 * @file config.h
 * @brief Configuración global del firmware Biofloc
 * * Centraliza TODAS las constantes de configuración siguiendo
 * principio de Single Responsibility. No hay magic numbers en el código.
 * * @version 4.1.4
 * @date 2026-02-20
 */

#ifndef BIOFLOC_CONFIG_H
#define BIOFLOC_CONFIG_H

#include "sdkconfig.h"

/* ============================================================================
 * VERSIÓN DEL FIRMWARE
 * ============================================================================ */

#define FIRMWARE_VERSION        "4.1.5"
#define FIRMWARE_BUILD_DATE     __DATE__
#define FIRMWARE_BUILD_TIME     __TIME__

/* ============================================================================
 * CONFIGURACIÓN DE RED
 * ============================================================================ */

#define AGENT_IP                CONFIG_BIOFLOC_AGENT_IP
#define AGENT_PORT              CONFIG_BIOFLOC_AGENT_PORT
#define ROS_NAMESPACE           CONFIG_BIOFLOC_ROS_NAMESPACE
#define DEVICE_LOCATION         CONFIG_BIOFLOC_LOCATION

/* Timeouts de conexión (ms) */
#define PING_TIMEOUT_MS         2000    /* Timeout para ping al Agent (LAN = 2s suficiente) */
#define PING_RETRIES            3       /* Reintentos de ping (reducido para evitar spam) */
#define PING_CHECK_INTERVAL_MS  15000   /* Intervalo entre pings: 15s (< watchdog 20s) */

/* Reconexión WiFi */
#define RECONNECT_DELAY_INITIAL 3000    /* Delay inicial: 3s */
#define RECONNECT_DELAY_MAX     60000   /* Delay máximo: 60s */
#define RECONNECT_FOREVER       true    /* Reconectar infinitamente */

/* ============================================================================
 * CONFIGURACIÓN DE SENSORES
 * ============================================================================ */

/* Intervalos de muestreo */
#define SAMPLE_INTERVAL_MS      CONFIG_BIOFLOC_SENSOR_SAMPLE_INTERVAL_MS  /* Normalmente 4000ms */
#define CALIBRATION_TIMEOUT_MS  600000  /* 10 min timeout para calibración */

/* Pines ADC (ESP32 específico) */
#define ADC_PH_CHANNEL          ADC1_CHANNEL_6  /* GPIO34 */
#define ADC_TEMP_CHANNEL        ADC1_CHANNEL_7  /* GPIO35 */
#define ADC_WIDTH               ADC_WIDTH_BIT_12
#define ADC_ATTEN               ADC_ATTEN_DB_11

/* Límites de voltaje ADC */
#define ADC_MIN_VOLTAGE         0.0f
#define ADC_MAX_VOLTAGE         3.3f
#define ADC_INVALID_THRESHOLD   0.05f   /* < 50mV = desconectado */

/* Calibración por defecto (Factory) */
#define PH_DEFAULT_SLOPE        -5.70f
#define PH_DEFAULT_OFFSET       21.34f
#define TEMP_DEFAULT_SLOPE      100.0f
#define TEMP_DEFAULT_OFFSET     0.0f

/* ============================================================================
 * CONFIGURACIÓN DE WATCHDOG
 * ============================================================================ */

#define WATCHDOG_TIMEOUT_SEC    20      /* Reset si no hay feed en 20s */
#define WATCHDOG_FEED_INTERVAL  5000    /* Feed cada 5s (ms) */

/* ============================================================================
 * CONFIGURACIÓN DE TAREAS (FreeRTOS)
 * ============================================================================ */

/* Tamaño de stacks (bytes) */
#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 20480  /* 20KB para micro-ROS task */
#endif

#define SENSOR_TASK_STACK       12288     /* 12KB para sensor task */
#define WIFI_TASK_STACK         4096      /* 4KB para WiFi task */

/* Prioridades de tareas */
#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

#define SENSOR_TASK_PRIO        4         /* Alta prioridad */
#define WIFI_TASK_PRIO          2         /* Baja prioridad */

/* ============================================================================
 * CONFIGURACIÓN DE BUFFERS
 * ============================================================================ */

#define JSON_BUFFER_SIZE        512       /* Buffer para JSON de sensores */
#define CAL_CMD_BUFFER_SIZE     1024      /* Buffer para comandos de calibración */
#define CAL_RESPONSE_SIZE       512       /* Buffer para respuestas de calibración */
#define DEVICE_ID_MAX_LEN       32        /* Longitud máxima de device_id */
#define MAC_ADDRESS_LEN         18        /* Formato: "XX:XX:XX:XX:XX:XX\0" */
#define IP_ADDRESS_LEN          16        /* Formato: "XXX.XXX.XXX.XXX\0" */

/* ============================================================================
 * CONFIGURACIÓN DE NVS (Non-Volatile Storage)
 * ============================================================================ */

#define NVS_NAMESPACE_SENSORS   "sensors"
#define NVS_NAMESPACE_WIFI      "wifi"
#define NVS_NAMESPACE_SYSTEM    "system"

/* Keys de calibración en NVS */
#define NVS_KEY_PH_SLOPE        "ph_slope"
#define NVS_KEY_PH_OFFSET       "ph_offset"
#define NVS_KEY_PH_R2           "ph_r2"
#define NVS_KEY_TEMP_SLOPE      "temp_slope"
#define NVS_KEY_TEMP_OFFSET     "temp_offset"
#define NVS_KEY_TEMP_R2         "temp_r2"

/* ============================================================================
 * LOGGING TAGS
 * CORRECCIÓN: Se cambiaron las variables 'static const char*' por '#define'
 * para evitar los warnings de "defined but not used" en el compilador de GCC.
 * ============================================================================ */

#define TAG_MAIN        "BIOFLOC"
#define TAG_WIFI        "WIFI"
#define TAG_UROS        "UROS"
#define TAG_SENSOR      "SENSOR"
#define TAG_CALIBRATION "CALIBRATE"
#define TAG_NVS         "NVS"
#define TAG_WATCHDOG    "WATCHDOG"

/* ============================================================================
 * VALIDACIÓN DE CONFIGURACIÓN
 * ============================================================================ */

/* Verificar que el feed interval es menor que el watchdog timeout */
#if (WATCHDOG_FEED_INTERVAL >= (WATCHDOG_TIMEOUT_SEC * 1000))
    #error "WATCHDOG_FEED_INTERVAL debe ser menor que WATCHDOG_TIMEOUT_SEC"
#endif

/* Verificar que el ping interval es menor que el watchdog timeout */
#if (PING_CHECK_INTERVAL_MS >= (WATCHDOG_TIMEOUT_SEC * 1000))
    #error "PING_CHECK_INTERVAL_MS debe ser menor que WATCHDOG_TIMEOUT_SEC"
#endif

#endif /* BIOFLOC_CONFIG_H */
