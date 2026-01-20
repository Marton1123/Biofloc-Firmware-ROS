/**
 * ==============================================================================
 * @file    main.c
 * @brief   Biofloc Firmware ROS - Punto de entrada principal
 * @version 1.0.1
 * 
 * @description
 *   Firmware base para ESP32 con micro-ROS Jazzy sobre transporte WiFi UDP.
 *   Usa el componente micro_ros_espidf_component para manejo de red.
 * 
 * @author  Biofloc Engineering Team
 * @date    2026
 * ==============================================================================
 */

// ============================================================================
// INCLUDES
// ============================================================================

#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

// micro-ROS network interface (maneja WiFi internamente)
#include <uros_network_interfaces.h>

// micro-ROS core
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

// ============================================================================
// CONFIGURACIÓN (desde Kconfig)
// ============================================================================

#define AGENT_IP                CONFIG_BIOFLOC_AGENT_IP
#define AGENT_PORT              CONFIG_BIOFLOC_AGENT_PORT   // int
#define ROS_NAMESPACE           CONFIG_BIOFLOC_ROS_NAMESPACE
#define PING_TIMEOUT_MS         CONFIG_BIOFLOC_PING_TIMEOUT_MS
#define PING_RETRIES            CONFIG_BIOFLOC_PING_RETRIES

// Buffer para convertir puerto a string
static char agent_port_str[8];

// Stack para tarea micro-ROS (16KB mínimo recomendado)
#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 16000
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

// ============================================================================
// TAGS Y MACROS
// ============================================================================

static const char *TAG = "BIOFLOC_MAIN";
static const char *TAG_UROS = "BIOFLOC_UROS";

#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGE(TAG_UROS, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGW(TAG_UROS, "Soft error on line %d: %d", __LINE__, (int)temp_rc); \
    } \
}

// ============================================================================
// VARIABLES GLOBALES micro-ROS
// ============================================================================

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// ============================================================================
// TAREA PRINCIPAL micro-ROS
// ============================================================================

void micro_ros_task(void *arg)
{
    (void)arg;
    
    // Convertir puerto a string (rmw_uros_options_set_udp_address requiere string)
    snprintf(agent_port_str, sizeof(agent_port_str), "%d", AGENT_PORT);
    
    ESP_LOGI(TAG_UROS, "micro-ROS task started");
    ESP_LOGI(TAG_UROS, "Connecting to Agent at %s:%s", AGENT_IP, agent_port_str);

    // Obtener allocator
    allocator = rcl_get_default_allocator();

    // Configurar opciones de inicialización
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // Configurar dirección IP y puerto del Agent (puerto como string)
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, agent_port_str, rmw_options));
    ESP_LOGI(TAG_UROS, "UDP transport configured: %s:%s", AGENT_IP, agent_port_str);
#endif

    // === PING AL AGENT ANTES DE INICIALIZAR ===
    ESP_LOGI(TAG_UROS, "Pinging Agent (timeout: %d ms, retries: %d)...", PING_TIMEOUT_MS, PING_RETRIES);
    
    int ping_attempts = 0;
    while (ping_attempts < PING_RETRIES) {
        ping_attempts++;
        ESP_LOGI(TAG_UROS, "Ping attempt %d/%d", ping_attempts, PING_RETRIES);
        
        // Usar rmw_uros_ping_agent_options con las opciones configuradas
        if (rmw_uros_ping_agent_options(PING_TIMEOUT_MS, 1, rmw_options) == RMW_RET_OK) {
            ESP_LOGI(TAG_UROS, "✓ Agent is ONLINE!");
            break;
        }
        
        ESP_LOGW(TAG_UROS, "Ping failed, waiting 2 seconds...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    if (ping_attempts >= PING_RETRIES) {
        ESP_LOGE(TAG_UROS, "✗ Agent unreachable after %d attempts. Restarting...", PING_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

    // === INICIALIZAR micro-ROS ===
    ESP_LOGI(TAG_UROS, "Initializing micro-ROS support...");
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Crear nodo
    ESP_LOGI(TAG_UROS, "Creating node: /%s/biofloc_node", ROS_NAMESPACE);
    RCCHECK(rclc_node_init_default(&node, "biofloc_node", ROS_NAMESPACE, &support));

    // Crear executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  ✓ micro-ROS Ready!");
    ESP_LOGI(TAG, "  Node: /%s/biofloc_node", ROS_NAMESPACE);
    ESP_LOGI(TAG, "  Agent: %s:%s", AGENT_IP, agent_port_str);
    ESP_LOGI(TAG, "===========================================");

    // === BUCLE PRINCIPAL ===
    uint32_t ping_counter = 0;
    
    while (1) {
        // Spin del executor
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        
        // Ping periódico cada ~30 segundos (300 * 100ms)
        ping_counter++;
        if (ping_counter >= 300) {
            ping_counter = 0;
            
            if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
                ESP_LOGW(TAG_UROS, "Lost connection to Agent!");
                
                // Intentar reconectar
                bool reconnected = false;
                for (int i = 0; i < 5; i++) {
                    ESP_LOGI(TAG_UROS, "Reconnection attempt %d/5...", i + 1);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    
                    if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
                        ESP_LOGI(TAG_UROS, "✓ Reconnected to Agent!");
                        reconnected = true;
                        break;
                    }
                }
                
                if (!reconnected) {
                    ESP_LOGE(TAG_UROS, "Reconnection failed. Restarting...");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    esp_restart();
                }
            }
        }
        
        usleep(10000);  // 10ms
    }

    // Cleanup (nunca debería llegar aquí)
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
    
    vTaskDelete(NULL);
}

// ============================================================================
// PUNTO DE ENTRADA - app_main()
// ============================================================================

void app_main(void)
{
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  Biofloc Firmware ROS v1.0.1");
    ESP_LOGI(TAG, "  ESP-IDF: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "  micro-ROS: Jazzy");
    ESP_LOGI(TAG, "===========================================");

    // Inicializar interfaz de red (WiFi) usando el componente micro-ROS
    // Esta función maneja NVS, WiFi STA, y espera conexión
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_LOGI(TAG, "Initializing network interface...");
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    ESP_LOGI(TAG, "Network interface ready!");
#endif

    // Crear tarea de micro-ROS en APP_CPU para que PRO_CPU maneje WiFi
    ESP_LOGI(TAG, "Starting micro-ROS task...");
    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL,
        1  // APP_CPU
    );
}
