/**
 * @file    main.c
 * @brief   Prueba LED con micro-ROS - Control por teclado
 * @version 1.0.0
 *
 * @description
 *   Firmware simple para ESP32 que controla un LED via micro-ROS.
 *   Se suscribe al tópico /led_control y enciende/apaga el LED
 *   según los mensajes recibidos.
 *
 * @hardware
 *   - GPIO 2: LED integrado en ESP32
 *   
 * @commands
 *   Publica en /led_control:
 *   - "ON" o "1" -> Enciende LED
 *   - "OFF" o "0" -> Apaga LED
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

/* ============================================================================
 * Configuration
 * ============================================================================ */

#define LED_GPIO                2       // LED integrado en ESP32
#define AGENT_IP                CONFIG_MICRO_ROS_AGENT_IP
#define AGENT_PORT              CONFIG_MICRO_ROS_AGENT_PORT
#define PING_TIMEOUT_MS         1000
#define RECONNECT_DELAY_MS      2000
#define WIFI_SSID               CONFIG_ESP_WIFI_SSID
#define WIFI_PASS               CONFIG_ESP_WIFI_PASSWORD

/* WiFi event bits */
#define WIFI_CONNECTED_BIT      BIT0
#define WIFI_FAIL_BIT           BIT1

#ifndef CONFIG_MICRO_ROS_APP_STACK
#define CONFIG_MICRO_ROS_APP_STACK 16000
#endif

#ifndef CONFIG_MICRO_ROS_APP_TASK_PRIO
#define CONFIG_MICRO_ROS_APP_TASK_PRIO 5
#endif

/* ============================================================================
 * Logging
 * ============================================================================ */

static const char *TAG = "LED_TEST";

/* ============================================================================
 * Error Handling Macros
 * ============================================================================ */

#define RCCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "Error at %s:%d (rc=%d)", __FILE__, __LINE__, (int)rc); \
        return; \
    } \
} while(0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "Warning at %s:%d (rc=%d)", __FILE__, __LINE__, (int)rc); \
    } \
} while(0)

/* ============================================================================
 * Global State
 * ============================================================================ */

static rcl_subscription_t subscriber;
static std_msgs__msg__String incoming_msg;
static bool led_state = false;
static EventGroupHandle_t wifi_event_group;
static int wifi_retry_count = 0;

/* ============================================================================
 * WiFi Event Handler
 * ============================================================================ */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < 10) {
            esp_wifi_connect();
            wifi_retry_count++;
            ESP_LOGW(TAG, "Reintentando conexión WiFi (%d/10)", wifi_retry_count);
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Falló conexión WiFi");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ============================================================================
 * WiFi Initialization
 * ============================================================================ */

static void wifi_init(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Conectando a WiFi SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Conectado a WiFi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ Falló conexión a WiFi");
    } else {
        ESP_LOGE(TAG, "⚠️  Error inesperado en WiFi");
    }
}

/* ============================================================================
 * GPIO Functions
 * ============================================================================ */

/**
 * @brief Inicializa el GPIO del LED
 */
static void led_init(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI(TAG, "LED inicializado en GPIO %d", LED_GPIO);
}

/**
 * @brief Enciende el LED
 */
static void led_on(void)
{
    gpio_set_level(LED_GPIO, 1);
    led_state = true;
    ESP_LOGI(TAG, "🟢 LED ENCENDIDO");
}

/**
 * @brief Apaga el LED
 */
static void led_off(void)
{
    gpio_set_level(LED_GPIO, 0);
    led_state = false;
    ESP_LOGI(TAG, "🔴 LED APAGADO");
}

/**
 * @brief Alterna el estado del LED
 */
static void led_toggle(void)
{
    if (led_state) {
        led_off();
    } else {
        led_on();
    }
}

/* ============================================================================
 * micro-ROS Callback
 * ============================================================================ */

/**
 * @brief Callback cuando se recibe un mensaje en /led_control
 */
static void subscription_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    ESP_LOGI(TAG, "📨 Mensaje recibido: '%s'", msg->data.data);
    
    // Comandos de encendido
    if (strcmp(msg->data.data, "ON") == 0 || 
        strcmp(msg->data.data, "1") == 0 ||
        strcmp(msg->data.data, "on") == 0) {
        led_on();
    }
    // Comandos de apagado
    else if (strcmp(msg->data.data, "OFF") == 0 || 
             strcmp(msg->data.data, "0") == 0 ||
             strcmp(msg->data.data, "off") == 0) {
        led_off();
    }
    // Comando de toggle
    else if (strcmp(msg->data.data, "TOGGLE") == 0 ||
             strcmp(msg->data.data, "toggle") == 0) {
        led_toggle();
    }
    else {
        ESP_LOGW(TAG, "⚠️  Comando desconocido: '%s'", msg->data.data);
        ESP_LOGW(TAG, "Comandos válidos: ON, OFF, TOGGLE, 1, 0");
    }
}

/* ============================================================================
 * micro-ROS Task
 * ============================================================================ */

/**
 * @brief Tarea principal de micro-ROS
 */
void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, "  Prueba LED con micro-ROS");
    ESP_LOGI(TAG, "  GPIO: %d | Tópico: /led_control", LED_GPIO);
    ESP_LOGI(TAG, "=================================================");

    // Inicializar LED
    led_init();
    
    // Inicializar transporte
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Crear opciones de inicialización
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(AGENT_IP, AGENT_PORT, rmw_options));
#endif

    ESP_LOGI(TAG, "Conectando al agente ROS en %s:%s...", AGENT_IP, AGENT_PORT);
    
    // Ping al agente con las opciones RMW configuradas
    ESP_LOGI(TAG, "Verificando conexión con agente...");
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    while (rmw_uros_ping_agent_options(PING_TIMEOUT_MS, 1, rmw_options) != RMW_RET_OK) {
        ESP_LOGW(TAG, "Esperando agente...");
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }
#endif
    ESP_LOGI(TAG, "✅ Agente detectado");

    // Crear soporte
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Crear nodo
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "led_controller", "", &support));
    ESP_LOGI(TAG, "✅ Nodo 'led_controller' creado");

    // Crear suscriptor
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/led_control"));
    ESP_LOGI(TAG, "✅ Suscrito a /led_control");

    // Crear executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &incoming_msg, 
        &subscription_callback, 
        ON_NEW_DATA));

    // Reservar memoria para mensajes
    incoming_msg.data.data = (char *)malloc(128 * sizeof(char));
    incoming_msg.data.size = 0;
    incoming_msg.data.capacity = 128;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "🚀 Sistema listo!");
    ESP_LOGI(TAG, "📡 Esperando comandos en /led_control...");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Prueba desde otra terminal:");
    ESP_LOGI(TAG, "  ros2 topic pub /led_control std_msgs/msg/String \"data: 'ON'\"");
    ESP_LOGI(TAG, "  ros2 topic pub /led_control std_msgs/msg/String \"data: 'OFF'\"");
    ESP_LOGI(TAG, "");

    // Loop principal
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Cleanup (nunca se alcanza en este ejemplo)
    free(incoming_msg.data.data);
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

/* ============================================================================
 * Main
 * ============================================================================ */

void app_main(void)
{
    // Inicializar NVS (requerido para WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar WiFi
    wifi_init();

    // Esperar un momento para estabilizar
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Crear tarea de micro-ROS
    xTaskCreate(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_APP_STACK,
        NULL,
        CONFIG_MICRO_ROS_APP_TASK_PRIO,
        NULL);
}
