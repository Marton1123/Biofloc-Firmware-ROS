/**
 * @file    main_simple.c  
 * @brief   Prueba simple de LED sin micro-ROS
 * 
 * Este código solo hace parpadear el LED integrado para verificar
 * que el hardware funciona correctamente.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_GPIO 2
static const char *TAG = "LED_SIMPLE";

void app_main(void)
{
    ESP_LOGI(TAG, "==================================");
    ESP_LOGI(TAG, "  Prueba Simple de LED");
    ESP_LOGI(TAG, "  GPIO: %d", LED_GPIO);
    ESP_LOGI(TAG, "==================================");

    // Configurar GPIO
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    
    ESP_LOGI(TAG, "LED configurado, iniciando parpadeo...");
    
    bool led_state = false;
    while (1) {
        gpio_set_level(LED_GPIO, led_state);
        ESP_LOGI(TAG, "LED: %s", led_state ? "ON 🟢" : "OFF 🔴");
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 segundo
    }
}
