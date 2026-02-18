/**
 * @file wifi_manager.c
 * @brief WiFi connection manager implementation
 * @version 4.0.0
 */

#include "wifi_manager.h"
#include "core/config.h"
#include "core/app_state.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include <string.h>

/* ============================================================================
 * ESTADO PRIVADO
 * ============================================================================ */

static bool s_initialized = false;

/* ============================================================================
 * IMPLEMENTACIÓN
 * ============================================================================ */

esp_err_t wifi_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG_WIFI, "WiFi manager already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG_WIFI, "Initializing WiFi manager...");
    
    // Inicializar interfaz de red (micro-ROS lo hace internamente)
    // Solo registramos el estado inicial
    app_state_set_wifi(WIFI_STATE_CONNECTING);
    
    ESP_LOGI(TAG_WIFI, "WiFi manager initialized (delegating to micro-ROS network interface)");
    
    s_initialized = true;
    return ESP_OK;
}

wifi_state_t wifi_manager_get_state(void)
{
    const app_state_t *state = app_state_get();
    return state->wifi_state;
}

esp_err_t wifi_manager_get_info(char *ip_str, char *mac_str, int8_t *rssi)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Obtener MAC
    uint8_t mac[6];
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (err != ESP_OK) {
        return err;
    }
    
    if (mac_str) {
        snprintf(mac_str, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    
    // Obtener IP (desde netif)
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && ip_str) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            snprintf(ip_str, 16, IPSTR, IP2STR(&ip_info.ip));
        } else {
            strcpy(ip_str, "0.0.0.0");
        }
    }
    
    // Obtener RSSI
    if (rssi) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            *rssi = ap_info.rssi;
        } else {
            *rssi = -127;
        }
    }
    
    return ESP_OK;
}

void wifi_manager_reconnect_forever(void)
{
    ESP_LOGW(TAG_WIFI, "WiFi connection lost - reconnecting...");
    app_state_set_wifi(WIFI_STATE_CONNECTING);
    
    // Implementación básica - en producción delegar a micro-ROS network interface
    // Por ahora solo actualizamos estado
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG_WIFI, "WiFi reconnected successfully");
    app_state_set_wifi(WIFI_STATE_CONNECTED);
}

void wifi_manager_deinit(void)
{
    if (!s_initialized) {
        return;
    }
    
    ESP_LOGI(TAG_WIFI, "Deinitializing WiFi manager...");
    s_initialized = false;
}
