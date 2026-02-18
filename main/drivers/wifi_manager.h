/**
 * @file wifi_manager.h
 * @brief WiFi connection manager - Clean API for network connectivity
 * @version 4.0.0
 * 
 * Responsabilidades:
 * - Inicialización WiFi (STA mode)
 * - Conexión automática con retry
 * - Monitoreo de estado
 * - Reconexión en caso de pérdida
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"
#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Inicializa subsistema WiFi y conecta al AP configurado
 * 
 * @return ESP_OK si WiFi conectado exitosamente
 * @return ESP_FAIL si falla inicialización o conexión
 * 
 * @note Bloqueante - retorna solo cuando WiFi está conectado o falla
 * @note Configuración tomada de core/config.h (SSID, password)
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Obtiene estado actual de WiFi
 * 
 * @return wifi_state_t Estado actual (DISCONNECTED, CONNECTING, CONNECTED, ERROR)
 */
wifi_state_t wifi_manager_get_state(void);

/**
 * @brief Obtiene información de la conexión WiFi actual
 * 
 * @param[out] ip_str Buffer para dirección IP (mínimo 16 bytes)
 * @param[out] mac_str Buffer para dirección MAC (mínimo 18 bytes)
 * @param[out] rssi RSSI de la señal WiFi en dBm
 * 
 * @return ESP_OK si WiFi conectado y datos disponibles
 * @return ESP_ERR_INVALID_STATE si WiFi no conectado
 */
esp_err_t wifi_manager_get_info(char *ip_str, char *mac_str, int8_t *rssi);

/**
 * @brief Reconecta WiFi indefinidamente hasta tener éxito
 * 
 * @note Bloqueante - retorna solo cuando WiFi está reconectado
 * @note Usado en caso de pérdida de conexión detectada por ping
 */
void wifi_manager_reconnect_forever(void);

/**
 * @brief Deinicializa subsistema WiFi y libera recursos
 */
void wifi_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H
