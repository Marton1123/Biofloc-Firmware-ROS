# Corrección de Conexión WiFi - WPA2/WPA3 Incompatibilidad

**Fecha**: 2026-02-11  
**Versión Firmware**: 2.1.0+wpa2fix  
**Problema**: ESP32 no se conectaba a Raspberry Pi 3 hotspot (Biofloc-Gateway)

## Diagnóstico

### Problema Identificado
El ESP32 estaba configurado para forzar **WPA3-SAE** pero el hotspot de Raspberry Pi 3 solo soporta **WPA1/WPA2**. Esto causaba incompatibilidad de seguridad WiFi.

**Evidencia**:
```bash
# sdkconfig anterior (incorrecto):
CONFIG_ESP_WIFI_ENABLE_WPA3_SAE=y
CONFIG_ESP_WIFI_ENABLE_SAE_PK=y
CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT=y
CONFIG_ESP_WIFI_ENABLE_WPA3_OWE_STA=y

# Escaneo de red mostró:
SSID: Biofloc-Gateway
Security: WPA1 WPA2  # Solo WPA1/WPA2, NO WPA3
```

### Problemas Secundarios
1. **Duplicación de variables WiFi**: `Kconfig.projbuild` definía `CONFIG_BIOFLOC_WIFI_*` pero el componente micro-ROS usa `CONFIG_ESP_WIFI_*` (estándar ESP-IDF)
2. **Falta de especificación de authmode**: El código no forzaba explícitamente WPA2-PSK
3. **Logs insuficientes**: Difícil depurar sin logs detallados de conexión

## Cambios Realizados

### 1. Desactivar WPA3 en sdkconfig.defaults

**Archivo**: `sdkconfig.defaults`

```ini
# ---- Seguridad WiFi - CRITICO: Desactivar WPA3 para compatibilidad con RPi ----
# La Raspberry Pi 3 hotspot solo soporta WPA1/WPA2, NO WPA3
CONFIG_ESP_WIFI_ENABLE_WPA3_SAE=n
CONFIG_ESP_WIFI_ENABLE_WPA3_OWE_STA=n
```

### 2. Eliminar Variables WiFi Duplicadas

**Archivo**: `main/Kconfig.projbuild`

**Antes**:
```kconfig
menu "WiFi Configuration"
    config BIOFLOC_WIFI_SSID
        string "WiFi SSID"
        default "<ssid-anterior>"
    
    config BIOFLOC_WIFI_PASSWORD
        string "WiFi Password"
        default "<password-anterior>"
    ...
endmenu
```

**Después**:
```kconfig
# ==============================================================================
# NOTA: WiFi usa CONFIG_ESP_WIFI_* (estándar ESP-IDF) definidas en menuconfig
# No usar CONFIG_BIOFLOC_WIFI_* para evitar confusión con micro_ros component
# ==============================================================================
```

**Razón**: El componente `micro_ros_espidf_component` usa las variables estándar de ESP-IDF (`CONFIG_ESP_WIFI_SSID`, `CONFIG_ESP_WIFI_PASSWORD`, `CONFIG_ESP_MAXIMUM_RETRY`), no las personalizadas.

### 3. Forzar WPA2-PSK en Configuración WiFi

**Archivo**: `components/micro_ros_espidf_component/network_interfaces/uros_wlan_netif.c`

```c
wifi_config_t wifi_config = {
    .sta = {
        .ssid = ESP_WIFI_SSID,
        .password = ESP_WIFI_PASS,
        /* CRITICAL: Force WPA2-PSK for Raspberry Pi 3 hotspot compatibility */
        /* RPi hotspot only supports WPA1/WPA2, NOT WPA3 */
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        .pmf_cfg = {
            .capable = true,
            .required = false
        },
        ...
    },
};
```

**Parámetros**:
- `threshold.authmode = WIFI_AUTH_WPA2_PSK`: Acepta WPA2 o superior (pero WPA3 deshabilitado por sdkconfig)
- `pmf_cfg.capable = true`: Soporta Protected Management Frames (opcional en WPA2)
- `pmf_cfg.required = false`: No exige PMF (compatibilidad con APs antiguos)

### 4. Logs de Conexión Mejorados

**Archivo**: `components/micro_ros_espidf_component/network_interfaces/uros_wlan_netif.c`

**Event Handler mejorado**:
```c
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "🔄 WiFi started, attempting connection to SSID: %s", ESP_WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconn_event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW(TAG, "❌ WiFi disconnected (reason: %d)", disconn_event->reason);
        
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "🔄 Retry %d/%d to connect to SSID: %s", 
                     s_retry_num, ESP_MAXIMUM_RETRY, ESP_WIFI_SSID);
        } else {
            ESP_LOGE(TAG, "❌ Max retries (%d) reached, giving up", ESP_MAXIMUM_RETRY);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ...
        ESP_LOGI(TAG, "✅ WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ...
    }
}
```

**Logs de inicialización**:
```c
ESP_LOGI(TAG, "📡 WiFi STA mode initialized");
ESP_LOGI(TAG, "🔑 Target SSID: %s", ESP_WIFI_SSID);
ESP_LOGI(TAG, "🔒 Security: WPA2-PSK (RPi compatible)");
```

## Resultado

### Logs del ESP32
```
I (1259) wifi_station_netif: 📡 WiFi STA mode initialized
I (1259) wifi_station_netif: 🔑 Target SSID: Biofloc-Gateway
I (1260) wifi_station_netif: 🔄 WiFi started, attempting connection to SSID: Biofloc-Gateway
I (1275) wifi_station_netif: 🔒 Security: WPA2-PSK (RPi compatible)
```

### Estado Actual
El firmware ahora:
- ✅ Desactiva WPA3 correctamente
- ✅ Fuerza modo WPA2-PSK
- ✅ Muestra logs detallados de conexión
- ⚠️ **Error 211 (WIFI_REASON_NO_AP_FOUND)**: El AP no es encontrado

**Error 211** significa que el ESP32 no encuentra el SSID "Biofloc-Gateway" en el escaneo WiFi, a pesar de que anteriormente era visible desde la NUC.

## Siguientes Pasos

1. **Verificar que el hotspot de Raspberry Pi esté corriendo**:
   ```bash
   ssh user@raspberry-pi-ip
   nmcli connection show
   nmcli device status
   iwconfig wlan0
   ```

2. **Verificar configuración del hotspot**:
   ```bash
   nmcli connection show Biofloc-Gateway
   # Verificar:
   # - SSID: "Biofloc-Gateway"
   # - Password: correcto
   # - Mode: AP
   # - Band: 2.4GHz (ESP32 no soporta 5GHz)
   ```

3. **Posibles causas del error 211**:
   - Hotspot no está corriendo
   - Hotspot configurado en 5GHz (ESP32 solo soporta 2.4GHz)
   - SSID oculto
   - Canal WiFi incompatible
   - Potencia de señal muy baja

4. **Escanear desde la NUC de nuevo**:
   ```bash
   sudo nmcli device wifi list
   # Verificar que Biofloc-Gateway aparece y qué canal/frecuencia usa
   ```

## Códigos de Error WiFi

| Código | Macro | Significado |
|--------|-------|-------------|
| 1 | WIFI_REASON_UNSPECIFIED | Error no especificado |
| 2 | WIFI_REASON_AUTH_EXPIRE | Autenticación expirada |
| 3 | WIFI_REASON_AUTH_LEAVE | Desautenticado (desconexión limpia) |
| 15 | WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT | Timeout en handshake WPA (password incorrecto) |
| 201 | WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY | AP encontrado pero seguridad incompatible |
| **211** | **WIFI_REASON_NO_AP_FOUND** | **AP no encontrado en escaneo** |

## Comandos de Compilación

```bash
cd /home/Biofloc-Firmware-ROS

# Limpiar configuración anterior
rm -f sdkconfig

# Regenerar desde sdkconfig.defaults
source ~/esp/v5.3.4/esp-idf/export.sh
idf.py reconfigure

# Compilar
idf.py build

# Verificar WPA3 deshabilitado
grep "CONFIG_ESP_WIFI_ENABLE_WPA3_SAE" sdkconfig
# Debe mostrar: # CONFIG_ESP_WIFI_ENABLE_WPA3_SAE is not set

# Flashear
idf.py -p /dev/ttyUSB0 flash

# Monitorear
idf.py -p /dev/ttyUSB0 monitor
```

## Referencias

- [ESP-IDF WiFi Driver](https://docs.espressif.com/projects/esp-idf/en/v5.3.4/esp32/api-reference/network/esp_wifi.html)
- [ESP32 WiFi Reasons](https://docs.espressif.com/projects/esp-idf/en/v5.3.4/esp32/api-guides/wifi.html#wi-fi-reason-code)
- [WPA3 Configuration](https://docs.espressif.com/projects/esp-idf/en/v5.3.4/esp32/api-guides/wifi-security.html)
