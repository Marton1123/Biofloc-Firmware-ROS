# Changelog

Todos los cambios notables de este proyecto serán documentados en este archivo.

El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.0.0/),
y este proyecto adhiere a [Semantic Versioning](https://semver.org/lang/es/).

## [3.2.1] - 2026-02-17

### Resumen
**HOTFIX CRÍTICO + ARQUITECTURA DIGITAL TWIN:** Corregido crash PANIC durante calibración remota y agregado respaldo profesional de calibraciones en MongoDB Atlas. El ESP32 ahora procesa calibraciones sin crashes y MongoDB actúa como "single source of truth" para recuperación.

### 🐛 Corregido - ESP32 PANIC Durante Calibración
- **PROBLEMA CRÍTICO**: ESP32 crasheaba con PANIC al recibir comando de calibración de 3 puntos
  - `Reset Reason: PANIC` después de enviar JSON
  - Firmware se reiniciaba inmediatamente sin procesar calibración
  - NVS no se actualizaba, perdiendo calibración completa
- **ROOT CAUSE**: 
  - `cJSON_GetObjectItem()` retornaba `NULL` sin validación
  - `cJSON_IsNumber()` era llamado en puntero NULL → segmentation fault
  - Struct `calibration_response_t` sin inicializar contenía basura
  - Loop sin flag de error continuaba con datos corruptos
- **SOLUCIÓN EN `main/main.c`**:
  ```c
  // ✅ Validación completa de punteros NULL
  cJSON *point = cJSON_GetArrayItem(points_json, i);
  if (!point) {
      ESP_LOGE(TAG_UROS, "NULL point at index %d", i);
      parse_error = true;
      break;
  }
  
  // ✅ Verificar existencia Y tipo antes de acceder
  cJSON *voltage = cJSON_GetObjectItem(point, "voltage");
  if (!voltage || !cJSON_IsNumber(voltage)) {
      parse_error = true;
      break;
  }
  
  // ✅ Inicialización segura de struct
  calibration_response_t cal_response;
  memset(&cal_response, 0, sizeof(cal_response));
  ```
- **RESULTADO**: 
  - ✅ ESP32 procesa calibración sin crashes
  - ✅ Validación robusta de JSON con parse_error flag
  - ✅ Logs detallados: "Point 1: 2.053V → 4.46 pH"
  - ✅ Reset Reason permanece en POWER_ON (no PANIC)

### ✨ Agregado - Arquitectura Digital Twin con MongoDB
- **NUEVA FEATURE**: Respaldo automático de calibraciones en MongoDB Atlas
  - Función `save_calibration_to_mongodb()` en `biofloc_manager.py`
  - Calibración guardada en colección `devices` después de respuesta exitosa
  - MongoDB como "single source of truth" (no solo NVS del ESP32)
  - Permite recuperación si NVS se corrompe
- **FLUJO**:
  ```
  1. CLI Manager lee voltajes (rclpy nativo)
  2. Publica comando → /biofloc/calibration_cmd
  3. ESP32 procesa y responde → /biofloc/calibration_status
  4. CLI Manager guarda en MongoDB (Digital Twin) ✨
  5. NVS + MongoDB sincronizados
  ```
- **SCHEMA DE MONGODB** (`devices` collection):
  ```json
  {
    "_id": "biofloc_esp32_c8e0",
    "calibracion": {
      "ph": {
        "fecha": "2026-02-17T12:45:00Z",
        "slope": 6.2558,
        "offset": -8.6328,
        "r_squared": 0.9997,
        "points": [
          {"voltage": 2.053, "value": 4.46},
          {"voltage": 2.49, "value": 7.2},
          {"voltage": 2.914, "value": 9.85}
        ],
        "num_points": 3,
        "status": "success"
      }
    },
    "ultima_calibracion": "2026-02-17T12:45:00Z"
  }
  ```

### ⚡ Mejorado - Logging y Diagnósticos
- **Logs mejorados en ESP32**:
  - Tamaño del mensaje JSON recibido
  - Puntos de calibración con formato: "2.053V → 4.46 pH"
  - Resultado de calibración con símbolos UTF-8: ✓/✗
  - "Starting calibration for ph with 3 points"
  - "✓ Calibration SUCCESS: R²=0.9997, slope=6.2558"
- **Logs mejorados en Python**:
  - "✓ Calibración guardada en MongoDB (Digital Twin)"
  - "Device: biofloc_esp32_c8e0"
  - "Sensor: ph"
  - "R²: 0.9997"

### 📚 Documentación
- **Nuevo**: [docs/DIGITAL_TWIN_ARCHITECTURE.md](docs/DIGITAL_TWIN_ARCHITECTURE.md)
  - Diagrama completo de arquitectura
  - Flujo de calibración paso a paso
  - Schema de MongoDB documentado
  - Guía de troubleshooting (PANIC scenarios)
  - Testing procedures
  - Recovery mechanisms (futuro)

### 🔧 Dependencias
- **Python**: Agregado `import json` y `from datetime import datetime` en biofloc_manager.py
- **MongoDB**: Requiere `pymongo` instalado (`pip install pymongo`)
- **Variables de entorno**:
  - `MONGODB_URI`: Conexión a MongoDB Atlas
  - `MONGODB_DATABASE`: Nombre de base de datos (default: SistemasLab)
  - `MONGODB_COLLECTION_DEVICES`: Colección de dispositivos (default: devices)

### 🐛 Conocido
- **Degradación graceful**: Si MongoDB no está configurado, calibración funciona normal (solo NVS)
- **Advertencia**: "MONGODB_URI no configurado - calibración solo guardada en ESP32"

## [3.1.1] - 2026-02-12

### Resumen
**HOTFIX CRÍTICO:** Eliminación del boot loop causado por `esp_restart()` en caso de desconexión. Estrategia de reconexión infinita sin reinicio del ESP32. Mejoras de robustez para producción.

### 🐛 Corregido - Boot Loop Crítico
- **PROBLEMA**: ESP32 entraba en boot loop infinito cuando perdía conexión con el Agent
  - `esp_restart()` en `micro_ros_task()` línea 495-498
  - `esp_restart()` en bucle principal línea 596-600
  - Causaba reinicios constantes sin resolver el problema
- **ROOT CAUSE**: Lógica de reconexión terminaba con reinicio en lugar de espera
- **SOLUCIÓN**:
  - ✅ Eliminado completamente `esp_restart()` de ambos puntos
  - ✅ Nueva función `reconnect_forever()` - reconexión infinita sin reinicio
  - ✅ Exponential backoff: 3s → 6s → 12s → 24s → 48s → 60s (cap)
  - ✅ ESP32 nunca se reinicia, espera indefinidamente al Agent
  - ✅ Cuando Agent vuelve, retoma operación normal automáticamente

### ⚡ Mejorado - Robustez de Conexión
- **Timeouts aumentados para redes lentas**:
  - `PING_TIMEOUT_MS`: 5000ms → **10000ms** (10 segundos)
  - `PING_RETRIES`: 3 → **5** intentos
  - `RECONNECT_DELAY_MAX`: 30s → **60s** (máximo entre reintentos)
- **Reconexión en startup**:
  - Si Agent no responde al iniciar, ESP32 espera en lugar de reiniciar
  - Bucle de espera de 5s hasta que Agent esté disponible
  - LOG: "ESP32 will NOT restart - waiting for Agent..."

### ✨ Agregado - Inicialización Explícita de NVS
- **Verificación de NVS antes de cargar calibraciones**:
  ```c
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || 
      nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase();
      nvs_err = nvs_flash_init();
  }
  ```
- **Manejo de errores NVS**:
  - Si NVS falla, firmware continúa sin persistencia
  - LOG: "⚠ NVS initialization failed - calibrations won't persist"
  - No bloquea operación del ESP32

### 📝 Documentación
- **Changelog actualizado**: Versión 3.1.0 → 3.1.1
- **Comentarios en código**: Explicación de estrategia anti-bootloop
- **Logs mejorados**: Emojis para mejor visibilidad (✅, ⚠️, ❌)

### 🔄 Compatibilidad
- ✅ 100% compatible con `biofloc_manager.py` v1.1
- ✅ Protocolo JSON sin cambios
- ✅ Topics ROS 2 sin cambios:
  - `/biofloc/sensor_data` (Publisher)
  - `/biofloc/calibration_cmd` (Subscriber)
  - `/biofloc/calibration_status` (Publisher)

### 📊 Estadísticas de Código
- **Archivos modificados**: 2
  - `main/main.c`: +47/-31 líneas
  - `main/sensors.c`: +15/-3 líneas
- **Total cambios**: +62/-34 líneas (+28 neto)
- **Tamaño firmware**: ~805 KB (sin cambios)
- **RAM libre**: 60% (sin cambios)

### ⚠️ Breaking Changes
**NINGUNO** - Actualización compatible sin reconfiguración.

### 🎯 Probado
- ✅ Compilación exitosa con ESP-IDF v5.3.4
- ⏳ **Pendiente**: Test en hardware con desconexión/reconexión del Agent
- ⏳ **Pendiente**: Test de persistencia NVS tras reinicio manual

---

## [3.0.0] - 2026-02-11

### Resumen
**MAJOR RELEASE:** Migración completa de NUC a Raspberry Pi 3 como Gateway IoT. ESP32 opera sin acceso a internet (aislado por firewall iptables). Solución definitiva de conectividad WiFi WPA1/WPA2. Gestor unificado con interfaz en español. Corrección de 3 errores críticos de calibración basados en verificación de hardware desde foto de PCB.

### 🚀 Migrado - Raspberry Pi 3 como Gateway
- **Hardware Gateway**: NUC → Raspberry Pi 3 Model B/B+
  - CPU: ARM Cortex-A53 quad-core 1.2GHz
  - RAM: 1GB
  - WiFi: 2.4GHz 802.11n (built-in)
  - OS: Ubuntu Server 24.04 LTS ARM64
  - ROS 2: Jazzy Jalisco
- **Hotspot WiFi optimizado**:
  - SSID: Biofloc-Gateway (configurable)
  - Security: WPA1+WPA2-PSK (compatibilidad ESP32)
  - Channel: 1 (2.4GHz fijo)
  - Network: 10.42.0.1/24
  - Interface: wlan0 (antes wlp0s20f3 en NUC)
- **Firewall**: ESP32 bloqueado de internet, solo acceso a Gateway
- **Documentación**: Guía completa de migración (390 líneas)
  - Instalación ROS 2 en ARM64
  - Configuración hotspot con nmcli
  - Setup de firewall iptables
  - Troubleshooting específico de RPi

### 🐛 Corregido - Conectividad WiFi (Error 211/201)
- **PROBLEMA CRÍTICO**: ESP32 no se conectaba a Raspberry Pi hotspot
  - Error 211: WIFI_REASON_NO_AP_FOUND (AP no encontrado)
  - Error 201: WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY (seguridad incompatible)
- **ROOT CAUSE**: Incompatibilidad WPA3 ↔ WPA1/WPA2
  - ESP32 tenía `CONFIG_ESP_WIFI_ENABLE_WPA3_SAE=y` (forzaba WPA3)
  - Raspberry Pi hotspot solo soporta WPA1/WPA2
  - `threshold.authmode = WIFI_AUTH_WPA2_PSK` rechazaba WPA1
- **SOLUCIÓN DEFINITIVA**:
  1. **Desactivar WPA3 en sdkconfig.defaults**:
     ```ini
     CONFIG_ESP_WIFI_ENABLE_WPA3_SAE=n
     CONFIG_ESP_WIFI_ENABLE_WPA3_OWE_STA=n
     ```
  2. **Ajustar threshold.authmode en uros_wlan_netif.c**:
     ```c
     .threshold.authmode = WIFI_AUTH_WPA_PSK,  // Acepta WPA1 y WPA2
     ```
  3. **Escaneo agresivo**:
     ```c
     .threshold.rssi = -127,                   // RSSI mínimo
     .scan_method = WIFI_ALL_CHANNEL_SCAN,    // Todos los canales
     .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,// Mejor señal
     .listen_interval = 3,                     // 3 beacon intervals
     ```
- **RESULTADO**:
  - ✅ Conexión exitosa: ESP32 → Biofloc-Gateway
  - ✅ Security: WPA-PSK (compatible WPA1+WPA2)
  - ✅ Signal: RSSI -42 dBm (excelente)
  - ✅ micro-ROS Agent: ONLINE (10.42.0.1:8888)
  - ✅ Topic: /biofloc/sensor_data publicando

### 🔒 Seguridad - Arquitectura Gateway
- **Gateway IoT Seguro**: ESP32 aislado de internet mediante firewall iptables
  - Hotspot WiFi en Raspberry Pi 3 (SSID: Biofloc-Gateway, red 10.42.0.1/24)
  - Firewall iptables con política FORWARD DROP
  - ESP32 solo puede comunicarse con gateway (UDP 8888 para micro-ROS)
  - Gateway tiene doble conexión: WiFi para ESP32 + Ethernet para servicios cloud
- **Variables WiFi unificadas**: Se eliminó duplicación CONFIG_BIOFLOC_WIFI_*
  - Solo se usan CONFIG_ESP_WIFI_* (estándar ESP-IDF)
  - micro_ros_espidf_component lee estas variables automáticamente
  - Configuración centralizada en sdkconfig.defaults

### 🛠️ Agregado - Gestor Unificado
- **biofloc_manager.py** (v1.0.0, 820 líneas):
  - **12 opciones de menú** en interfaz completamente en español
  - **Detección automática de Raspberry Pi**: Modo Gateway sin compilación ESP-IDF
  - **Operaciones del Sistema**: Iniciar Agent, Bridge, Monitor
  - **Verificación**: Estado completo (8s) y conectividad ESP32 (DHCP/ARP/ping/ROS)
  - **Calibración**: pH y Temperatura (3 puntos + ajustes rápidos)
  - **Configuración**: WiFi (credentials unificadas) y regeneración de sdkconfig
  - **Firmware**: Pipeline completo de build/flash (solo en NUC)
  - **Timeouts inteligentes**: 8s para verificación rápida, 20s opcional para tasa
  - **Manejo robusto**: Filtrado en Python (no pipes grep), try-except en todos los subprocess

### ⚙️ Cambiado - Configuración
- **sdkconfig.defaults** ahora es **single source of truth**
  - Todos los valores deben configurarse aquí primero
  - Workflow: editar defaults → rm sdkconfig → idf.py reconfigure
  - Previene drift de configuración entre archivos
- **Agent IP**: Cambiado de red externa a 10.42.0.1 (gateway interno)
- **ESP32 IP**: Asignada por DHCP del gateway (típicamente 10.42.0.123)

### 🐛 Corregido - Calibración (3 errores críticos)
- **ERROR 1 - pH Divisor**: Factor 3.0 → 1.5 (hardware verificado: R1=10kΩ, R2=20kΩ)
- **ERROR 2 - Temp Divisor**: Factor 3.0 → 1.5 (mismo hardware que pH)
- **ERROR 3 - Temp Offset Sign**: -423 → +1382 millidegrees (signo invertido)
- **Root Cause**: Foto de PCB reveló que R1 y R2 estaban invertidos respecto a la asunción inicial
- **Hardware documentado**: 
  - R1 = 10kΩ (pull-up desde Vin del sensor)
  - R2 = 20kΩ (pull-down a GND)
  - Factor = (R1+R2)/R2 = 30k/20k = 1.5

### ⏱️ Cambiado - Gestión de Tiempo
- **NTP eliminado del ESP32**: Opera sin acceso a servidores de tiempo
- **Timestamps de contador**: ESP32 usa `sample_XXXX` incremental
- **Timestamps del servidor**: Gateway agrega timestamps reales (UTC) antes de MongoDB
- **Formato en MongoDB**:
  ```json
  {
    "timestamp": "2026-02-10T14:32:15.847Z",     // Del servidor (real)
    "timestamp_esp32": "sample_1523"              // Del ESP32 (contador)
  }
  ```

### 📚 Documentación
- **docs/guides/RASPBERRY_PI_MIGRATION.md**: Guía completa de migración (390 líneas)
  - Instalación ROS 2 Jazzy en Ubuntu Server 24.04 ARM64
  - Configuración hotspot WiFi con nmcli (2.4GHz, WPA2)
  - Setup de firewall iptables para aislamiento ESP32
  - Flashing de firmware desde NUC
  - Troubleshooting específico de conectividad
- **docs/CHANGELOG_WPA2_FIX.md**: Documentación técnica del fix WiFi
  - Diagnóstico de incompatibilidad WPA3 ↔ WPA1/WPA2
  - Códigos de error WiFi (211, 201)
  - Solución paso a paso con diffs de código
  - Comandos de verificación
- **Actualización masiva de documentación**:
  - README.md: Arquitectura Raspberry Pi, diagrama actualizado
  - TECHNICAL_SUMMARY.md: Valores corregidos, red 10.42.0.0/24
  - GUIA_PASO_A_PASO.md: Uso del gestor, configuración de gateway RPi
  - DOCUMENTATION_INDEX.md: Referencias actualizadas, link a guía RPi
  - PROJECT_STATUS.md: Métricas de migración, estado actual
  - QUICKSTART.md: Configuración de gateway RPi incluida

### 🔧 Técnico
- **WiFi Security**: WPA3 deshabilitado, WPA_PSK como threshold mínimo
- **WiFi Scan**: ALL_CHANNEL_SCAN, RSSI threshold=-127, sort by signal
- **WiFi Config**: listen_interval=3, PMF capable but not required
- **CONFIG_ESP_WIFI_ENABLE_WPA3_SAE**: y → n (compatibilidad RPi)
- **CONFIG_ESP_WIFI_ENABLE_WPA3_OWE_STA**: y → n
- **CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR**: 3000 → 1500
- **CONFIG_BIOFLOC_TEMP_VOLTAGE_DIVIDER_FACTOR**: 3000 → 1500
- **CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES**: -423 → 1382
- **CONFIG_MICRO_ROS_AGENT_IP**: Variable externa → "10.42.0.1"
- **Subprocess handling**: Pipes con grep eliminados, filtrado en Python
- **Error handling**: Doble shutdown de rclpy corregido en monitor_sensores.py
- **Kconfig.projbuild**: Variables CONFIG_BIOFLOC_WIFI_* eliminadas (redundantes)

### 🎯 Métricas
- **Conectividad WiFi**: ✅ 100% exitosa tras fix WPA
- **Signal Strength**: -42 dBm (excelente)
- **micro-ROS Agent**: ✅ ONLINE (latencia <10ms)
- **Precisión pH**: ±0.05 pH (con divisor corregido)
- **Precisión Temperatura**: ~±1.6°C error residual (ajustable con gestor opción [9])
- **Tiempo de verificación**: 8s (vs 10+ minutos anterior)
- **Uptime gateway**: 24/7
- **Seguridad**: ESP32 completamente bloqueado de internet
- **Compatibilidad**: Raspberry Pi 3/4, NUC, cualquier Linux con ROS 2

### ⚠️ Breaking Changes
- **Requiere Raspberry Pi 3+ o Linux con WiFi 2.4GHz**: NUC opcional solo para desarrollo
- **Hotspot debe ser WPA1/WPA2**: WPA3 no soportado por ESP32
- **Canal WiFi fijo en 1-11**: 2.4GHz solamente (ESP32 no soporta 5GHz)
- **Variables CONFIG_BIOFLOC_WIFI_* eliminadas**: Usar solo CONFIG_ESP_WIFI_*
- **Agent IP hardcoded**: 10.42.0.1 (no configurable desde red externa)
- **NTP no disponible**: ESP32 no puede sincronizar tiempo (usa contador)
- **sdkconfig.defaults obligatorio**: Regenerar sdkconfig requiere tener valores en defaults

### 📦 Archivos Movidos/Limpiados
- **docs/archive/calibration_old/**: calibration_result.txt, calibration_3point_result.txt
- **docs/archive/**: sdkconfig.old, sdkconfig.old.backup
- **Logs temporales**: /tmp/esp32_*.log eliminados
- **Documentación consolidada**: Guías organizadas en docs/guides/

### 📖 Migración desde v2.x
Ver [MIGRATION_GUIDE_SECURE_GATEWAY.md](MIGRATION_GUIDE_SECURE_GATEWAY.md) para:
1. Configuración completa del gateway (hotspot + firewall)
2. Actualización de credenciales WiFi (dual)
3. Eliminación de NTP del código
4. Cambio de IP del agente
5. Actualización de scripts para timestamps del servidor

---

## [3.2.1] - 2026-02-05

### Resumen
Nueva herramienta de análisis de ciclos de pH para detectar patrones fotosintéticos en cultivos de microalgas. Permite diagnosticar si las oscilaciones de pH son fenómenos biológicos normales o problemas de sensores.

### Agregado
- **scripts/check_ph_cycles.py**: Herramienta de análisis de ciclos circadianos de pH
  - Análisis de pH por hora del día (últimas 96 horas configurables)
  - Detección automática de patrón fotosintético
  - Comparación madrugada temprana (23:00-02:00) vs mediodía solar (10:00-13:00)
  - Cálculo de amplitud total del ciclo (pH máximo - pH mínimo)
  - Detección de outliers (pH < 3 o > 12)
  - Configuración de período de análisis (pre-mantenimiento vs tiempo real)
  - Documentación completa del fenómeno biológico esperado

### Características del análisis
- **Período configurable**: Análisis de 72-96 horas
- **Timezone-aware**: Maneja correctamente zonas horarias (Chile GMT-3)
- **Estructura anidada**: Compatible con formato MongoDB `sensors.ph.value`
- **Detección inteligente**: Identifica patrones de fotosíntesis vs anomalías
- **Interpretación automática**: Explica los resultados en contexto biológico

### Interpretación de resultados
```
Patrón Fotosintético Detectado:
  - pH máximo: 10:00-13:00 (pico solar)
  - Amplitud total del ciclo: >0.12 pH
  - Diferencia madrugada→mediodía: >0.08 pH
  - Indica: Cultivo con actividad fotosintética
  
Niveles de actividad:
  - Amplitud >0.3 pH: Cultivo muy activo, alta biomasa
  - Amplitud 0.12-0.3 pH: Cultivo activo, densidad moderada
  - Amplitud <0.12 pH: Baja biomasa o sistema muy tamponado

Patrón Atípico:
  - Amplitud <0.08 pH: Patrón muy débil o ausente
  - pH máximo fuera de 10:00-14:00: Posible iluminación artificial
  - Patrón inverso: Interferencia de equipos/mantenimiento
```

### Casos de uso documentados
1. **Diagnóstico de sensor**: Distinguir fallo de sensor vs fenómeno biológico
2. **Salud del cultivo**: Evaluar actividad fotosintética
3. **Optimización**: Correlacionar pH con densidad de microalgas
4. **Troubleshooting**: Identificar períodos de mantenimiento que afectan lecturas

### Resultados del análisis inicial
- **Período normal (29 ene - 1 feb)**: Patrón fotosintético detectado
  - pH máximo: 7.10 a las 11:00
  - pH mínimo: 6.94 a las 17:00
  - Amplitud del ciclo: 0.153 pH
  - Diferencia madrugada→mediodía: +0.073 pH
  - Amplitud: ~0.4 pH (normal para cultivos densos)
  - Ritmo circadiano claro y consistente
- **Período mantenimiento (2-5 feb)**: Sin patrón significativo
  - Confirmó que las anomalías eran por manipulación de sensores

### Dependencias
- pymongo: Conexión a MongoDB Atlas
- python-dateutil: Parsing de timestamps ISO
- pytz: Manejo de zonas horarias
- python-dotenv: Variables de entorno

## [3.2.0] - 2026-02-04

### Resumen
Agregado proyecto de ejemplo `test_led_project` para demostrar control de GPIO mediante micro-ROS con teclado. Proyecto educativo completamente documentado y funcional.

### Agregado
- **test_led_project/**: Proyecto de ejemplo completo
  - Control de LED via micro-ROS desde teclado de la computadora
  - Firmware ESP32 con suscripción a tópico `/led_control`
  - Script Python `keyboard_led_control.py` para control interactivo
  - Comandos soportados: ON, OFF, TOGGLE
  - GPIO configurable (por defecto GPIO 2)

- **Documentación completa del test_led_project:**
  - `README.md`: Guía completa de instalación y uso
  - `QUICKSTART.md`: Guía rápida de 5 minutos
  - `DESARROLLO.md`: Historial completo del desarrollo (164 minutos)
  - `SOLUCION_PROBLEMA_CONEXION.md`: Solución al bug crítico de conexión
  - `ARQUITECTURA_Y_FUNCIONAMIENTO.md`: Explicación técnica de ROS 2, micro-ROS, DDS vs XRCE-DDS (800+ líneas)

- **Archivos de configuración:**
  - `CMakeLists.txt`: Configuración del proyecto y main
  - `sdkconfig.defaults.example`: Plantilla de configuración
  - `check_network.sh`: Script de diagnóstico de red
  - `main_simple.c`: Firmware de prueba simple sin ROS

### Corregido
- **Bug crítico de conexión micro-ROS:**
  - **Problema:** ESP32 se conectaba a WiFi pero no al agente micro-ROS
  - **Causa:** Uso de `rmw_uros_ping_agent()` en lugar de `rmw_uros_ping_agent_options()`
  - **Solución:** Usar `rmw_uros_ping_agent_options(timeout, attempts, rmw_options)`
  - **Impacto:** Sin esto, el ESP32 nunca detecta al agente cuando se usa configuración UDP personalizada
  - **Documentación:** Solución completamente documentada en `SOLUCION_PROBLEMA_CONEXION.md`

### Seguridad
- **Sanitización de credenciales:**
  - Archivo `sdkconfig.defaults.example` con placeholders
  - `sdkconfig.defaults` actualizado con credenciales genéricas
  - `.gitignore` actualizado para prevenir commit de archivos sensibles

### Detalles técnicos
- **Implementación:**
  - Firmware: 838,896 bytes (20% de partición libre)
  - Protocolo: XRCE-DDS sobre UDP puerto 8888
  - Latencia: ~50ms desde tecla hasta LED
  - QoS: Best Effort con History 10

- **Aprendizajes documentados:**
  - Comparación DDS vs XRCE-DDS (92% menos overhead)
  - Flujo completo de mensaje (17 pasos)
  - Arquitectura de callbacks y executors
  - Debugging con tcpdump y ros2 tools

## [3.1.0] - 2026-01-29

### Resumen
Implementación completa del sistema de calibración de temperatura con slope+offset mediante regresión lineal de 3 puntos. Precisión mejorada a ≤0.03°C (R²=0.999999).

### Agregado
- **Sistema de calibración de temperatura:**
  - Calibración de 3 puntos con regresión lineal (método similar al de pH)
  - Configuración automática desde Kconfig: `CONFIG_BIOFLOC_TEMP_SLOPE` y `CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES`
  - Aplicación automática en `sensors_read_temperature()`: `T_cal = slope × T_raw + offset`
  - Logs con marcador `[CAL]` cuando la calibración está activa
  - Valores almacenados en `sdkconfig.defaults` para persistencia

- **Herramientas de calibración:**
  - `scripts/calibrate_temperature.py`: Script interactivo de calibración de 3 puntos
  - Espera forzada de 3 minutos por punto para estabilización térmica
  - Cálculo automático de slope, offset y R² por regresión lineal
  - Verificación de errores en cada punto de calibración
  - Generación automática de archivo `temperature_calibration_result.txt`

- **Opciones de configuración en `main/Kconfig.projbuild`:**
  - `CONFIG_BIOFLOC_TEMP_SLOPE`: Pendiente × 1,000,000 (rango 0.5-1.5, default: 1086092)
  - `CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES`: Offset × 1000 (rango ±5°C, default: -423)
  - Documentación integrada en menú de configuración

### Cambiado
- **main/sensors.c v2.3.0:**
  - Añadido struct `temp_cal` con campos slope, offset, enabled
  - Modificado `sensors_init()` para cargar calibración desde Kconfig
  - Modificado `sensors_read_temperature()` para aplicar calibración si está habilitada
  - Logs mejorados con información de calibración al inicio

- **sdkconfig.defaults:**
  - Añadida sección de calibración de temperatura con valores óptimos
  - Documentación de ecuación de calibración en comentarios
  - Fecha de calibración registrada (2026-01-29)

### Corregido
- **Configuración WiFi duplicada:**
  - El componente micro-ROS usa `CONFIG_ESP_WIFI_SSID/PASSWORD` (no `CONFIG_BIOFLOC_WIFI_*`)
  - Corregidas credenciales en líneas 457-458 de sdkconfig
  - Conexión WiFi ahora funciona correctamente

### Rendimiento
- **Precisión de temperatura mejorada:**
  - De ±1.5°C (sin calibración) a ≤0.03°C (con calibración)
  - R² = 0.999999 (ajuste prácticamente perfecto)
  - Verificado en rango 0-44°C con termómetro TP101
  - Error máximo observado: 0.03°C en 3 puntos de calibración

### Detalles técnicos
- **Calibración realizada:** 2026-01-29
  - Punto 1: ESP32=0.47°C, TP101=0.10°C → error -0.01°C (post-cal)
  - Punto 2: ESP32=21.96°C, TP101=23.40°C → error +0.03°C (post-cal)
  - Punto 3: ESP32=40.52°C, TP101=43.60°C → error -0.01°C (post-cal)
  - Ecuación: `T_calibrada = 1.086092 × T_raw - 0.423`

## [3.0.0] - 2026-01-29

### Resumen
Arquitectura de 2 colecciones con indexación optimizada para escalabilidad y consultas eficientes. Migración desde estructura de 1 colección a arquitectura IoT profesional.

### Agregado
- **Arquitectura de 2 colecciones:**
  - `telemetria`: Lecturas de sensores (series temporales) con índices compuestos
  - `devices`: Metadatos, estado e historial de conexión de dispositivos
  - Auto-registro de nuevos dispositivos en primera lectura
  
- **Índices MongoDB:**
  - `telemetria.idx_device_timestamp`: (device_id, timestamp DESC) para consultas por dispositivo
  - `telemetria.idx_timestamp`: (timestamp DESC) para lecturas recientes de todos los dispositivos
  - Rendimiento de consultas: tiempo de respuesta promedio <5ms

- **Nuevos scripts:**
  - `migrate_to_devices_collection.py`: Script de migración con cálculo automático de estadísticas
  - `verify_migration.py`: Verificación exhaustiva de migración

- **Metadatos de dispositivos:**
  - Historial de conexión (primera, última, total de lecturas)
  - Estado del dispositivo (activo, inactivo, pendiente)
  - Flag de auto-registro
  - Almacenamiento de parámetros de calibración
  - Configuración de umbrales por dispositivo
  - Seguimiento de versión de firmware

### Cambiado
- **sensor_db_bridge.py v3.0:**
  - Escritura dual: Insert en telemetria + Update en devices
  - Auto-creación de documento de dispositivo en primera lectura
  - Actualización de timestamp de última conexión y contador de lecturas
  - Eliminados caracteres emoji de logs (salida profesional)
  - **IMPORTANTE:** Sin cambios en firmware ESP32, solo backend Python
  
- **.env.example:**
  - Agregada variable `MONGODB_COLLECTION_DEVICES=devices`
  - Comentarios de documentación actualizados

- **README.md:**
  - Eliminados caracteres emoji (documentación profesional)
  - Documentada arquitectura de 2 colecciones
  - Agregadas instrucciones de migración
  - Actualizados ejemplos de formato de documentos
  - Agregada explicación de beneficios de indexación

### Detalles Técnicos
- **Beneficios de indexación:**
  - Consultas por dispositivo: O(log n) en lugar de O(n) escaneo completo
  - Consulta con 17,000+ documentos: 200ms → 5ms (40x más rápido)
  - Uso automático de índices por el planificador de consultas de MongoDB
  
- **Estructura de documento de dispositivo:**
  ```json
  {
    "_id": "device_id",
    "alias": "ESP32-xxxx",
    "location": "tanque_01",
    "estado": "activo",
    "auto_registrado": true,
    "conexion": {
      "primera": "ISO timestamp",
      "ultima": "ISO timestamp",
      "total_lecturas": 17210
    }
  }
  ```

### Guía de Migración
1. Respaldar datos existentes (recomendado)
2. Ejecutar: `python3 scripts/migrate_to_devices_collection.py`
3. Verificar: `python3 scripts/verify_migration.py`
4. Actualizar .env con `MONGODB_COLLECTION_DEVICES=devices`
5. Reiniciar bridge: `python3 scripts/sensor_db_bridge.py`
6. **NO es necesario flashear el ESP32** - cambios solo en backend Python

---

## [2.3.0] - 2026-01-22

### 🎯 Resumen
Corrección del bridge ROS 2 → MongoDB y mejora de documentación con guía paso a paso.

### ✨ Added
- **Guía paso a paso (`GUIA_PASO_A_PASO.md`):**
  - Instrucciones detalladas para ejecutar el proyecto completo
  - Comandos copiables para cada terminal
  - Sección de solución de problemas rápida
  - Script de verificación del sistema

- **Nuevos scripts:**
  - `scripts/monitor_sensores.py`: Monitor mejorado con estadísticas
  - `scripts/check_mongodb.py`: Verificador de conexión MongoDB

### 🔧 Changed
- **Bridge MongoDB (`sensor_db_bridge.py`):**
  - Corregido: Ahora usa `String` (JSON) en vez de `Float32MultiArray`
  - Preserva estructura JSON original del ESP32
  - Incluye voltajes en el documento guardado
  
- **Documentación:**
  - README.md actualizado con comandos correctos (sin Docker)
  - Estructura del proyecto actualizada
  - Información del autor: @Marton1123

### 🗑️ Removed
- `sensor_db_bridge_v2.py` (duplicado)
- Referencias a Docker (no se usa en este proyecto)

### 🐛 Fixed
- Bridge no recibía datos (usaba tipo de mensaje incorrecto)
- Import duplicado de `json` en sensor_db_bridge.py

---

## [2.2.0] - 2026-01-21

### 🎯 Resumen
Versión de calibración profesional del sensor de pH. Se logró reducir el error de ±7.71 pH a ±0.03 pH mediante calibración de 3 puntos con soluciones buffer profesionales.

### ✨ Added
- **Sistema de calibración de 3 puntos:**
  - `scripts/calibrate_ph.py`: Calibración automatizada con buffers pH 4.01, 6.86, 9.18
  - Timeout extendido: 420 segundos (7 minutos) por buffer
  - Criterio de estabilidad: σ < 0.002V durante 50 segundos
  - Tiempo mínimo de espera: 180 segundos (3 minutos) antes de verificar estabilidad
  - Cálculo de R² para validar calidad del ajuste lineal
  
- **Herramientas de diagnóstico:**
  - `scripts/monitor_temperature.py`: Monitor en tiempo real
  - Documentación en `docs/`
  
- **Documentación:**
  - `docs/CALIBRATION.md`: Guía completa de calibración
  - `docs/TROUBLESHOOTING.md`: Solución de problemas
  - `docs/SECURITY.md`: Guías de seguridad
  - `calibration_3point_result.txt`: Archivo de resultados de calibración

### 🔧 Changed
- **Voltage divider factor:**
  - Valor inicial: 3.0 (incorrecto, basado en cálculo teórico)
  - **Valor final: 1.474** (calibrado con multímetro físico)
  
- **Calibración del sensor:**
  - **Calibración 3 puntos: slope=2.559823, offset=0.469193**
    - R² = 0.9997 (ajuste casi perfecto)
    - Errores: [0.021, 0.049, 0.028] pH en los 3 puntos
    - Verificación en agua pH 7.06: lectura 7.09 (error 0.03 pH)
  
- **Timezone configuration:**
  - Valor anterior: `CLT3CLST,M10.2.0/00:00,M3.2.0/00:00` (con horario de verano)
  - **Valor actual: `CLT3`** (GMT-3 fijo, sin horario de verano)
  - Razón: Simplificación y evitar confusión con cambios de horario
  
- **MongoDB bridge (`sensor_db_bridge.py`):**
  - Eliminado campo `_received_at` (redundante con timestamp del dispositivo)
  - Campos actuales: `timestamp`, `ph`, `temperature`, `device_id`, `location`, `_ros_topic`
  - Timestamp usa formato ISO 8601 con timezone: `2026-01-21T17:15:42-0300`

### 🐛 Fixed
- **Error crítico de lectura de pH:**
  - Problema: Lectura 14.8 pH cuando el agua real era 7.06 pH (error 7.74 pH)
  - Causa raíz: Voltage divider factor incorrecto (3.0 vs 1.474 real)
  - Solución: Calibración en 2 etapas:
    1. Corrección del divisor de voltaje con multímetro
    2. Calibración de 3 puntos con soluciones buffer
  - **Resultado:** Error reducido a 0.03 pH (mejora 258x)
  
- **Timestamps incorrectos:**
  - Problema: ESP32 marcaba 18:04 cuando hora real era 17:04 (1 hora adelantado)
  - Causa: Configuración de timezone con horario de verano mal interpretada
  - Solución: Cambio a timezone fijo `CLT3` (GMT-3)
  - **Resultado:** Timestamps correctos alineados con hora del sistema
  
- **Sensor no estabilizaba en calibración:**
  - Problema: Timeout de 60 segundos insuficiente, lecturas inestables
  - Causa: Sensor CWT-BL requiere 3-5 minutos de estabilización
  - Solución: 
    - Timeout aumentado a 420 segundos (7 minutos)
    - Espera mínima de 180 segundos antes de verificar estabilidad
    - Criterio de estabilidad más estricto (50 segundos estable)
  - **Resultado:** Calibración exitosa con R² = 0.9997

### 📊 Performance
- **Precisión de pH:**
  - Antes: ±7.71 pH (completamente fuera de rango)
  - Después corrección divisor: ±0.27 pH
  - **Después calibración 3 puntos: ±0.03 pH** (mejora 9x desde corrección)
  
- **Calidad de calibración:**
  - R² = 0.9997 (99.97% de varianza explicada)
  - Error máximo en puntos de calibración: 0.049 pH
  - Error RMS: ~0.033 pH
  
- **MongoDB guardado:**
  - Tasa de guardado: ~250 registros/hora
  - Success rate: 100% (sin pérdida de datos)
  - Latencia típica: <50ms (LAN)

### 🧪 Testing
- **Calibración verificada con:**
  - Soluciones buffer profesionales: pH 4.01, 6.86, 9.18
  - Sensor manual calibrado chino (verificación independiente)
  - Agua de pH conocido: 7.06 medido → 7.09 leído
  
- **Voltage divider verificado con:**
  - Multímetro digital: 1.71V en GPIO (pH 7.06)
  - Cálculo inverso: 1.474 = (7.06 / 2.8) / 1.71
  
- **Timezone verificado con:**
  - Hora del sistema: 17:04 local
  - Timestamp ESP32: 17:04 -0300 ✓
  - Timestamp MongoDB: 17:04 -0300 ✓

### 📝 Documentation
- README.md ampliado con:
  - Tabla de estado de calibración con todos los parámetros
  - Proceso de calibración paso a paso
  - Troubleshooting específico para sensores de pH
  - Especificaciones técnicas completas
  - Changelog integrado
  
- docs/CALIBRATION.md creado con:
  - Teoría de operación del sensor CWT-BL
  - Guía detallada de calibración de 3 puntos
  - Mejores prácticas y tips
  - Troubleshooting avanzado

### 🔒 Security
- Archivo `.env` en `.gitignore` (credenciales MongoDB no commiteadas)
- MongoDB URI con autenticación en variable de entorno

### 🗑️ Deprecated
- Ninguno

### ❌ Removed
- Campo `_received_at` de documentos MongoDB (redundante)
- Script `calibrate_ph.py` original (reemplazado por versión de 3 puntos)

### 🔐 Security
- Ningún cambio

---

## [2.1.0] - 2026-01-20

### Added
- MongoDB bridge (`sensor_db_bridge.py`)
- Publicación de datos estructurados en JSON
- Integración con MongoDB Atlas
- Configuración por variables de entorno (.env)

### Changed
- Formato de mensaje de sensor a estructura personalizada
- Sistema de logging mejorado

---

## [2.0.0] - 2026-01-19

### Added
- Firmware base con micro-ROS Jazzy
- Soporte para ESP-IDF v5.3.4
- Lectura básica de sensores de pH y temperatura
- Publicación de datos por ROS 2 topics
- Configuración WiFi por Kconfig
- Reconexión automática WiFi y Agent
- Sistema de ping para verificar conectividad

### Changed
- Migración de micro-ROS Humble a Jazzy
- Actualización de componentes ESP-IDF

---

## [1.0.0] - 2026-01-15

### Added
- Versión inicial del firmware
- Soporte básico para micro-ROS Humble
- Lectura de sensores analógicos ADC

---

**Leyenda:**
- ✨ Added: Nuevas características
- 🔧 Changed: Cambios en funcionalidad existente
- 🐛 Fixed: Correcciones de bugs
- 📊 Performance: Mejoras de rendimiento
- 🗑️ Deprecated: Características obsoletas
- ❌ Removed: Características eliminadas
- 🔒 Security: Cambios de seguridad
