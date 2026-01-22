# 🔒 Guía de Seguridad — Biofloc Firmware ROS

> **Clasificación:** Uso Interno  
> **Versión:** 1.0.0  
> **Última Actualización:** 2026-01-22  
> **Estándares:** OWASP IoT Top 10 | IEC 62443 | NIST Cybersecurity Framework

---

## 📋 Tabla de Contenidos

1. [Visión General](#-visión-general)
2. [Gestión de Credenciales](#-gestión-de-credenciales)
3. [Seguridad de Red](#-seguridad-de-red)
4. [Seguridad del Firmware](#-seguridad-del-firmware)
5. [Protección de Datos](#-protección-de-datos)
6. [Desarrollo Seguro](#-desarrollo-seguro)
7. [Respuesta a Incidentes](#-respuesta-a-incidentes)
8. [Checklist de Cumplimiento](#-checklist-de-cumplimiento)

---

## 🎯 Visión General

### Principios de Seguridad (Tríada CIA)

| Principio | Implementación |
|-----------|----------------|
| **Confidencialidad** | Credenciales cifradas, comunicación segura |
| **Integridad** | Validación de entrada, checksums, firmware firmado |
| **Disponibilidad** | Watchdog timer, auto-recuperación, redundancia |

### Modelo de Amenazas

```
┌─────────────────────────────────────────────────────────────────┐
│                    PANORAMA DE AMENAZAS                          │
├─────────────────────────────────────────────────────────────────┤
│  Amenazas Externas         │  Amenazas Internas                  │
│  ├─ Sniffing de red        │  ├─ Exposición de credenciales      │
│  ├─ Man-in-the-middle      │  ├─ Configuración insegura          │
│  ├─ Denegación de servicio │  ├─ Vulnerabilidades sin parchar    │
│  └─ Acceso no autorizado   │  └─ Manipulación física del disp.   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔑 Gestión de Credenciales

### ⚠️ CRÍTICO: Nunca Commitear Secretos

**Archivos que DEBEN estar en `.gitignore`:**

```gitignore
# Credenciales - NUNCA COMMITEAR
scripts/.env
*.env
*.env.*
config/secrets.yaml
**/*_credentials*
**/*_secret*

# Credenciales WiFi/Red en sdkconfig
sdkconfig.old

# Claves SSH
*.pem
*.key
id_rsa*
```

### Variables de Entorno

**✅ CORRECTO: Usar archivos `.env` (no commiteados)**

```bash
# scripts/.env (solo local, en .gitignore)
MONGODB_URI=mongodb+srv://usuario:PASSWORD@cluster.mongodb.net/
MONGODB_DB=SistemasLab
MONGODB_COLLECTION=telemetria
```

**❌ INCORRECTO: Credenciales hardcodeadas**

```python
# NUNCA HACER ESTO
MONGODB_URI = "mongodb+srv://admin:SuperSecreto123@cluster.mongodb.net/"
```

### Credenciales WiFi

**Opción 1: Usar menuconfig (almacenado en sdkconfig)**
```bash
idf.py menuconfig
# → Biofloc Configuration → WiFi Configuration
```

**Opción 2: Usar variables de entorno (CI/CD)**
```bash
export WIFI_SSID="MiRed"
export WIFI_PASSWORD="MiPassword"
idf.py build
```

### Rotación de Credenciales

| Tipo de Credencial | Frecuencia de Rotación | Procedimiento |
|--------------------|------------------------|---------------|
| Password WiFi | Anual o en brecha | Actualizar menuconfig, reflashear |
| Password MongoDB | Trimestral | Actualizar .env, reiniciar bridge |
| API Keys | Semestral | Actualizar .env, reiniciar servicios |

---

## 🌐 Seguridad de Red

### Seguridad WiFi

**Requisitos:**
- ✅ WPA2-PSK mínimo (WPA3 preferido)
- ✅ Password fuerte (12+ caracteres, mayúsculas, números, símbolos)
- ❌ Nunca usar WEP o redes abiertas
- ❌ Evitar WPS (vulnerable a fuerza bruta)

**Configuración ESP32:**
```c
// En sdkconfig.defaults - forzar WPA2 mínimo
CONFIG_ESP_WIFI_AUTH_WPA2_PSK=y
```

### Reglas de Firewall

**Puertos Requeridos:**

| Puerto | Protocolo | Dirección | Propósito |
|--------|-----------|-----------|-----------|
| 8888 | UDP | Entrada | micro-ROS Agent |
| 123 | UDP | Salida | Sincronización NTP |
| 27017 | TCP | Salida | MongoDB Atlas |
| 443 | TCP | Salida | HTTPS (MongoDB TLS) |

**Ejemplo de Firewall Linux:**
```bash
# Permitir solo puertos requeridos
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow 8888/udp comment 'micro-ROS Agent'
sudo ufw enable
```

### Segmentación de Red

**Arquitectura Recomendada:**
```
┌─────────────────────────────────────────────────────────────┐
│                    TOPOLOGÍA DE RED                          │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐      ┌──────────┐      ┌──────────────────┐   │
│  │  ESP32   │      │  Agent   │      │  MongoDB Atlas   │   │
│  │ (Sensor) │─WiFi─│  (Host)  │─TLS──│    (Cloud)       │   │
│  └──────────┘      └──────────┘      └──────────────────┘   │
│       │                  │                    │              │
│    VLAN IoT          VLAN Servidor      Internet (TLS)      │
│  192.168.10.0/24    192.168.1.0/24                          │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Seguridad de Transporte

**Implementación Actual:**
- ESP32 ↔ Agent: UDP (sin cifrar, solo LAN)
- Agent ↔ MongoDB: TLS 1.2+ (cifrado)

**Mejora Futura:**
- Considerar características de seguridad de micro-ROS cuando estén disponibles
- Implementar certificate pinning para MongoDB

---

## 🔧 Seguridad del Firmware

### Secure Boot (Recomendado para Producción)

```bash
# Habilitar secure boot en menuconfig
idf.py menuconfig
# → Security features → Enable hardware Secure Boot in bootloader
```

### Cifrado de Flash

```bash
# Habilitar cifrado de flash
idf.py menuconfig
# → Security features → Enable flash encryption on boot
```

### Validación de Entrada

**✅ CORRECTO: Validar todas las entradas**

```c
esp_err_t sensors_calibrate_ph_manual(float slope, float offset)
{
    // Validar entradas antes de usar
    if (!s_ctx.initialized) {
        ESP_LOGE(TAG, "Sensores no inicializados");
        return ESP_ERR_INVALID_STATE;
    }

    if (slope <= 0.0f || slope > 10.0f) {
        ESP_LOGE(TAG, "Slope inválido: %.4f (debe ser 0 < slope <= 10)", slope);
        return ESP_ERR_INVALID_ARG;
    }

    if (offset < -10.0f || offset > 10.0f) {
        ESP_LOGE(TAG, "Offset inválido: %.4f (debe ser -10 <= offset <= 10)", offset);
        return ESP_ERR_INVALID_ARG;
    }

    // ... proceder con entradas válidas
}
```

**❌ INCORRECTO: Confiar en todas las entradas**

```c
// NUNCA HACER ESTO - sin validación
void set_calibration(float slope, float offset) {
    g_slope = slope;  // ¡Podría ser cualquier valor!
    g_offset = offset;
}
```

### Prevención de Buffer Overflow

**✅ CORRECTO: Usar funciones seguras**

```c
// Siempre usar funciones con límite de tamaño
snprintf(buffer, sizeof(buffer), "pH: %.2f", ph_value);
strncpy(dest, src, sizeof(dest) - 1);
dest[sizeof(dest) - 1] = '\0';  // Asegurar terminación null
```

**❌ INCORRECTO: Operaciones sin límites**

```c
// NUNCA HACER ESTO
sprintf(buffer, "pH: %.2f", ph_value);  // ¡Sin límite de tamaño!
strcpy(dest, src);  // ¡Sin límite de tamaño!
```

### Watchdog Timer

```c
// Configurado en sdkconfig.defaults
CONFIG_ESP_TASK_WDT_EN=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=10

// Auto-reinicio en bloqueo
CONFIG_ESP_TASK_WDT_PANIC=y  // Opcional: panic en timeout de WDT
```

---

## 📊 Protección de Datos

### Clasificación de Datos

| Tipo de Dato | Clasificación | Protección |
|--------------|---------------|------------|
| Lecturas de sensores | Interno | Validación de integridad |
| Parámetros de calibración | Interno | Backup, control de versiones |
| Credenciales WiFi | Confidencial | Cifrado, control de acceso |
| URI de MongoDB | Confidencial | Archivo .env, no commitear |
| MAC/ID del dispositivo | Interno | No exponer externamente |

### Integridad de Datos

**Validación JSON:**
```python
# En sensor_db_bridge.py
import json

def validar_datos_sensor(data: dict) -> bool:
    """Validar datos del sensor antes de insertar en BD."""
    campos_requeridos = ['device_id', 'timestamp', 'sensors']
    
    for campo in campos_requeridos:
        if campo not in data:
            return False
    
    # Validar rangos de sensores
    ph = data.get('sensors', {}).get('ph', {}).get('value')
    if ph is not None and not (0.0 <= ph <= 14.0):
        return False
    
    temp = data.get('sensors', {}).get('temperature', {}).get('value')
    if temp is not None and not (-20.0 <= temp <= 80.0):
        return False
    
    return True
```

### Seguridad en Logs

**✅ CORRECTO: Sanitizar logs**

```c
ESP_LOGI(TAG, "Conectado a WiFi SSID: %s", wifi_config.sta.ssid);
// No loguear: ESP_LOGI(TAG, "Password: %s", wifi_config.sta.password);
```

**❌ INCORRECTO: Loguear datos sensibles**

```c
// NUNCA HACER ESTO
ESP_LOGI(TAG, "MongoDB URI: %s", mongodb_uri);  // ¡Expone password!
```

---

## 👨‍💻 Desarrollo Seguro

### Checklist de Code Review

- [ ] Sin credenciales hardcodeadas
- [ ] Todas las entradas validadas
- [ ] Tamaños de buffer verificados
- [ ] Manejo de errores completo
- [ ] Logs sanitizados
- [ ] Memoria liberada correctamente
- [ ] Sin funciones deprecadas

### Principios SOLID Aplicados

| Principio | Aplicación |
|-----------|------------|
| **S**ingle Responsibility | `sensors.c` solo maneja lógica de sensores |
| **O**pen/Closed | Calibración extensible via punteros a función |
| **L**iskov Substitution | `sensor_reading_t` funciona para cualquier sensor |
| **I**nterface Segregation | Interfaces separadas init/read/calibrate |
| **D**ependency Inversion | Sensores dependen de interfaz abstracta de ADC |

### Prácticas de Clean Code

```c
// ✅ BUENO: Código claro y auto-documentado
esp_err_t sensors_read_ph(sensor_reading_t *reading)
{
    // Validar argumentos (fail fast)
    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Verificar precondiciones
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Responsabilidad única: leer y convertir
    int raw_adc, voltage_mv;
    esp_err_t ret = read_adc_averaged(PH_CHANNEL, &raw_adc, &voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }

    // Transformación de datos clara
    float v_sensor = (voltage_mv / 1000.0f) * PH_DIVIDER_FACTOR;
    float ph_value = apply_calibration(v_sensor);

    // Poblar resultado
    reading->value = ph_value;
    reading->voltage = v_sensor;
    reading->valid = is_valid_ph(ph_value);

    return ESP_OK;
}
```

### Gestión de Dependencias

```bash
# Bloquear versiones de componentes en CMakeLists.txt
set(MICRO_ROS_VERSION "jazzy")

# Usar versión específica de ESP-IDF
# En CI/CD: source ~/esp/v5.3.4/esp-idf/export.sh
```

---

## 🚨 Respuesta a Incidentes

### Detección

**Indicadores de Compromiso:**
- Reinicios inesperados del dispositivo
- Tráfico de red anormal
- Lecturas de sensores inválidas
- Intentos de autenticación fallidos
- Cambios de configuración no autorizados

### Procedimiento de Respuesta

1. **Aislar:** Desconectar dispositivo afectado de la red
2. **Preservar:** Capturar logs y configuración
3. **Analizar:** Identificar causa raíz
4. **Contener:** Bloquear vector de ataque
5. **Recuperar:** Restaurar desde estado conocido bueno
6. **Documentar:** Registrar detalles del incidente

### Comandos de Recuperación

```bash
# 1. Reset de fábrica del firmware
idf.py erase-flash
idf.py flash

# 2. Rotar credenciales
idf.py menuconfig  # Cambiar password WiFi
# Actualizar .env con nuevo password de MongoDB

# 3. Verificar integridad
idf.py build  # Recompilar desde fuente
md5sum build/biofloc_firmware_ros.bin  # Comparar con conocido bueno
```

---

## ✅ Checklist de Cumplimiento

### Revisión de Seguridad Pre-Despliegue

- [ ] Todas las credenciales en archivos .env (no commiteados)
- [ ] .gitignore incluye archivos sensibles
- [ ] WiFi usa WPA2-PSK mínimo
- [ ] Firewall configurado correctamente
- [ ] Validación de entrada en todas las funciones públicas
- [ ] Protección contra buffer overflow habilitada
- [ ] Watchdog timer configurado
- [ ] Secure boot habilitado (producción)
- [ ] Cifrado de flash habilitado (producción)
- [ ] Logs no exponen secretos
- [ ] Dependencias bloqueadas a versiones específicas

### Auditoría de Seguridad Periódica

| Item | Frecuencia | Última Completada |
|------|------------|-------------------|
| Rotación de credenciales | Trimestral | — |
| Actualización de dependencias | Mensual | — |
| Revisión de seguridad de código | Por release | — |
| Pruebas de penetración | Anual | — |
| Verificación de backups | Mensual | — |

---

## 📚 Referencias

- [OWASP IoT Top 10](https://owasp.org/www-project-internet-of-things/)
- [ESP-IDF Security Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/index.html)
- [NIST Cybersecurity Framework](https://www.nist.gov/cyberframework)
- [IEC 62443 Industrial Security](https://www.iec.ch/cyber-security)

---

**Versión del Documento:** 1.0.0  
**Clasificación:** Uso Interno  
**Fecha de Revisión:** 2026-04-22
