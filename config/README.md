# 📁 Directorio de Configuración

Este directorio contiene archivos de configuración auxiliares y de respaldo.

## Estructura de Configuración del Proyecto

### ✅ Configuración Principal (USAR ESTO)

**Archivo:** `/.env` (en la raíz del proyecto)

Este es el **único archivo de configuración** que necesitas editar para:
- Credenciales MongoDB
- Configuración del gateway (IPs, WiFi, red)
- Configuración del ESP32 (MAC, IP)
- Calibración de sensores (pH, temperatura)
- ROS topics y namespaces
- Logging

**Ubicación:** `/home/Biofloc-Firmware-ROS/.env`

**Cómo crear:**
```bash
cd /home/Biofloc-Firmware-ROS
cp .env.example .env
nano .env  # Edita los valores
```

**Archivos que lo usan:**
- `biofloc_manager.py` - Gestor unificado
- `scripts/sensor_db_bridge.py` - Bridge ROS→MongoDB
- `scripts/monitor_sensores.py` - Monitor
- `scripts/check_ph_cycles.py` - Análisis de ciclos
- `scripts/calibrate_ph.py` - Calibración pH
- `scripts/calibrate_temperature.py` - Calibración temperatura
- Todos los scripts de verificación y diagnóstico

---

### ⚙️ Configuración del Firmware ESP32

**Archivo:** `/sdkconfig.defaults`

Configuración del firmware ESP-IDF que se flashea al ESP32.

**Valores clave:**
- WiFi credentials (dual: CONFIG_ESP_WIFI_* y CONFIG_BIOFLOC_WIFI_*)
- IP del Agent (CONFIG_MICRO_ROS_AGENT_IP)
- Calibración de sensores (divisor, slope, offset)

**Gestión:**
- Usa `biofloc_manager.py` opción [10] para configurar WiFi
- Usa `biofloc_manager.py` opciones [6-9] para calibración
- Edita manualmente solo si sabes lo que haces

**⚠️ IMPORTANTE:** Este archivo es el "source of truth" para el firmware. Después de editarlo:
```bash
rm sdkconfig
idf.py reconfigure
idf.py build flash
```

---

### 📄 Archivos en este Directorio

#### `db_bridge_config.yaml` (LEGACY - NO SE USA)

Archivo de configuración legacy del bridge sensor_db_bridge.py.

**Estado:** NO SE USA ACTUALMENTE

**Razón:** El bridge ahora usa `.env` en lugar de este archivo YAML.

**Mantener por:** Referencia histórica, posible uso futuro si se implementa soporte multi-configuración.

---

## 🗺️ Mapa de Configuración

```
Configuración del Sistema
│
├─ GATEWAY Y SCRIPTS
│  └─ .env (raíz del proyecto)
│     ├─ MongoDB credentials
│     ├─ Gateway network config
│     ├─ ESP32 identification
│     ├─ ROS topics
│     └─ Sensor calibration (referencia)
│
├─ FIRMWARE ESP32
│  └─ sdkconfig.defaults
│     ├─ WiFi credentials (DUAL)
│     ├─ Agent IP/port
│     └─ Sensor calibration (aplicada)
│
└─ LEGACY/BACKUP
   └─ config/db_bridge_config.yaml
      └─ (no se usa)
```

---

## 🔄 Flujo de Configuración

### 1. Primera Vez (Setup Inicial)

```bash
# 1. Configurar el sistema (gateway + scripts)
cd /home/Biofloc-Firmware-ROS
cp .env.example .env
nano .env  # Editar valores

# 2. Configurar firmware ESP32
python3 biofloc_manager.py
# Opción [10] - Configurar WiFi
# Opción [11] - Regenerar sdkconfig

# 3. Compilar y flashear
# Opción [12] - Compilar y Flashear
```

### 2. Cambiar WiFi

```bash
# Opción A: Usar gestor (RECOMENDADO)
python3 biofloc_manager.py
# Opción [10] - Actualiza .env Y sdkconfig.defaults automáticamente

# Opción B: Manual
nano .env  # Actualizar GATEWAY_WIFI_SSID y GATEWAY_WIFI_PASSWORD
nano sdkconfig.defaults  # Actualizar ambos CONFIG_ESP_WIFI_* y CONFIG_BIOFLOC_WIFI_*
python3 biofloc_manager.py
# Opción [11] - Regenerar sdkconfig
# Opción [12] - Compilar y Flashear
```

### 3. Cambiar MongoDB

```bash
# Solo editar .env
nano .env  # Actualizar MONGODB_URI, MONGODB_DATABASE, etc.

# Reiniciar bridge
python3 biofloc_manager.py
# Opción [2] - Iniciar sensor_db_bridge.py (reiniciar si estaba corriendo)
```

### 4. Recalibrar Sensores

```bash
# Usar gestor
python3 biofloc_manager.py
# Opción [6] o [7] - Calibración completa (actualiza .env Y sdkconfig.defaults)
# Opción [8] o [9] - Ajuste rápido

# El gestor automáticamente:
# 1. Actualiza .env (para scripts de referencia)
# 2. Actualiza sdkconfig.defaults (para firmware)
# 3. Regenera sdkconfig
# 4. Puede compilar y flashear si lo pides
```

---

## 📝 Jerarquía de Configuración

**Prioridad (de mayor a menor):**

1. **Variables de entorno del sistema** (si están definidas, sobrescriben todo)
2. **`.env` en raíz del proyecto** (principal, usar este)
3. **`sdkconfig`** (generado desde sdkconfig.defaults, no editar directamente)
4. **Valores por defecto en código** (fallback si falta configuración)

---

## 🔒 Seguridad

### ⚠️ Archivos que NO deben subirse a Git:

- ✅ `.env` (ya en .gitignore)
- ✅ `sdkconfig` (ya en .gitignore)
- ✅ `scripts/.env` (ya en .gitignore)

### ✅ Archivos que SÍ se suben a Git:

- `.env.example` (plantilla sin credenciales)
- `sdkconfig.defaults` (valores por defecto seguros)
- `sdkconfig.example` (si existe)
- `config/db_bridge_config.yaml` (legacy, sin credenciales)

---

## 🆘 Troubleshooting

### "El ESP32 no se conecta al WiFi"

**Verificar:**
```bash
# 1. Revisar que .env y sdkconfig.defaults tienen las mismas credenciales
cat .env | grep GATEWAY_WIFI
cat sdkconfig.defaults | grep CONFIG_ESP_WIFI
cat sdkconfig.defaults | grep CONFIG_BIOFLOC_WIFI

# 2. Usar gestor para sincronizar
python3 biofloc_manager.py
# Opción [10] - Configurar WiFi (actualiza ambos)
```

### "El bridge no conecta a MongoDB"

**Verificar:**
```bash
# 1. Revisar credenciales en .env
cat .env | grep MONGODB

# 2. Probar conexión
python3 scripts/check_mongodb.py
```

### "Las calibraciones no se aplican"

**Verificar:**
```bash
# 1. Revisar que sdkconfig.defaults tiene los valores
cat sdkconfig.defaults | grep CONFIG_BIOFLOC_PH
cat sdkconfig.defaults | grep CONFIG_BIOFLOC_TEMP

# 2. Regenerar y flashear
python3 biofloc_manager.py
# Opción [11] - Regenerar sdkconfig
# Opción [12] - Compilar y Flashear
```

---

## 📚 Documentación Relacionada

- [GUIA_PASO_A_PASO.md](../GUIA_PASO_A_PASO.md) - Uso diario del sistema
- [TECHNICAL_SUMMARY.md](../TECHNICAL_SUMMARY.md) - Detalles técnicos de configuración
- [MIGRATION_GUIDE_SECURE_GATEWAY.md](../MIGRATION_GUIDE_SECURE_GATEWAY.md) - Arquitectura de red
- [biofloc_manager.py](../biofloc_manager.py) - Gestor unificado
