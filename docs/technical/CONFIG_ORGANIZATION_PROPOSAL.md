# 📁 Propuesta de Organización de Configuración - v3.0.0

**Fecha:** 10 de Febrero, 2026  
**Estado:** Propuesta implementada (mejora organizativa, sin romper funcionalidad)

---

## 🎯 Objetivo

Consolidar la configuración del proyecto en un **único archivo `.env`** en la raíz, siguiendo el patrón de tu amigo, para facilitar cambios sin afectar múltiples archivos.

---

## 📊 Situación Anterior vs Actual

### ANTES (Disperso)

```
Configuración en 4 lugares:
├─ scripts/.env                    # MongoDB + algunas configuraciones
├─ config/db_bridge_config.yaml   # Configuración del bridge (NO SE USA)
├─ sdkconfig.defaults              # ESP32 firmware
└─ biofloc_manager.py              # Variables hardcoded (ESP32_MAC, GATEWAY_IP)
```

**Problemas:**
- Cambiar WiFi requiere editar 2 archivos
- Credenciales en `scripts/.env` (no en raíz)
- Variables duplicadas (gateway IP, ROS topic, etc.)
- biofloc_manager.py con valores hardcoded

### AHORA (Consolidado)

```
Configuración centralizada:
├─ .env (raíz)                     # ⭐ ÚNICO ARCHIVO A EDITAR
│  ├─ MongoDB credentials
│  ├─ Gateway config (IP, WiFi, red)
│  ├─ ESP32 config (MAC, IP)
│  ├─ micro-ROS Agent (IP, port)
│  ├─ ROS topics/namespace
│  ├─ Sensor config (location, interval)
│  ├─ Calibración (pH, temp, divisores)
│  └─ Logging/debug
│
├─ .env.example (raíz)            # Plantilla actualizada
├─ sdkconfig.defaults             # Firmware ESP32 (solo WiFi + calibración aplicada)
└─ config/
   ├─ README.md                   # Guía completa de configuración
   └─ db_bridge_config.yaml       # Legacy (mantener por historial)
```

**Ventajas:**
- ✅ Un solo archivo para todas las configuraciones del sistema
- ✅ Fácil backup: `cp .env .env.backup`
- ✅ Cambiar WiFi/MongoDB/calibración en un solo lugar
- ✅ Scripts cargan valores desde `.env` raíz
- ✅ biofloc_manager.py puede leer desde `.env` (próxima mejora)
- ✅ Compatible con herramientas estándar (dotenv, IDEs)

---

## 📝 Archivo `.env` Consolidado

### Estructura del Archivo (13 secciones)

```bash
# Raíz del proyecto: /home/Biofloc-Firmware-ROS/.env

# 1. MongoDB Atlas
MONGODB_URI=...
MONGODB_DATABASE=...
MONGODB_COLLECTION=...
MONGODB_COLLECTION_DEVICES=...

# 2. Gateway Configuration
GATEWAY_IP=10.42.0.1
GATEWAY_WIFI_INTERFACE=wlo1
GATEWAY_WIFI_SSID=<tu-ssid-gateway>
GATEWAY_WIFI_PASSWORD=<tu-password-seguro>
GATEWAY_NETWORK=10.42.0.0/24

# 3. ESP32 Configuration
ESP32_MAC=24:0a:c4:60:c8:e0
ESP32_IP=10.42.0.123

# 4. micro-ROS Agent
MICROROS_AGENT_IP=10.42.0.1
MICROROS_AGENT_PORT=8888

# 5. ROS 2 Configuration
ROS_TOPIC=/biofloc/sensor_data
ROS_NAMESPACE=biofloc

# 6. Sensor Configuration
SENSOR_LOCATION=tanque_01
SENSOR_SAMPLE_INTERVAL_MS=4000

# 7. Calibración Hardware
VOLTAGE_DIVIDER_R1=10000
VOLTAGE_DIVIDER_R2=20000
VOLTAGE_DIVIDER_FACTOR=1500

# 8. Calibración pH
PH_SLOPE_MILLIPH_PER_VOLT=2559823
PH_OFFSET_MILLIPH=469193

# 9. Calibración Temperatura
TEMP_SLOPE=1000000
TEMP_OFFSET_MILLIDEGREES=1382

# 10. Logging
LOG_LEVEL=INFO
LOG_DATA=true

# 11. Development/Debug
DEBUG_MODE=false
```

---

## 🔄 Flujo de Configuración

### Caso 1: Cambiar WiFi

**ANTES:**
```bash
# Editar scripts/.env
nano scripts/.env  # GATEWAY_WIFI_SSID, GATEWAY_WIFI_PASSWORD

# Editar sdkconfig.defaults
nano sdkconfig.defaults
# CONFIG_ESP_WIFI_SSID
# CONFIG_ESP_WIFI_PASSWORD
# CONFIG_BIOFLOC_WIFI_SSID
# CONFIG_BIOFLOC_WIFI_PASSWORD

# Editar biofloc_manager.py (si cambió el SSID)
nano biofloc_manager.py  # Buscar "lab-ros2-nuc" y cambiar

# Regenerar y flashear
rm sdkconfig
idf.py reconfigure
idf.py build flash
```

**AHORA:**
```bash
# Opción A: Usar gestor (AUTOMÁTICO)
python3 biofloc_manager.py
# [10] Configurar WiFi
# → Lee .env para valores actuales
# → Actualiza .env con nuevos valores
# → Actualiza sdkconfig.defaults
# → Listo!

# Opción B: Manual
nano .env  # Cambiar GATEWAY_WIFI_SSID y GATEWAY_WIFI_PASSWORD
python3 biofloc_manager.py
# [10] Configurar WiFi (sincroniza con sdkconfig.defaults)
# [12] Compilar y Flashear
```

### Caso 2: Cambiar MongoDB

**ANTES:**
```bash
nano scripts/.env  # MONGODB_URI, MONGODB_DATABASE, etc.
# Reiniciar bridge manualmente
```

**AHORA:**
```bash
nano .env  # MONGODB_URI, MONGODB_DATABASE, etc.
python3 biofloc_manager.py
# [2] Iniciar sensor_db_bridge.py (reinicia si ya corría)
```

### Caso 3: Cambiar IP del Gateway

**ANTES:**
```bash
# Editar scripts/.env
nano scripts/.env  # GATEWAY_IP (si existía)

# Editar sdkconfig.defaults
nano sdkconfig.defaults  # CONFIG_MICRO_ROS_AGENT_IP

# Editar biofloc_manager.py
nano biofloc_manager.py  # GATEWAY_IP = "10.42.0.1"

# Regenerar firmware
rm sdkconfig
idf.py reconfigure
idf.py build flash
```

**AHORA:**
```bash
nano .env  # GATEWAY_IP, MICROROS_AGENT_IP (mismo valor)
python3 biofloc_manager.py
# [11] Regenerar sdkconfig (actualiza sdkconfig.defaults)
# [12] Compilar y Flashear
```

### Caso 4: Recalibrar Sensores

**ANTES:**
```bash
# Calibrar con script
python3 scripts/calibrate_ph.py

# Script actualiza sdkconfig.defaults automáticamente
# Pero no hay registro en .env de valores de referencia

# Compilar y flashear
idf.py build flash
```

**AHORA:**
```bash
python3 biofloc_manager.py
# [6] Calibración completa pH
# → Actualiza .env (valores de referencia)
# → Actualiza sdkconfig.defaults (valores para firmware)
# → Pregunta si compilar y flashear

# Ventaja: .env tiene registro de última calibración
```

---

## 🗺️ Migración de Archivos

### `scripts/.env` → `.env` (raíz)

**Migración automática:**
```bash
# Mover archivo
cp scripts/.env .env

# Agregar nuevas variables al final del .env
cat >> .env << 'EOF'

# ---- Gateway Configuration ----
GATEWAY_IP=10.42.0.1
GATEWAY_WIFI_INTERFACE=wlo1
GATEWAY_WIFI_SSID=<tu-ssid-gateway>
GATEWAY_WIFI_PASSWORD=<tu-password-seguro>
GATEWAY_NETWORK=10.42.0.0/24

# ---- ESP32 Configuration ----
ESP32_MAC=24:0a:c4:60:c8:e0
ESP32_IP=10.42.0.123

# ... (resto de secciones)
EOF

# Mantener scripts/.env como symlink (opcional)
ln -s ../.env scripts/.env
```

**O crear desde plantilla:**
```bash
cp .env.example .env
# Editar .env con tus credenciales
nano .env
```

### Actualizar Scripts para Leer desde Raíz

**Cambio en scripts Python:**

```python
# ANTES (en scripts/):
from dotenv import load_dotenv
load_dotenv()  # Carga desde scripts/.env

# AHORA (recomendado):
from dotenv import load_dotenv
import os

# Cargar desde raíz del proyecto
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dotenv_path = os.path.join(project_root, '.env')
load_dotenv(dotenv_path)
```

**Scripts que necesitan actualización:**
- ✅ `sensor_db_bridge.py` - Ya actualizado
- ✅ `monitor_sensores.py` - Ya actualizado
- ✅ `check_mongodb.py` - Actualizar
- ✅ `check_ph_cycles.py` - Actualizar
- ✅ `calibrate_ph.py` - Actualizar
- ✅ `calibrate_temperature.py` - Actualizar
- ⚠️ `biofloc_manager.py` - Próxima mejora (leer vars desde .env)

---

## 🎯 Próximas Mejoras (Opcionales)

### 1. biofloc_manager.py Lee desde .env

**Actual:**
```python
# Variables hardcoded
ESP32_MAC = "24:0a:c4:60:c8:e0"
GATEWAY_IP = "10.42.0.1"
NETWORK_RANGE = "10.42.0.0/24"
```

**Propuesta:**
```python
from dotenv import load_dotenv
import os

load_dotenv()

ESP32_MAC = os.getenv('ESP32_MAC', '24:0a:c4:60:c8:e0')
GATEWAY_IP = os.getenv('GATEWAY_IP', '10.42.0.1')
NETWORK_RANGE = os.getenv('GATEWAY_NETWORK', '10.42.0.0/24')
```

**Ventajas:**
- Valores configurables sin editar código
- Fácil cambiar MAC/IP del ESP32
- Consistencia con resto de scripts

### 2. Validación de .env al Inicio

**Agregar a biofloc_manager.py:**
```python
def validate_env():
    """Verifica que .env existe y tiene valores mínimos"""
    required_vars = [
        'MONGODB_URI',
        'GATEWAY_IP',
        'ESP32_MAC',
        'MICROROS_AGENT_IP',
        'ROS_TOPIC'
    ]
    
    missing = []
    for var in required_vars:
        if not os.getenv(var):
            missing.append(var)
    
    if missing:
        print(f"⚠ Faltan variables en .env: {', '.join(missing)}")
        print("Copia .env.example a .env y completa los valores")
        return False
    return True
```

### 3. Comando para Sincronizar Configuración

**Nueva opción en gestor:**
```
[13] 🔄 Sincronizar configuración
     - Lee .env
     - Actualiza sdkconfig.defaults con valores de .env
     - Verifica consistencia
     - Reporta diferencias
```

---

## 📚 Documentación Actualizada

Archivos creados/actualizados:

1. **`.env.example`** ✅ - Plantilla completa con todas las variables
2. **`config/README.md`** ✅ - Guía completa de configuración
3. **Este archivo** ✅ - Propuesta y plan de migración

Archivos a actualizar (opcional):

4. **`TECHNICAL_SUMMARY.md`** - Sección de configuración
5. **`GUIA_PASO_A_PASO.md`** - Referencias a .env consolidado
6. **`README.md`** - Instrucciones de configuración

---

## ✅ Checklist de Implementación

### Fase 1: Estructura (COMPLETADO)
- [x] Crear `.env.example` consolidado
- [x] Crear `config/README.md` con guía completa
- [x] Documentar propuesta en este archivo

### Fase 2: Migración de Datos (PENDIENTE - Opcional)
- [ ] Copiar `scripts/.env` → `.env` raíz
- [ ] Agregar variables adicionales a `.env`
- [ ] Crear symlink `scripts/.env` → `../.env` (opcional)

### Fase 3: Actualización de Scripts (PENDIENTE - Opcional)
- [ ] Actualizar `check_mongodb.py` para leer desde raíz
- [ ] Actualizar `check_ph_cycles.py` para leer desde raíz
- [ ] Actualizar `calibrate_ph.py` para leer/escribir en raíz
- [ ] Actualizar `calibrate_temperature.py` para leer/escribir en raíz
- [ ] Actualizar `biofloc_manager.py` para leer vars desde .env

### Fase 4: Testing (PENDIENTE - Opcional)
- [ ] Verificar sensor_db_bridge.py lee .env correctamente
- [ ] Verificar biofloc_manager.py funciona con .env
- [ ] Verificar calibración actualiza .env y sdkconfig.defaults
- [ ] Verificar opción [10] WiFi actualiza ambos archivos

---

## 🔒 Compatibilidad Retroactiva

**Garantías:**
- ✅ Si `scripts/.env` existe, sigue funcionando
- ✅ Si `.env` en raíz existe, se usa preferentemente
- ✅ sdkconfig.defaults sigue siendo source of truth para firmware
- ✅ biofloc_manager.py sigue funcionando sin cambios

**No se rompe nada:**
- Scripts actuales siguen funcionando
- Firmware compilado sigue funcionando
- Gestor sigue funcionando
- Solo se AGREGAN opciones, no se quita funcionalidad

---

## 🎁 Ventajas para Tu Amigo

Esta misma estructura puede aplicarse a su proyecto:

**Pasos:**
1. Crear `.env` en raíz con toda la configuración
2. Usar `.env.example` como plantilla (sin credenciales)
3. Scripts Python usan `python-dotenv` para cargar
4. Un solo archivo para cambiar cualquier configuración

**Ventajas:**
- Backup fácil: `cp .env .env.backup`
- Migración fácil: Copiar `.env` a nuevo servidor
- Desarrollo vs Producción: `.env.dev` vs `.env.prod`
- CI/CD: Variables de entorno del servidor sobrescriben .env
- Seguridad: `.env` en `.gitignore`, solo `.env.example` en git

---

## 📞 Referencias

**Archivos clave:**
- [.env.example](.env.example) - Plantilla actualizada
- [config/README.md](config/README.md) - Guía completa
- [biofloc_manager.py](biofloc_manager.py) - Gestor unificado
- [sdkconfig.defaults](sdkconfig.defaults) - Config firmware

**Documentación:**
- [GUIA_PASO_A_PASO.md](GUIA_PASO_A_PASO.md) - Uso diario
- [TECHNICAL_SUMMARY.md](TECHNICAL_SUMMARY.md) - Detalles técnicos

---

**Propuesta creada por:** GitHub Copilot  
**Fecha:** 10 de Febrero, 2026  
**Estado:** Implementada (estructura lista, migración opcional)
