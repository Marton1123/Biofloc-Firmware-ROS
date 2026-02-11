# Biofloc Firmware ROS

Sistema completo de telemetría IoT para acuicultura con **micro-ROS Jazzy**, **ESP-IDF v5.3** y **MongoDB Atlas**.

**Versión:** 3.0.0 (Secure Gateway + Manager)  
**Bridge:** 3.1.0 (Server-side Timestamps)  
**Manager:** 1.0.0 (Unified Management Tool)

---

## 🔒 Arquitectura de Gateway Seguro

Sistema IoT con **seguridad mejorada** donde los ESP32 **NO tienen acceso a Internet**:

```
Internet
   |
   | (Ethernet - enp88s0)
   |
[GATEWAY - NUC Ubuntu 24.04]
   | - WiFi Hotspot (wlo1) - 10.42.0.1/24
   | - Firewall iptables (FORWARD DROP)
   | - micro-ROS Agent (UDP 8888)
   | - Scripts Python (Bridge, Monitor)
   | - biofloc_manager.py (Gestor Unificado)
   |
   | (WiFi - sin internet)
   | SSID: <tu-ssid-gateway>
   | Red: 10.42.0.0/24
   |
[ESP32 - 10.42.0.123]
   | - SIN acceso a internet (bloqueado por firewall)
   | - Solo UDP a 10.42.0.1:8888
   | - Timestamps basados en contador (sin NTP)
   | - micro-ROS Publisher
   |
   ↓ Datos enviados al Gateway
   ↓ Gateway agrega timestamps reales
   ↓ Bridge guarda a MongoDB Atlas
```

**Ver:**  
- [MIGRATION_GUIDE_SECURE_GATEWAY.md](MIGRATION_GUIDE_SECURE_GATEWAY.md) - Guía genérica de migración  
- [biofloc_manager.py](biofloc_manager.py) - Gestor unificado del sistema (CLI español, 12 opciones)

---

## Inicio Rápido

**[GUÍA PASO A PASO](GUIA_PASO_A_PASO.md)** - Instrucciones completas de ejecución  
**[MIGRACIÓN A GATEWAY SEGURO](SECURE_GATEWAY_MIGRATION.md)** - Nueva arquitectura de red

---

## Características

### 🔒 Seguridad
- **Gateway IoT Seguro** - ESP32 aislado de internet mediante firewall iptables
- **Doble red** - WiFi para ESP32 (sin internet) + Ethernet para servicios cloud
- **Firewall iptables** - FORWARD DROP bloquea ESP32→Internet, solo permite UDP 8888

### ⏱️ Gestión de Tiempo
- **Sin NTP en ESP32** - Opera sin acceso a servidores de tiempo
- **Timestamps del servidor** - Gateway agrega timestamps reales (UTC)
- **Timestamps de contador** - ESP32 usa contador incremental para correlación

### 🛠️ Gestor Unificado (biofloc_manager.py)
- **CLI en español** - Interfaz completamente traducida
- **12 opciones de menú** - Sistema, verificación, calibración, configuración, firmware
- **Timeouts inteligentes** - 8s para verificación rápida, 20s opcional para tasa
- **Diagnóstico completo** - Estado ESP32, conectividad, DHCP, ARP, ROS topics
- **Calibración integrada** - pH y temperatura (3 puntos + ajustes rápidos)
- **Gestión WiFi** - Actualiza dual credentials (ESP_WIFI + BIOFLOC_WIFI)
- **Build/Flash** - Pipeline completo de firmware desde el gestor

### 📊 Monitoreo y Telemetría
- 📡 **micro-ROS sobre WiFi UDP** - Comunicación ESP32↔Gateway
- 💾 **MongoDB Atlas** - Almacenamiento cloud con manejo robusto de errores
- 🔄 **Reconexión robusta** - Backoff exponencial (hasta 15 intentos)
- 📈 **Publicación cada 4s** - Datos de pH y temperatura en tiempo real

### 🎯 Calibración de Sensores
- **pH: 3 puntos** (4.01, 6.86, 9.18) - Precisión <0.05 pH, R²=0.9997
- **Temperatura: Slope+Offset** - Precisión ≤0.1°C
- **Hardware verificado** - R1=10kΩ, R2=20kΩ (factor 1.5 documentado desde PCB)
- **Scripts automáticos** - Calibración interactiva con actualización de sdkconfig

### 🏗️ Arquitectura
- 📱 Auto-registro de dispositivos por MAC
- 🔗 Formato JSON estructurado
- 🏗️ Arquitectura de 2 colecciones MongoDB
- 📝 Configuración centralizada en sdkconfig.defaults
- 🔌 **[test_led_project](test_led_project/)** - Ejemplo de control por teclado via micro-ROS

## 📋 Requisitos

### Hardware
- **ESP32** (240MHz, Dual Core, WiFi 2.4GHz) - MAC: 24:0a:c4:60:c8:e0
- **Gateway** Intel NUC o PC Linux (Ubuntu 24.04+, WiFi + Ethernet)
- **Sensor** CWT-BL pH/Temperature Transmitter (0-5V output, 24V powered)
- **Voltage Divider** (AMBOS sensores):
  - R1 = 10kΩ (pull-up, conectado a Vin del sensor)
  - R2 = 20kΩ (pull-down a GND)
  - R3 = 470Ω (protección)
  - C1 = 100nF (filtro)
  - **Factor = 1.5** (30k/20k)
- **Soluciones buffer** pH 4.01, 6.86, 9.18 (para calibración de 3 puntos)
- **Termómetro de referencia** (para calibración de temperatura)

### Software
- **ESP-IDF** v5.3.4+ instalado y configurado
- **ROS 2 Jazzy Desktop** (en Gateway)
- **Python 3.12+** con pymongo, python-dotenv, numpy (en Gateway)
- **micro-ROS Agent** (compilado localmente en Gateway)
- **MongoDB Atlas** account con credenciales en `.env`
- **NetworkManager** para gestión de hotspot WiFi
- **iptables** para firewall (incluido en Ubuntu)

## 🚀 Quick Start

### Opción 1: Usando el Gestor Unificado (Recomendado)

```bash
# Ejecutar gestor interactivo (CLI en español)
cd /home/Biofloc-Firmware-ROS
python3 biofloc_manager.py
```

**Menú del gestor (12 opciones):**
1. ▶️ Iniciar micro-ROS Agent
2. ▶️ Iniciar sensor_db_bridge.py
3. 📊 Iniciar monitor_sensores.py
4. ✅ Verificar estado del sistema
5. 🔌 Verificar conectividad ESP32
6. 🧪 Calibración completa pH
7. 🌡️ Calibración completa Temperatura
8. ⚡ Ajuste rápido pH
9. ⚡ Ajuste rápido Temperatura
10. 📶 Configurar WiFi
11. ⚙️ Regenerar sdkconfig
12. 🛠️ Compilar y Flashear

---

### Opción 2: Configuración Manual (Primera Vez)

> **🔒 IMPORTANTE:** Las credenciales WiFi mostradas aquí son ejemplos genéricos.  
> Reemplaza `<tu-ssid-gateway>` y `<tu-password-seguro>` con tus propias credenciales.  
> Consulta el archivo `.env` para la configuración real (no está en Git por seguridad).

### 1. Configurar Gateway WiFi Hotspot

```bash
# Crear hotspot en interfaz WiFi del gateway
nmcli device wifi hotspot \
  ifname wlo1 \
  ssid "<tu-ssid-gateway>" \
  password "<tu-password-seguro>"

# Configurar autoconexión
nmcli connection modify Hotspot connection.autoconnect yes

# Verificar red creada (debe mostrar 10.42.0.1/24)
ip addr show wlo1
```

**Ver:** [MIGRATION_GUIDE_SECURE_GATEWAY.md](MIGRATION_GUIDE_SECURE_GATEWAY.md#11-crear-hotspot-wifi-networkmanager) para más detalles.

### 2. Configurar Firewall iptables

```bash
# Copiar script de firewall a home
cp MIGRATION_GUIDE_SECURE_GATEWAY.md ~/setup_iot_firewall.sh
# (Extraer sección del script del .md)

# Hacer ejecutable y correr
chmod +x ~/setup_iot_firewall.sh
sudo ~/setup_iot_firewall.sh

# Hacer persistente
sudo apt install iptables-persistent
sudo netfilter-persistent save

# Verificar (debe mostrar FORWARD policy DROP)
sudo iptables -L FORWARD -v -n
```

### 3. Configurar el entorno ESP-IDF

```bash
. $HOME/esp/esp-idf/export.sh
# o donde tengas instalado ESP-IDF
```

### 2. Configurar el target

```bash
cd /home/Biofloc-Firmware-ROS
idf.py set-target esp32  # o esp32s3, esp32c3, etc.
```

### 3. Configurar WiFi y Agent (sdkconfig.defaults)

Editar `sdkconfig.defaults` para configurar **ambos sets de credenciales WiFi**:

```bash
cd /home/Biofloc-Firmware-ROS
nano sdkconfig.defaults
```

**Credenciales WiFi (DUAL - micro_ros y aplicación):**
```ini
# Credenciales para componente micro_ros_espidf
CONFIG_ESP_WIFI_SSID="<tu-ssid-gateway>"
CONFIG_ESP_WIFI_PASSWORD="<tu-password-seguro>"
CONFIG_ESP_MAXIMUM_RETRY=15

# Credenciales para aplicación biofloc
CONFIG_BIOFLOC_WIFI_SSID="<tu-ssid-gateway>"
CONFIG_BIOFLOC_WIFI_PASSWORD="<tu-password-seguro>"
```

**Agent IP (Gateway en red interna):**
```ini
CONFIG_MICRO_ROS_AGENT_IP="10.42.0.1"  # IP del gateway
CONFIG_MICRO_ROS_AGENT_PORT=8888
```

**⚠️ IMPORTANTE:** Ambos sets de credenciales WiFi deben ser **idénticos**. Si usas `idf.py menuconfig`, actualiza ambos lugares.

**Alternativa:** Usar opción [10] del gestor para actualizar automáticamente.

### 4. Regenerar sdkconfig y Compilar

```bash
# Cargar entorno ESP-IDF
. $HOME/esp/esp-idf/export.sh

# Regenerar sdkconfig desde defaults
rm sdkconfig
idf.py reconfigure

# Compilar
idf.py build
```

**Alternativa:** Usar opción [12] del gestor para compilar y flashear automáticamente.

### 5. Flashear y Verificar

```bash
# Flashear
idf.py -p /dev/ttyUSB0 flash

# Monitorear (verificar que se conecta a tu SSID y obtiene IP 10.42.0.x)
idf.py -p /dev/ttyUSB0 monitor
```

**Salida esperada:**
```
I (3421) WIFI: WiFi connected to <tu-ssid-gateway>
I (3425) WIFI: Got IP: 10.42.0.123
I (3430) MAIN: Connecting to micro-ROS agent at 10.42.0.1:8888
```

---

## 📊 Uso del Sistema

### Verificar Estado (Gestor - Opción 4)

```bash
python3 biofloc_manager.py
# Seleccionar: [4] Verificar estado del sistema
```

**Verifica:**
- ✅ micro-ROS Agent corriendo
- ✅ sensor_db_bridge.py corriendo
- ✅ Gateway WiFi activo (10.42.0.1)
- ✅ Topic ROS disponible
- ✅ ESP32 publicando mensajes (8s timeout)
- ✅ Tasa de publicación (opcional, 20s)

### Verificar Conectividad ESP32 (Gestor - Opción 5)

```bash
python3 biofloc_manager.py
# Seleccionar: [5] Verificar conectividad ESP32
```

**Verifica:**
- 📶 DHCP lease (MAC 24:0a:c4:60:c8:e0)
- 🔗 ARP table (IP 10.42.0.123)
- 🎯 Ping al ESP32
- 📡 Comunicación ROS (mensajes y tasa)

### Monitoreo Manual

```bash
# Terminal 1: Iniciar Agent
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2: Ver datos
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 topic echo /biofloc/sensor_data

# Terminal 3: Guardar a MongoDB
cd /home/Biofloc-Firmware-ROS/scripts && source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && python3 sensor_db_bridge.py
```

## 🎯 Calibración del Sensor de pH

### Estado Actual del Sistema (v2.2.0)

**✅ Sistema completamente calibrado:**

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Voltage Divider Factor** | 1.474 | R1=20kΩ, R2=10kΩ (calibrado con multímetro) |
| **Calibration Slope** | 2.559823 | Pendiente de la curva pH vs Voltaje |
| **Calibration Offset** | 0.469193 | Desplazamiento vertical |
| **R² (ajuste lineal)** | 0.9997 | Ajuste casi perfecto |
| **Precisión lograda** | ±0.03 pH | Verificado en agua pH 7.06 → 7.09 leído |
| **Rango calibrado** | pH 4-9 | Buffers: 4.01, 6.86, 9.18 |
| **Error máximo** | 0.049 pH | En los 3 puntos de calibración |

### Herramientas de Calibración

```bash
# 1. Monitor de voltaje en tiempo real (comparar con multímetro)
python3 scripts/monitor_voltage.py

# 2. Diagnóstico del divisor de voltaje
python3 scripts/fix_voltage_divider.py

# 3. Calibración de 3 puntos (espera 3-7 min por buffer)
python3 scripts/calibrate_ph_3points.py

# 4. Diagnóstico general del sensor
python3 scripts/diagnose_ph.py
```

### Proceso de Calibración Completo

1. **Verificar divisor de voltaje:**
   - Medir V_GPIO con multímetro mientras el sensor está en agua
   - Comparar con la lectura del software
   - Si difieren >5%, ajustar `CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR` en sdkconfig

2. **Realizar calibración de 3 puntos:**
   - Preparar soluciones buffer pH 4.01, 6.86, 9.18 a temperatura ambiente
   - Ejecutar `calibrate_ph_3points.py`
   - **IMPORTANTE:** Esperar 3-5 minutos de estabilización por buffer
   - El script verifica estabilidad (σ < 0.002V por 50 segundos)

3. **Aplicar parámetros al firmware:**
   - Copiar slope y offset desde `calibration_3point_result.txt`
   - Editar `main/main.c`: `sensors_calibrate_ph_manual(slope, offset)`
   - Recompilar y flashear: `idf.py build flash`

4. **Verificar calibración:**
   - Probar en agua de pH conocido (medido con sensor manual)
   - Error esperado: <0.05 pH

**Guía detallada:** Ver [docs/CALIBRATION.md](docs/CALIBRATION.md)

## 🌡️ Calibración del Sensor de Temperatura

### Estado Actual del Sistema (v3.1.0)

**✅ Sistema completamente calibrado:**

| Parámetro | Valor | Notas |
|-----------|-------|-------|
| **Calibration Slope** | 1.086092 | Factor de corrección multiplicativo |
| **Calibration Offset** | -0.423°C | Corrección aditiva |
| **R² (ajuste lineal)** | 0.999999 | Ajuste prácticamente perfecto |
| **Precisión lograda** | ≤0.03°C | Verificado en 3 puntos (0°C, 23°C, 43°C) |
| **Rango calibrado** | 0-44°C | Hielo, ambiente, agua caliente |
| **Error máximo** | 0.03°C | En los 3 puntos de calibración |
| **Referencia usada** | TP101 Digital | Termómetro de referencia profesional |

### Herramientas de Calibración

```bash
# 1. Monitor de temperatura en tiempo real
python3 scripts/monitor_temperature.py

# 2. Calibración de 3 puntos (espera 3 min por punto)
python3 scripts/calibrate_temperature.py
```

### Proceso de Calibración de Temperatura

1. **Preparar 3 puntos de calibración:**
   - **Punto frío:** Agua con hielo (~0°C)
   - **Punto ambiente:** Agua a temperatura ambiente (~20-25°C)
   - **Punto caliente:** Agua caliente (~40-50°C)
   - Tener termómetro de referencia (TP101 o similar)

2. **Ejecutar calibración:**
   ```bash
   python3 scripts/calibrate_temperature.py
   ```
   - Seguir instrucciones en pantalla
   - **IMPORTANTE:** Esperar 3 minutos de estabilización por punto
   - El script calcula automáticamente slope y offset por regresión lineal

3. **Aplicar parámetros al firmware:**
   - Los valores se guardan automáticamente en `sdkconfig.defaults`:
     ```
     CONFIG_BIOFLOC_TEMP_SLOPE=1086092
     CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES=-423
     ```
   - Recompilar y flashear: `idf.py build flash`

4. **Verificar calibración:**
   - Al iniciar, el ESP32 muestra: `Temperature calibration loaded: slope=1.086092, offset=-0.423°C`
   - Comparar lecturas con termómetro de referencia
   - Error esperado: ≤0.1°C

**Fórmula de calibración:** `T_calibrada = slope × T_raw + offset`

**Guía detallada:** Ver [docs/CALIBRATION.md](docs/CALIBRATION.md)

## MongoDB Bridge (Almacenamiento de Datos)

### Arquitectura v3.0

Diseño de 2 colecciones para escalabilidad y consultas eficientes:

**telemetria** - Lecturas de sensores (series temporales)
- Indexada por (device_id, timestamp) para consultas rápidas por dispositivo
- Indexada por (timestamp) para lecturas recientes de todos los dispositivos

**devices** - Metadatos y estado de dispositivos
- Clave primaria: device_id (dirección MAC)
- Auto-registro de nuevos dispositivos
- Historial de conexión y estadísticas
- Parámetros de configuración y calibración

### Configuración Inicial

**1. Configurar archivo de entorno:**

```bash
cd /home/Biofloc-Firmware-ROS/scripts
cp .env.example .env
nano .env
```

**Variables de entorno (.env):**
```bash
MONGODB_URI=mongodb+srv://user:PASSWORD@cluster.mongodb.net/?retryWrites=true&w=majority
MONGODB_DATABASE=SistemasLab
MONGODB_COLLECTION=telemetria
MONGODB_COLLECTION_DEVICES=devices
ROS_TOPIC=/biofloc/sensor_data
LOG_DATA=true
```

**2. Instalar dependencias:**

```bash
pip install pymongo python-dotenv
```

**3. Ejecutar migración (solo primera vez):**

```bash
cd /home/Biofloc-Firmware-ROS/scripts
python3 migrate_to_devices_collection.py
```

Esto:
- Crea colección devices desde system_config existente
- Crea índices optimizados en telemetria
- Calcula estadísticas de conexión
- Verifica éxito de la migración

**4. Verificar migración:**

```bash
python3 verify_migration.py
```

**NOTA IMPORTANTE:** No es necesario flashear el ESP32. Los cambios son solo en el bridge Python (backend), el firmware continúa funcionando sin modificaciones.

### Uso Diario (3 Terminales)

**Terminal 1 - micro-ROS Agent:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2 - Ver datos (opcional):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /biofloc/sensor_data std_msgs/msg/String
```

**Terminal 3 - Guardar en MongoDB:**
```bash
cd /home/Biofloc-Firmware-ROS/scripts
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
python3 sensor_db_bridge.py
```

### Formatos de Documentos

**Colección telemetria (lecturas de sensores):**
```json
{
  "_id": ObjectId("..."),
  "device_id": "biofloc_esp32_c8e0",
  "location": "tanque_01",
  "timestamp": "2026-01-29T16:34:17-0300",
  "sensors": {
    "ph": { "value": 7.06, "voltage": 2.58, "unit": "pH", "valid": true },
    "temperature": { "value": 22.2, "voltage": 2.10, "unit": "C", "valid": true }
  },
  "_ros_topic": "/biofloc/sensor_data"
}
```

**Colección devices (metadatos de dispositivos):**
```json
{
  "_id": "biofloc_esp32_c8e0",
  "alias": "ESP32-c8e0",
  "location": "tanque_01",
  "estado": "activo",
  "auto_registrado": true,
  "firmware_version": "2.3.0",
  "intervalo_lectura_seg": 4,
  "sensores_habilitados": ["ph", "temperatura"],
  "calibracion": {
    "ph_slope": 2.559823,
    "ph_offset": 0.469193,
    "voltage_divider": 1.474
  },
  "umbrales": {
    "ph": { "min_value": 6.5, "max_value": 8.5 },
    "temperatura": { "min_value": 20, "max_value": 30 }
  },
  "unidades": {
    "temperatura": "°C",
    "ph": "pH"
  },
  "conexion": {
    "primera": "2026-01-21T14:00:00-0300",
    "ultima": "2026-01-29T16:34:17-0300",
    "total_lecturas": 17210
  }
}
```

### Notas Importantes

- **Timezone:** ESP32 usa CLT3 (GMT-3 fijo, Chile)
- **Timestamp:** Generado por ESP32 después de sincronizar con NTP
- **Tasa de guardado:** ~1 mensaje cada 4 segundos (250 msg/hora)
- **Success rate:** 100% (verificado con 17,000+ documentos)
- **Auto-registro:** Nuevos dispositivos se crean automáticamente en colección devices
- **Índices:** Consultas optimizadas con índices compuestos para tiempos de respuesta <5ms
- **Sin cambios en firmware:** No requiere flashear el ESP32, cambios solo en backend Python

## micro-ROS Agent

En tu PC con ROS 2 Jazzy (compilado localmente en ~/microros_ws/):

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

## Estructura del Proyecto

```
Biofloc-Firmware-ROS/
├── GUIA_PASO_A_PASO.md              # Guía de ejecución paso a paso
├── README.md                        # Este archivo
├── CMakeLists.txt                   # CMake raíz del proyecto ESP-IDF
├── sdkconfig                        # Configuración actual del proyecto
├── partitions.csv                   # Tabla de particiones (2MB flash)
├── calibration_3point_result.txt    # Resultados de última calibración
│
├── main/
│   ├── main.c                       # Firmware principal v2.2.0 (sin cambios)
│   ├── sensors.c                    # Driver de sensores CWT-BL
│   ├── sensors.h                    # API de sensores
│   ├── CMakeLists.txt               # CMake del componente
│   └── Kconfig.projbuild            # Opciones de menuconfig
│
├── scripts/
│   ├── sensor_db_bridge.py          # Bridge ROS 2 -> MongoDB v3.0
│   ├── migrate_to_devices_collection.py  # Script de migración MongoDB
│   ├── verify_migration.py          # Verificación de migración
│   ├── calibrate_ph.py              # Calibración de pH (3 puntos)
│   ├── monitor_sensores.py          # Monitor en tiempo real
│   ├── monitor_temperature.py       # Monitor de temperatura
│   ├── check_mongodb.py             # Verificador de conexión MongoDB
│   ├── .env.example                 # Plantilla de configuración
│   └── .env                         # Credentials (NOT in git)
│
├── docs/
│   ├── CALIBRATION.md               # pH calibration guide
│   ├── TROUBLESHOOTING.md           # Problem solving
│   └── SECURITY.md                  # Security guidelines
│
└── components/
    └── micro_ros_espidf_component/  # micro-ROS Jazzy component
```

## Kconfig Configuration

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `BIOFLOC_WIFI_SSID` | MyNetwork | WiFi network SSID |
| `BIOFLOC_WIFI_PASSWORD` | MyPassword | WiFi password |
| `BIOFLOC_WIFI_MAXIMUM_RETRY` | 5 | Connection retries |
| `BIOFLOC_AGENT_IP` | 192.168.1.100 | micro-ROS Agent IP |
| `BIOFLOC_AGENT_PORT` | 8888 | Agent UDP port |
| `BIOFLOC_ROS_NAMESPACE` | biofloc | ROS 2 namespace |
| `BIOFLOC_PING_TIMEOUT_MS` | 1000 | Ping timeout |
| `BIOFLOC_PING_RETRIES` | 10 | Ping retries |
| `BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR` | 1474 | Divider factor × 1000 (1.474) |
| `BIOFLOC_TIMEZONE` | CLT3 | Timezone (CLT3 = Chile GMT-3) |
| `BIOFLOC_NTP_SERVER` | pool.ntp.org | NTP server for sync |
| `BIOFLOC_LOCATION` | tanque_01 | Location identifier |

## 🔄 Flujo de Inicialización

1. **NVS Init** - Inicializa almacenamiento no volátil
2. **WiFi Connect** - Conecta a la red configurada
3. **Agent Ping** - Verifica conectividad con micro-ROS Agent
4. **micro-ROS Init** - Crea nodo y executor
5. **Main Loop** - Spin del executor + ping periódico

## 📝 Extender el Firmware

Para añadir publishers/subscribers, editar `main.c`:

```c
// Declarar
static rcl_publisher_t publisher;
static std_msgs__msg__Float32 msg;

// En microros_init()
RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "sensor_data"
));

// En el loop
msg.data = 25.5f;
rcl_publish(&publisher, &msg, NULL);
```

## 🐛 Troubleshooting

### pH Sensor

**Lectura fuera de rango (>14 o <0):**
- Verificar voltaje en GPIO con multímetro
- Revisar `CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR` en `sdkconfig`
- Ejecutar `scripts/fix_voltage_divider.py` para diagnóstico

**Error >0.3 pH:**
- Realizar calibración de 3 puntos con `scripts/calibrate_ph_3points.py`
- Usar soluciones buffer pH 4.01, 6.86, 9.18
- Esperar 3-5 minutos de estabilización por buffer
- Verificar que R² > 0.999 en resultados

**Sensor no estabiliza:**
- Enjuagar con agua desmineralizada entre mediciones
- Revisar temperatura del agua (±2°C)
- Asegurar que el sensor está completamente sumergido
- Esperar mínimo 3 minutos antes de leer

### micro-ROS Agent

**"Agent unreachable":**
- Verificar que el Agent está corriendo: `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`
- Confirmar IP y puerto en `idf.py menuconfig`
- Revisar firewall del host: `sudo ufw allow 8888/udp`

**"WiFi connection failed":**
- Verificar SSID y contraseña
- Asegurar que la red está en rango
- Verificar banda WiFi (2.4GHz recomendado para ESP32)

### MongoDB Bridge

**"Connection refused":**
- Verificar `.env` con credenciales correctas
- Verificar IP whitelisting en MongoDB Atlas (0.0.0.0/0 para testing)
- Verificar que pymongo está instalado: `pip list | grep pymongo`

**"No data received":**
- Verificar que micro-ROS Agent está ejecutando
- Confirmar que el firmware está publicando: `ros2 topic echo /biofloc/sensor_data`
- Revisar namespace de ROS 2 (`BIOFLOC_ROS_NAMESPACE`)

### Build Errors

**"Component not found":**
```bash
idf.py fullclean
idf.py build
```

**"Flash size too small":**
- Verificar `partitions.csv` para módulos con flash de 2MB
- Usar `idf.py size` para analizar uso de memoria

## 📊 Especificaciones Técnicas

### Sensor CWT-BL pH
- **Rango:** 0-14 pH
- **Salida:** 0-5V (linear, típicamente pH = V × 2.8)
- **Tiempo de respuesta:** 3-5 minutos (estabilización completa)
- **Precisión post-calibración:** ±0.03 pH (verificado)
- **Factor divisor de voltaje:** 1.474 (R1=20kΩ, R2=10kΩ, calibrado)
- **Fórmula calibrada:** pH = 2.5598 × V_sensor + 0.4692

### Firmware v2.2.0
- **Tamaño binario:** 867 KB (57% flash libre, 2MB total)
- **RAM disponible:** ~98 KB (70% libre)
- **Tasa de publicación:** 1 Hz (cada ~4 segundos)
- **Formato de mensaje:** ROS 2 custom `SensorData.msg`
- **Calibración aplicada:** Automática al inicio (hardcoded en firmware)

### Sistema de Telemetría
- **Protocolo:** UDP sobre WiFi (micro-ROS middleware)
- **Puerto Agent:** 8888 UDP
- **Latencia típica:** <50ms (LAN)
- **Base de datos:** MongoDB Atlas (cloud)
- **Guardado:** ~250 registros/hora, success rate 100%
- **Timestamp:** Sincronizado con NTP (pool.ntp.org)

## 📜 Changelog

### v2.2.0 (2026-01-21) - pH Calibration System

**Added:**
- Sistema completo de calibración de pH de 3 puntos
- Script `calibrate_ph_3points.py` con timeout extendido (7 min)
- Script `monitor_voltage.py` para verificación en tiempo real
- Script `fix_voltage_divider.py` para diagnóstico del divisor
- Script `diagnose_ph.py` para troubleshooting general
- Archivo `calibration_3point_result.txt` para guardar resultados
- Documentación completa en `docs/CALIBRATION.md`

**Changed:**
- Voltage divider factor: 3.0 → 1.474 (calibrado con multímetro)
- Timezone: `CLT3CLST` → `CLT3` (GMT-3 fijo, sin horario de verano)
- MongoDB bridge: eliminado campo `_received_at` redundante
- Calibración pH aplicada en firmware: slope=2.5598, offset=0.4692

**Fixed:**
- Error de lectura pH: 14.8 → 7.09 (diferencia de 7.71 pH corregida)
- Timestamps incorrectos: ahora marca hora local correcta (GMT-3)
- Estabilización del sensor: aumentado timeout de calibración a 3-7 min

**Performance:**
- Precisión pH: ±0.27 pH → ±0.03 pH (mejora 9x)
- R² calibración: 0.9997 (ajuste casi perfecto)
- Error máximo en buffers: 0.049 pH

## 👤 Autor

**Marton1123**
- GitHub: [@Marton1123](https://github.com/Marton1123)
- Repositorio: [Biofloc-Firmware-ROS](https://github.com/Marton1123/Biofloc-Firmware-ROS)
