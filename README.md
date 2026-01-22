# Biofloc Firmware ROS

Sistema completo de telemetría IoT para acuicultura con **micro-ROS Jazzy**, **ESP-IDF v5.3** y **MongoDB Atlas**.

**Versión:** 2.3.0 (Bridge Fix + Documentation)

---

## 🚀 ¿Primera vez? Empieza aquí

👉 **[GUÍA PASO A PASO](GUIA_PASO_A_PASO.md)** — Instrucciones completas para ejecutar el proyecto

---

## ✨ Características

- 🌊 Monitoreo en tiempo real de pH y temperatura
- 📡 Telemetría vía micro-ROS sobre WiFi UDP
- 🗄️ Almacenamiento automático en MongoDB Atlas
- 🎯 Calibración de pH de 3 puntos (precisión <0.05 pH)
- 🔄 Reconexión automática WiFi y Agent
- 📊 Publicación JSON estructurada
- 🛠️ Herramientas de diagnóstico y calibración

## 📋 Requisitos

### Hardware
- ESP32 (240MHz, Dual Core, WiFi)
- Sensor CWT-BL pH/Temperature Transmitter (0-5V output)
- Voltage divider: R1=20kΩ, R2=10kΩ
- Soluciones buffer pH 4.01, 6.86, 9.18 (para calibración)

### Software
- ESP-IDF v5.3.4+ instalado y configurado
- ROS 2 Jazzy Desktop
- Python 3.12+ con pymongo, python-dotenv, numpy
- micro-ROS Agent
- MongoDB Atlas account (opcional)

## 🚀 Quick Start

### 1. Configurar el entorno ESP-IDF

```bash
. $HOME/esp/esp-idf/export.sh
# o donde tengas instalado ESP-IDF
```

### 2. Configurar el target

```bash
cd /home/Biofloc-Firmware-ROS
idf.py set-target esp32  # o esp32s3, esp32c3, etc.
```

### 3. Configurar WiFi y Agent

```bash
idf.py menuconfig
```

Navegar a: **Biofloc Configuration** →
- **WiFi Configuration**: SSID y Password
- **micro-ROS Agent Configuration**: IP y Puerto del Agent

### 4. Compilar

```bash
idf.py build
```

### 5. Flashear y Monitorear

```bash
idf.py -p /dev/ttyUSB0 flash monitor
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

## 🗄️ MongoDB Bridge (Almacenamiento de Datos)

### Configuración

1. **Crear archivo `.env` en el directorio `scripts/`:**

```bash
cd /home/Biofloc-Firmware-ROS/scripts
cp .env.example .env
nano .env  # Editar MONGODB_URI con tu conexión
```

**Contenido del `.env`:**
```bash
MONGODB_URI=mongodb+srv://usuario:PASSWORD@cluster.mongodb.net/?retryWrites=true&w=majority
MONGODB_DATABASE=SistemasLab
MONGODB_COLLECTION=telemetria
ROS_TOPIC=/biofloc/sensor_data
LOG_DATA=true
```

2. **Instalar dependencias:**

```bash
pip install pymongo python-dotenv
```

### Uso (3 Terminales)

**Terminal 1 — micro-ROS Agent:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2 — Ver datos (opcional):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /biofloc/sensor_data std_msgs/msg/String
```

**Terminal 3 — Guardar en MongoDB:**
```bash
cd /home/Biofloc-Firmware-ROS/scripts
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
python3 sensor_db_bridge.py
```

### Formato de Documento en MongoDB

```json
{
  "device_id": "biofloc_esp32_c8e0",
  "location": "tanque_01",
  "timestamp": "2026-01-22T16:34:17-0300",
  "sensors": {
    "ph": { "value": 7.06, "voltage": 2.58, "unit": "pH", "valid": true },
    "temperature": { "value": 22.2, "voltage": 2.10, "unit": "C", "valid": true }
  },
  "_ros_topic": "/biofloc/sensor_data"
}
```

### Notas Importantes

- **Timezone:** El ESP32 usa **CLT3** (GMT-3 fijo, Chile)
- **Timestamp:** Generado por el ESP32 después de sincronizar con NTP
- **Tasa de guardado:** ~1 mensaje cada 4 segundos (250 msg/hora)
- **Success rate:** 100% (verificado con 17,000+ documentos)

## 🖥️ micro-ROS Agent

En tu PC con ROS 2 Jazzy (compilado localmente en `~/microros_ws/`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

## 📁 Estructura del Proyecto

```
Biofloc-Firmware-ROS/
├── GUIA_PASO_A_PASO.md              # 🚀 Guía de ejecución paso a paso
├── README.md                        # Este archivo
├── CMakeLists.txt                   # CMake raíz del proyecto ESP-IDF
├── sdkconfig                        # Configuración actual del proyecto
├── partitions.csv                   # Tabla de particiones (2MB flash)
├── calibration_3point_result.txt    # Resultados de última calibración
│
├── main/
│   ├── main.c                       # Firmware principal v2.2.0
│   ├── sensors.c                    # Driver de sensores CWT-BL
│   ├── sensors.h                    # API de sensores
│   ├── CMakeLists.txt               # CMake del componente
│   └── Kconfig.projbuild            # Opciones de menuconfig
│
├── scripts/
│   ├── sensor_db_bridge.py          # 🗄️ Bridge ROS 2 → MongoDB
│   ├── calibrate_ph.py              # 🎯 Calibración de pH (3 puntos)
│   ├── monitor_sensores.py          # 📊 Monitor en tiempo real
│   ├── monitor_temperature.py       # 🌡️ Monitor de temperatura
│   ├── check_mongodb.py             # ✅ Verificar conexión MongoDB
│   ├── .env.example                 # Plantilla de configuración
│   └── .env                         # Credenciales (NO en git)
│
├── docs/
│   ├── CALIBRATION.md               # Guía de calibración de pH
│   ├── TROUBLESHOOTING.md           # Solución de problemas
│   └── SECURITY.md                  # Guías de seguridad
│
└── components/
    └── micro_ros_espidf_component/  # Componente micro-ROS Jazzy
```

## ⚙️ Configuración Kconfig

| Parámetro | Valor por defecto | Descripción |
|-----------|-------------------|-------------|
| `BIOFLOC_WIFI_SSID` | MyNetwork | SSID de la red WiFi |
| `BIOFLOC_WIFI_PASSWORD` | MyPassword | Contraseña WiFi |
| `BIOFLOC_WIFI_MAXIMUM_RETRY` | 5 | Reintentos de conexión |
| `BIOFLOC_AGENT_IP` | 192.168.1.100 | IP del micro-ROS Agent |
| `BIOFLOC_AGENT_PORT` | 8888 | Puerto UDP del Agent |
| `BIOFLOC_ROS_NAMESPACE` | biofloc | Namespace de ROS 2 |
| `BIOFLOC_PING_TIMEOUT_MS` | 1000 | Timeout del ping |
| `BIOFLOC_PING_RETRIES` | 10 | Reintentos de ping |
| `BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR` | 1474 | Factor divisor × 1000 (1.474) |
| `BIOFLOC_TIMEZONE` | CLT3 | Timezone (CLT3 = Chile GMT-3) |
| `BIOFLOC_NTP_SERVER` | pool.ntp.org | Servidor NTP para sincronización |
| `BIOFLOC_LOCATION` | tanque_01 | Identificador de ubicación |

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
