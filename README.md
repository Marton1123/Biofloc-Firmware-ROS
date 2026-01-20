# Biofloc Firmware ROS

Firmware profesional para ESP32 con **micro-ROS Jazzy** sobre **ESP-IDF v5.3**.

## 📋 Requisitos

- ESP-IDF v5.3+ instalado y configurado
- ESP32, ESP32-S2, ESP32-S3, ESP32-C3, o ESP32-C6
- micro-ROS Agent corriendo en un host (PC/Raspberry Pi)
- Red WiFi

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

## 🖥️ Levantar el micro-ROS Agent

En tu PC/host con ROS 2 Jazzy:

```bash
# Opción 1: Docker (recomendado)
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6

# Opción 2: Desde source
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

## 📁 Estructura del Proyecto

```
Biofloc-Firmware-ROS/
├── CMakeLists.txt                   # CMake raíz del proyecto ESP-IDF
├── sdkconfig                        # Configuración actual del proyecto
├── sdkconfig.defaults               # Configuración por defecto
├── partitions.csv                   # Tabla de particiones (2MB flash)
├── components/
│   └── micro_ros_espidf_component/  # Componente micro-ROS Jazzy (rama jazzy)
├── main/
│   ├── CMakeLists.txt               # CMake del componente main
│   ├── Kconfig.projbuild            # Configuración para menuconfig
│   └── main.c                       # Firmware principal (WiFi + micro-ROS)
└── src/                             # Módulos adicionales (futuro)
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

### "Agent unreachable"
- Verificar que el Agent está corriendo
- Confirmar IP y puerto en menuconfig
- Revisar firewall del host

### "WiFi connection failed"
- Verificar SSID y contraseña
- Asegurar que la red está en rango

### Build errors con micro-ROS
```bash
idf.py fullclean
idf.py build
```
