# 🚀 Guía Paso a Paso — Biofloc Firmware ROS

> **Versión:** 2.3.0 | **Tiempo estimado:** 30-45 min (primera vez) | 5 min (uso diario)

---

## 📋 Índice

1. [Requisitos Previos](#1-requisitos-previos)
2. [Instalación Inicial](#2-instalación-inicial-solo-primera-vez)
3. [Uso Diario](#3-uso-diario)
4. [Calibración del Sensor](#4-calibración-del-sensor-de-ph)
5. [Solución de Problemas](#5-solución-de-problemas-rápida)

---

## 1. Requisitos Previos

### Hardware
- ✅ ESP32 conectado por USB (`/dev/ttyUSB0`)
- ✅ Sensor CWT-BL de pH/Temperatura conectado
- ✅ PC con Ubuntu 24.04+ y WiFi en la misma red que el ESP32

### Software (ya instalado en este sistema)
- ✅ ESP-IDF v5.3.4 en `~/esp/v5.3.4/esp-idf/`
- ✅ ROS 2 Jazzy en `/opt/ros/jazzy/`
- ✅ micro-ROS Agent en `~/microros_ws/`
- ✅ Python 3.12+ con pymongo, python-dotenv

---

## 2. Instalación Inicial (Solo Primera Vez)

### Paso 2.1: Configurar WiFi y IP del Agent

```bash
# Entrar al directorio del proyecto
cd /home/Biofloc-Firmware-ROS

# Cargar entorno ESP-IDF
source ~/esp/v5.3.4/esp-idf/export.sh

# Abrir configuración
idf.py menuconfig
```

**En el menú, navegar a:**
```
Biofloc Configuration
├── WiFi Configuration
│   ├── WiFi SSID: [tu_red_wifi]
│   └── WiFi Password: [tu_password]
│
└── micro-ROS Agent Configuration
    ├── micro-ROS Agent IP: [IP_de_tu_PC]  ← Importante!
    └── micro-ROS Agent Port: 8888
```

> 💡 **Para saber la IP de tu PC:** `ip addr show | grep "192.168"`

**Guardar y salir:** `S` (Save) → `Enter` → `Q` (Quit)

### Paso 2.2: Compilar el Firmware

```bash
# Compilar (toma ~2-3 minutos la primera vez)
idf.py build
```

**Salida esperada:**
```
Project build complete. To flash, run:
 idf.py flash
```

### Paso 2.3: Flashear al ESP32

```bash
# Flashear al ESP32
idf.py -p /dev/ttyUSB0 flash
```

**Salida esperada:**
```
Writing at 0x000e2a18... (100 %)
Wrote 867904 bytes...
Hard resetting via RTS pin...
```

---

## 3. Uso Diario

### ⚡ Comando Rápido (Copiar y Pegar)

**Terminal 1 - Iniciar micro-ROS Agent:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Terminal 2 - Ver datos del sensor (solo visualizar):**
```bash
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 topic echo /biofloc/sensor_data std_msgs/msg/String
```

**Terminal 3 - Guardar en MongoDB (opcional):**
```bash
cd /home/Biofloc-Firmware-ROS/scripts && source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && python3 sensor_db_bridge.py
```

> ⚠️ **Importante:** Sin el Terminal 3, los datos solo se muestran pero **NO se guardan** en la base de datos.

---

### 📝 Paso a Paso Detallado

#### Paso 3.1: Abrir Terminal 1 - Iniciar el Agent

```bash
# 1. Cargar entorno ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Cargar micro-ROS Agent (compilado localmente)
source ~/microros_ws/install/local_setup.bash

# 3. Iniciar el Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Salida esperada (cuando ESP32 se conecta):**
```
[info] | UDPv4AgentLinux.cpp | init | running... | port: 8888
[info] | Root.cpp | create_client | create | client_key: 0x6CB95074
[info] | SessionManager.hpp | establish_session | session established
[info] | ProxyClient.cpp | create_participant | participant created
[info] | ProxyClient.cpp | create_topic | topic created
[info] | ProxyClient.cpp | create_publisher | publisher created
```

> ⏳ Esperar ~10 segundos para que el ESP32 se conecte

#### Paso 3.2: Abrir Terminal 2 - Monitorear Datos

```bash
# 1. Cargar entorno ROS 2
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash

# 2. Ver datos en tiempo real
ros2 topic echo /biofloc/sensor_data std_msgs/msg/String
```

**Salida esperada:**
```
data: '{"device_id":"biofloc_esp32_c8e0","timestamp":"2026-01-22T16:00:00-0300","location":"tanque_01","sensors":{"ph":{"value":7.06,"voltage":2.58,"unit":"pH","valid":true},"temperature":{"value":22.0,"voltage":2.10,"unit":"C","valid":true}}}'
---
```

> 📊 Los datos llegan cada ~4 segundos
> 
> ⚠️ **Nota:** Este comando solo MUESTRA los datos, NO los guarda en MongoDB.

#### Paso 3.3: Guardar en MongoDB (Terminal 3)

> 🗄️ **Este paso es NECESARIO para almacenar datos en la base de datos.**

```bash
# Terminal 3 - EJECUTAR SI QUIERES GUARDAR EN MONGODB
cd /home/Biofloc-Firmware-ROS/scripts
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
python3 sensor_db_bridge.py
```

**Salida esperada:**
```
✓ Entorno cargado desde: /home/Biofloc-Firmware-ROS/scripts/.env
✓ Conectado a MongoDB: SistemasLab.telemetria
============================================================
🚀 Sensor DB Bridge Iniciado
   Topic: /biofloc/sensor_data
   Base de datos: SistemasLab
   Almacenamiento conectado: True
============================================================
[biofloc_esp32@tanque_01] pH: 7.06 ✓ | Temp: 22.0°C ✓
```

**⚠️ Requisito previo:** Configurar credenciales de MongoDB:
```bash
cd /home/Biofloc-Firmware-ROS/scripts
cp .env.example .env
nano .env  # Editar MONGODB_URI con tu conexión
```

---

## 4. Calibración del Sensor de pH

### Paso 4.1: Preparar Materiales

- ✅ Solución buffer pH 4.01 (ácida - roja)
- ✅ Solución buffer pH 6.86 (neutra - amarilla)  
- ✅ Solución buffer pH 9.18 (alcalina - azul)
- ✅ Agua destilada para enjuagar
- ✅ Papel absorbente

### Paso 4.2: Ejecutar Calibración

```bash
# Terminal (con Agent corriendo en otro terminal)
cd /home/Biofloc-Firmware-ROS
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
python3 scripts/calibrate_ph.py
```

### Paso 4.3: Seguir las Instrucciones en Pantalla

1. **Seleccionar calibración de 3 puntos** (opción 3)
2. **Para cada buffer:**
   - Enjuagar sensor con agua destilada
   - Secar suavemente
   - Sumergir en buffer
   - Presionar Enter
   - Esperar ~3 minutos (el script espera estabilización)
3. **Al finalizar:** El script mostrará los parámetros

### Paso 4.4: Aplicar Calibración al Firmware

```bash
# 1. Editar main.c con los valores del script
nano main/main.c

# 2. Buscar la línea (aproximadamente línea 248):
#    sensors_calibrate_ph_manual(2.559823f, 0.469193f);
#    Reemplazar con tus nuevos valores

# 3. Recompilar y flashear
source ~/esp/v5.3.4/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

---

## 5. Solución de Problemas Rápida

### ❌ "No se detecta el sensor" o "Agent unreachable"

```bash
# 1. Verificar que el Agent está corriendo
pgrep -f "micro_ros_agent" && echo "✅ Agent activo" || echo "❌ Agent NO activo"

# 2. Si no está activo, iniciarlo:
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### ❌ "No hay datos del ESP32"

```bash
# 1. Verificar topics
source /opt/ros/jazzy/setup.bash && ros2 topic list | grep biofloc

# 2. Si no hay topic, reiniciar ESP32 (presionar botón EN)

# 3. Si aún no funciona, verificar IP configurada
source ~/esp/v5.3.4/esp-idf/export.sh
cd /home/Biofloc-Firmware-ROS
idf.py menuconfig
# Verificar: Biofloc Configuration → micro-ROS Agent IP
```

### ❌ "Permission denied: /dev/ttyUSB0"

```bash
# Agregar usuario al grupo dialout
sudo usermod -a -G dialout $USER

# Cerrar sesión y volver a entrar, o ejecutar:
newgrp dialout
```

### ❌ "idf.py: command not found"

```bash
# Cargar entorno ESP-IDF
source ~/esp/v5.3.4/esp-idf/export.sh
```

### ❌ pH lee valores incorrectos (>14 o <0)

```bash
# Verificar voltaje con monitor
cd /home/Biofloc-Firmware-ROS
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
python3 scripts/monitor_temperature.py

# Si el voltaje es muy diferente al esperado, recalibrar
python3 scripts/calibrate_ph.py
```

---

## 📊 Verificación Rápida del Sistema

```bash
# Ejecutar este script para verificar todo:
echo "=== VERIFICACIÓN DEL SISTEMA ===" && \
echo "" && \
echo "1. ESP32 conectado:" && \
ls /dev/ttyUSB0 2>/dev/null && echo "   ✅ Sí" || echo "   ❌ No" && \
echo "" && \
echo "2. Agent corriendo:" && \
pgrep -f "micro_ros_agent" > /dev/null && echo "   ✅ Sí" || echo "   ❌ No" && \
echo "" && \
echo "3. Topics activos:" && \
source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q biofloc && echo "   ✅ Sí" || echo "   ❌ No" && \
echo "" && \
echo "=== FIN VERIFICACIÓN ==="
```

---

## 📁 Estructura de Archivos Importantes

```
/home/Biofloc-Firmware-ROS/
├── main/
│   ├── main.c              ← Código principal del ESP32
│   ├── sensors.c           ← Driver de sensores
│   └── sensors.h           ← API de sensores
├── scripts/
│   ├── calibrate_ph.py     ← Script de calibración
│   ├── monitor_temperature.py  ← Monitor en tiempo real
│   └── sensor_db_bridge.py ← Puente a MongoDB
├── sdkconfig               ← Configuración actual
└── calibration_3point_result.txt  ← Última calibración
```

---

## 🔄 Resumen de Comandos Frecuentes

| Acción | Comando |
|--------|---------|
| Iniciar Agent | `source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888` |
| Ver datos | `ros2 topic echo /biofloc/sensor_data std_msgs/msg/String` |
| Compilar | `source ~/esp/v5.3.4/esp-idf/export.sh && idf.py build` |
| Flashear | `idf.py -p /dev/ttyUSB0 flash` |
| Configurar | `idf.py menuconfig` |
| Calibrar pH | `python3 scripts/calibrate_ph.py` |

---

---

**Versión:** 1.0.0  
**Última actualización:** 2026-01-22  
**Autor:** [@Marton1123](https://github.com/Marton1123)  
**Repositorio:** [Biofloc-Firmware-ROS](https://github.com/Marton1123/Biofloc-Firmware-ROS)
