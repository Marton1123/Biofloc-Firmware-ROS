# 🛠️ Guía de Troubleshooting — Biofloc Firmware ROS

> **Estándares:** ITIL v4 (Gestión de Incidentes) | ISO 20000 (Gestión de Servicios TI)

| Metadatos | Valor |
|-----------|-------|
| **Versión** | 2.2.0 |
| **Última Actualización** | 2026-01-22 |
| **Tiempo de Resolución Esperado** | 5-15 minutos |

---

## 📋 Tabla de Contenidos

1. [Diagnóstico Rápido](#-diagnóstico-rápido)
2. [Sistema No Responde](#-problema-el-sistema-se-pega-deja-de-responder)
3. [Procedimiento de Recuperación](#%EF%B8%8F-procedimiento-de-recuperación-completo)
4. [Verificación de Calibración](#-verificación-de-calibración-de-ph)
5. [Comandos de Referencia](#-comandos-rápidos-de-referencia)
6. [Arquitectura del Sistema](#-arquitectura-del-sistema)
7. [FAQ](#-faq)

---

## ⚡ Diagnóstico Rápido

```bash
# Ejecutar en orden para diagnóstico rápido:

# 1. ¿Hay topics de ROS 2?
source /opt/ros/jazzy/setup.bash && ros2 topic list | grep biofloc

# 2. ¿Llegan datos?
source ~/microros_ws/install/local_setup.bash
timeout 10 ros2 topic echo /biofloc/sensor_data --once

# 3. ¿Agent está corriendo?
pgrep -f "micro_ros_agent" && echo "✅ Agent activo" || echo "❌ Agent NO activo"

# 4. ¿Puerto 8888 abierto?
sudo lsof -i :8888 | head -3
```

## 🔴 Problema: El Sistema se "Pega" (Deja de Responder)

### Síntomas
- El ESP32 deja de publicar datos en `/biofloc/sensor_data`
- El micro-ROS Agent no muestra actividad
- No hay respuesta a ping del Agent
- Los LEDs del ESP32 pueden quedar en estado fijo

### Causas y Soluciones

| # | Causa | Probabilidad | Tiempo de Resolución |
|---|-------|--------------|---------------------|
| 1 | micro-ROS Agent no corriendo | Alta (60%) | 2 min |
| 2 | ESP32 perdió conexión WiFi | Media (25%) | 3 min |
| 3 | Conflicto de IP/Puerto | Baja (10%) | 5 min |
| 4 | Timeout de Watchdog | Baja (5%) | 5 min |

#### 1. micro-ROS Agent No Está Corriendo
**Diagnóstico:**
```bash
# Verificar si hay topics activos
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep biofloc
```

**Solución:**
```bash
# Iniciar el Agent desde el workspace correcto
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

> ⚠️ **IMPORTANTE:** El Agent está instalado en `~/microros_ws/`, NO como paquete del sistema.

#### 2. ESP32 Perdió Conexión WiFi
**Diagnóstico:**
- Revisar el monitor serial del ESP32
- El firmware hace auto-restart después de N intentos fallidos

**Solución:**
```bash
# Reiniciar el ESP32 (presionar botón EN o desconectar/conectar USB)
# El firmware intentará reconectar automáticamente
```

#### 3. Conflicto de Puertos/IPs
**Diagnóstico:**
```bash
# Verificar que el puerto 8888 no esté ocupado
sudo lsof -i :8888

# Verificar IP del Agent
ip addr show | grep "inet "
```

**Solución:**
- Asegurar que la IP del Agent en el firmware coincida con la IP real del host
- Configurar via `idf.py menuconfig` → Biofloc Configuration → micro-ROS Agent

#### 4. Timeout de Watchdog
El firmware tiene watchdog de 10 segundos. Si una tarea se bloquea, el ESP32 se reiniciará automáticamente.

**Si el reinicio automático no funciona:**
```bash
# Flashear de nuevo
source ~/esp/v5.3.4/esp-idf/export.sh
cd /home/Biofloc-Firmware-ROS
idf.py -p /dev/ttyUSB0 flash
```

---

## 🛠️ Procedimiento de Recuperación Completo

### Paso 1: Verificar Infraestructura de Red

```bash
# Verificar WiFi del host
ping -c 3 192.168.0.1  # Router

# Verificar IP del host (debe ser la misma configurada en el ESP32)
ip addr show | grep "192.168"
```

### Paso 2: Iniciar micro-ROS Agent

```bash
# Terminal 1: Iniciar Agent
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4

# La opción -v4 muestra verbose logging para debug
```

Deberías ver algo como:
```
[info] | UDPv4AgentLinux.cpp | init | running... | port: 8888
```

### Paso 3: Verificar Conexión del ESP32

Espera ~10 segundos después de iniciar el Agent. Deberías ver:
```
[info] | Root.cpp | create_client | create | client_key: 0x6CB95074
[info] | SessionManager.hpp | establish_session | session established
[info] | ProxyClient.cpp | create_participant | participant created
[info] | ProxyClient.cpp | create_topic | topic created
[info] | ProxyClient.cpp | create_publisher | publisher created
```

### Paso 4: Verificar Topics ROS 2

```bash
# Terminal 2: Verificar topics
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash

ros2 topic list
# Debe mostrar: /biofloc/sensor_data

ros2 topic echo /biofloc/sensor_data std_msgs/msg/String
# Debe mostrar datos JSON cada ~4 segundos
```

### Paso 5: Si Aún No Hay Datos

1. **Reiniciar ESP32:** Presionar botón EN o desconectar/conectar alimentación
2. **Verificar monitor serial:**
   ```bash
   idf.py -p /dev/ttyUSB0 monitor
   # Ctrl+] para salir
   ```
3. **Re-flashear si es necesario:**
   ```bash
   source ~/esp/v5.3.4/esp-idf/export.sh
   cd /home/Biofloc-Firmware-ROS
   idf.py -p /dev/ttyUSB0 flash monitor
   ```

---

## 📊 Verificación de Calibración de pH

### Datos de Calibración Actuales
```
Slope:  2.559823
Offset: 0.469193
R²:     0.9997
Max Error: 0.049 pH
```

### Verificar Lecturas
```bash
# Monitorear datos del sensor
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 topic echo /biofloc/sensor_data std_msgs/msg/String
```

Ejemplo de salida esperada:
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "timestamp": "2026-01-22T15:58:02-0300",
  "sensors": {
    "ph": {"value": 7.06, "voltage": 2.58, "unit": "pH", "valid": true},
    "temperature": {"value": 22.0, "voltage": 2.10, "unit": "C", "valid": true}
  }
}
```

### Re-calibrar pH (si es necesario)
```bash
cd /home/Biofloc-Firmware-ROS
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
python3 scripts/calibrate_ph.py
```

**Requisitos:**
- Soluciones buffer pH 4.01, 6.86, 9.18 (o al menos 2 puntos)
- Agua destilada para enjuagar
- ~10 minutos para calibración de 3 puntos

---

## 🔧 Comandos Rápidos de Referencia

### Iniciar Sistema Completo
```bash
# Terminal 1: Agent
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Terminal 2: Monitorear
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
ros2 topic echo /biofloc/sensor_data std_msgs/msg/String
```

### Build y Flash
```bash
source ~/esp/v5.3.4/esp-idf/export.sh
cd /home/Biofloc-Firmware-ROS
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

### Configuración
```bash
source ~/esp/v5.3.4/esp-idf/export.sh
cd /home/Biofloc-Firmware-ROS
idf.py menuconfig
# → Biofloc Configuration para WiFi, IP del Agent, etc.
```

### Monitor Serial ESP32
```bash
source ~/esp/v5.3.4/esp-idf/export.sh
idf.py -p /dev/ttyUSB0 monitor
# Ctrl+] para salir
```

---

## 📁 Archivos Importantes

| Archivo | Descripción |
|---------|-------------|
| `main/main.c` | Código principal del firmware |
| `main/sensors.c` | Driver de sensores y calibración |
| `main/Kconfig.projbuild` | Configuración de menuconfig |
| `sdkconfig.defaults` | Valores por defecto |
| `calibration_3point_result.txt` | Resultado de última calibración |
| `scripts/calibrate_ph.py` | Script de calibración interactivo |

---

## 🔄 Arquitectura del Sistema

```
┌─────────────────┐     UDP:8888      ┌──────────────────┐
│     ESP32       │ ◄───────────────► │  micro-ROS Agent │
│  (Firmware)     │                   │  (~/microros_ws) │
└────────┬────────┘                   └────────┬─────────┘
         │                                     │
   ┌─────┴─────┐                        ┌──────┴──────┐
   │  Sensores │                        │   ROS 2     │
   │  CWT-BL   │                        │   Topics    │
   │  pH/Temp  │                        │  /biofloc/* │
   └───────────┘                        └─────────────┘
```

**Tareas del ESP32:**
- `micro_ros_task` (CPU1): Comunicación ROS 2, ping al Agent
- `sensor_task` (CPU0): Lectura ADC, publicación JSON

**Watchdog:** 10 segundos - reinicio automático si se congela

---

## ❓ FAQ (Preguntas Frecuentes)

### ¿Por qué el Agent no es un paquete del sistema?

> El micro-ROS Agent para ROS 2 Jazzy se compiló localmente en `~/microros_ws/` porque no existe paquete apt oficial para esta versión. Esto es común en versiones recientes de ROS 2.

**Ubicación:** `~/microros_ws/install/local_setup.bash`

### ¿Cuál es el intervalo de muestreo?

> **4 segundos** por defecto.

**Configurar:**
```bash
idf.py menuconfig
# → Biofloc Configuration → Sensor Configuration → Sensor sampling interval
```

**Parámetro:** `CONFIG_BIOFLOC_SENSOR_SAMPLE_INTERVAL_MS`

### ¿Cómo cambiar la IP del Agent?

```bash
idf.py menuconfig
# → Biofloc Configuration → micro-ROS Agent Configuration → micro-ROS Agent IP
```

**Importante:** Después de cambiar, recompilar y reflashear:
```bash
idf.py build && idf.py -p /dev/ttyUSB0 flash
```

### ¿Se puede usar Docker para el Agent?

> **No en esta instalación.** El Agent está compilado nativamente en `~/microros_ws/`.

### ¿Cómo verificar que todo funciona?

```bash
# Test completo en 30 segundos:
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash
ros2 topic hz /biofloc/sensor_data --window 5
# Debe mostrar: average rate: 0.250 Hz (1 mensaje cada 4 seg)
```

---

## 📞 Contacto de Soporte

| Nivel | Contacto | Tiempo de Respuesta |
|-------|----------|--------------------|
| L1 - Operador | Consultar esta guía | Inmediato |
| L2 - Técnico | Equipo de Ingeniería | < 4 horas |
| L3 - Desarrollo | Biofloc Engineering Team | < 24 horas |

---

**Versión del Documento:** 2.0.0  
**Última Revisión:** 2026-01-22  
**Próxima Revisión:** 2026-04-22
