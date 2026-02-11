# 🚀 Guía Paso a Paso — Biofloc Firmware ROS

> **Versión:** 3.0.0 (Secure Gateway + Manager) | **Tiempo estimado:** 30-45 min (primera vez) | 1 min (uso diario con gestor)

---

## 📋 Índice

1. [Requisitos Previos](#1-requisitos-previos)
2. [Instalación Inicial](#2-instalación-inicial-solo-primera-vez)
3. [Uso Diario con Gestor](#3-uso-diario-con-gestor-recomendado)
4. [Uso Diario Manual](#4-uso-diario-manual-alternativa)
5. [Calibración del Sensor](#5-calibración-del-sensor)
6. [Solución de Problemas](#6-solución-de-problemas-rápida)

---

## 1. Requisitos Previos

### Hardware
- ✅ **Gateway** Intel NUC o PC Linux con Ubuntu 24.04+
- ✅ **Gateway** con WiFi (wlo1) + Ethernet (enp88s0)
- ✅ **ESP32** conectado por USB (`/dev/ttyUSB0`) o WiFi (10.42.0.x)
- ✅ **Sensor** CWT-BL de pH/Temperatura conectado al ESP32
- ✅ **Voltage Divider** R1=10kΩ, R2=20kΩ (factor 1.5) en ambos sensores

### Software (ya instalado en este sistema)
- ✅ ESP-IDF v5.3.4 en `~/esp/v5.3.4/esp-idf/`
- ✅ ROS 2 Jazzy en `/opt/ros/jazzy/`
- ✅ micro-ROS Agent en `~/microros_ws/`
- ✅ Python 3.12+ con pymongo, python-dotenv
- ✅ **biofloc_manager.py** (gestor unificado)

### Red (Arquitectura Gateway Seguro)
- ✅ Hotspot WiFi en gateway: SSID `<tu-ssid-gateway>`, IP `10.42.0.1/24`
- ✅ Firewall iptables: FORWARD DROP (ESP32 sin internet)
- ✅ ESP32 obtiene IP `10.42.0.x` vía DHCP
- ✅ ESP32 se comunica SOLO con gateway (UDP 8888)

---

## 2. Instalación Inicial (Solo Primera Vez)

### Paso 2.1: Configurar Gateway (Hotspot WiFi)

```bash
# Crear hotspot WiFi en gateway
nmcli device wifi hotspot \
  ifname wlo1 \
  ssid "<tu-ssid-gateway>" \
  password "<tu-password-seguro>"

# Configurar autoconexión
nmcli connection modify Hotspot connection.autoconnect yes

# Verificar (debe mostrar 10.42.0.1/24)
ip addr show wlo1
```

**Ver:** [MIGRATION_GUIDE_SECURE_GATEWAY.md](MIGRATION_GUIDE_SECURE_GATEWAY.md) para firewall iptables completo.

### Paso 2.2: Configurar WiFi en ESP32

**Opción A: Usar Gestor (Recomendado)**
```bash
cd /home/Biofloc-Firmware-ROS
python3 biofloc_manager.py
# Seleccionar: [10] Configurar WiFi
```

**Opción B: Editar manualmente**
```bash
cd /home/Biofloc-Firmware-ROS
nano sdkconfig.defaults
```

**Cambiar ambos sets de credenciales:**
```ini
# Credenciales para micro_ros_espidf_component
CONFIG_ESP_WIFI_SSID="<tu-ssid-gateway>"
CONFIG_ESP_WIFI_PASSWORD="<tu-password-seguro>"

# Credenciales para aplicación biofloc
CONFIG_BIOFLOC_WIFI_SSID="<tu-ssid-gateway>"
CONFIG_BIOFLOC_WIFI_PASSWORD="<tu-password-seguro>"
```

⚠️ **IMPORTANTE:** Ambos sets deben ser idénticos.

### Paso 2.2: Compilar y Flashear el Firmware

**Opción A: Usar Gestor (Recomendado)**
```bash
cd /home/Biofloc-Firmware-ROS
python3 biofloc_manager.py
# Seleccionar: [12] Compilar y Flashear
```

**Opción B: Manual**
```bash
# Cargar entorno ESP-IDF
source ~/esp/v5.3.4/esp-idf/export.sh

# Regenerar sdkconfig desde defaults
rm sdkconfig
idf.py reconfigure

# Compilar (toma ~2-3 minutos la primera vez)
idf.py build

# Flashear al ESP32
idf.py -p /dev/ttyUSB0 flash
```

**Salida esperada:**
```
Writing at 0x000e2a18... (100 %)
Wrote 867904 bytes...
Hard resetting via RTS pin...
```

**Verificar conexión:**
```bash
idf.py -p /dev/ttyUSB0 monitor
```

**Debe mostrar:**
```
I (3421) WIFI: WiFi connected to <tu-ssid-gateway>
I (3425) WIFI: Got IP: 10.42.0.x
I (3430) MAIN: Connecting to micro-ROS agent at 10.42.0.1:8888
```

---

## 3. Uso Diario con Gestor (Recomendado)

### ⚡ Opción Rápida: Gestor Unificado

```bash
cd /home/Biofloc-Firmware-ROS
python3 biofloc_manager.py
```

**Menú principal (12 opciones):**

```
═══════════════════════════════════════════════════════
           Gestor de Firmware Biofloc v1.0
═══════════════════════════════════════════════════════

────────────── Operaciones del Sistema ──────────────
 1. ▶️  Iniciar micro-ROS Agent
 2. ▶️  Iniciar sensor_db_bridge.py  
 3. 📊 Iniciar monitor_sensores.py

───────────────── Verificación ─────────────────────
 4. ✅ Verificar estado del sistema
 5. 🔌 Verificar conectividad ESP32

───────────────── Calibración ──────────────────────
 6. 🧪 Calibración completa pH (3 puntos)
 7. 🌡️ Calibración completa Temperatura (3 puntos)
 8. ⚡ Ajuste rápido pH
 9. ⚡ Ajuste rápido Temperatura

───────────────── Configuración ────────────────────
10. 📶 Configurar WiFi
11. ⚙️ Regenerar sdkconfig

───────────────── Firmware ─────────────────────────
12. 🛠️ Compilar y Flashear

 0. ❌ Salir
═══════════════════════════════════════════════════════
Selecciona una opción: _
```

### Uso Típico:

**Para iniciar el sistema:**
1. Opción [1] - Inicia micro-ROS Agent (deja corriendo)
2. Opción [2] - Inicia Bridge a MongoDB (deja corriendo)
3. Opción [4] - Verifica que ESP32 esté publicando

**Para verificar conectividad:**
- Opción [5] - Diagnóstico completo (DHCP, ARP, ping, ROS)

**Para calibrar sensores:**
- Opción [6] o [7] - Calibración completa (3 puntos, 15-30 min)
- Opción [8] o [9] - Ajuste rápido (1 valor, 30 segundos)

---

## 4. Uso Diario Manual (Alternativa)

Si prefieres NO usar el gestor:

### Paso 4.1: Iniciar micro-ROS Agent (Terminal 1)

```bash
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Salida esperada (cuando ESP32 se conecta):**
```
[info] | UDPv4AgentLinux.cpp | init | running... | port: 8888
[info] | Root.cpp | create_client | create | client_key: 0x6CB95074
[info] | SessionManager.hpp | establish_session | session established
```

> ⏳ Esperar ~10 segundos para que el ESP32 se conecte

### Paso 4.2: Ver Datos del Sensor (Terminal 2)

```bash
source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 topic echo /biofloc/sensor_data
```

**Salida esperada:**
```yaml
data: '{"timestamp":"sample_1234","ph":7.08,"temperature":23.5,"device_id":"biofloc_esp32_c8e0","location":"tanque_01"}'
---
```

### Paso 4.3: Guardar a MongoDB (Terminal 3 - Opcional)

```bash
cd /home/Biofloc-Firmware-ROS/scripts && source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && python3 sensor_db_bridge.py
```

**Salida esperada:**
```
✓ Conectado a MongoDB Atlas
✓ Subscrito a /biofloc/sensor_data
[2026-02-10 14:32:15] Guardado: pH=7.08, Temp=23.5°C
```

> ⚠️ **Importante:** Sin el Terminal 3, los datos solo se muestran pero **NO se guardan** en la base de datos.

---

## 5. Calibración del Sensor

### Opción A: Usar Gestor (Más Fácil)

```bash
python3 biofloc_manager.py
```

**Para calibración completa (3 puntos):**
- Selecciona [6] para pH o [7] para Temperatura
- Sigue las instrucciones en pantalla
- Duración: 15-30 minutos
- Actualiza automáticamente sdkconfig.defaults

**Para ajuste rápido (1 valor conocido):**
- Selecciona [8] para pH o [9] para Temperatura
- Ingresa valor actual y valor esperado
- Duración: 30 segundos
- Actualiza automáticamente sdkconfig.defaults

### Opción B: Scripts Individuales

**Calibración pH (3 puntos):**
```bash
cd /home/Biofloc-Firmware-ROS
python3 scripts/calibrate_ph.py
```

**Calibración Temperatura (3 puntos):**
```bash
python3 scripts/calibrate_temperature.py
```

**Materiales necesarios:**
- Soluciones buffer pH: 4.01, 6.86, 9.18
- Termómetro de referencia (±0.1°C)
- Contenedores para sumergir sensores

---

## 6. Solución de Problemas Rápida

### ESP32 no se conecta al WiFi

**Verificar con gestor:**
```bash
python3 biofloc_manager.py
# Opción [5] - Verificar conectividad ESP32
```

**Comandos manuales:**
```bash
# Ver hotspot activo
nmcli connection show --active | grep Hotspot

# Ver IP del gateway (debe ser 10.42.0.1)
ip addr show wlo1

# Ver si ESP32 está en DHCP leases
cat /var/lib/NetworkManager/dnsmasq-wlo1.leases | grep XX:XX:XX:XX:XX:XX

# Ver en ARP table
ip neigh show dev wlo1
```

**Solución:**
- Verificar que hotspot esté activo
- Verificar credenciales en sdkconfig.defaults (DUAL)
- Reiniciar ESP32
- Usar gestor opción [10] para reconfigurar WiFi

### ESP32 conectado pero no publica datos

**Verificar con gestor:**
```bash
python3 biofloc_manager.py
# Opción [4] - Verificar estado del sistema
```

**Debe mostrar:**
- ✅ micro-ROS Agent: CORRIENDO
- ✅ Topic ROS: /biofloc/sensor_data DISPONIBLE
- ✅ ESP32 publicando: SÍ (en 8s)

**Si falla:**
1. Verificar que Agent esté corriendo (opción [1])
2. Verificar que ESP32 tenga IP 10.42.0.x
3. Hacer ping al ESP32: `ping -c 3 10.42.0.x`
4. Revisar firewall permite UDP 8888

### Datos no llegan a MongoDB

**Verificar:**
```bash
python3 biofloc_manager.py
# Opción [4] - Debe mostrar sensor_db_bridge.py CORRIENDO
```

**Si no está corriendo:**
```bash
python3 biofloc_manager.py
# Opción [2] - Iniciar sensor_db_bridge.py
```

**Verificar credenciales MongoDB:**
```bash
cat /home/Biofloc-Firmware-ROS/scripts/.env
# Debe tener MONGODB_URI válida
```

### Lecturas de sensores incorrectas

**Verificar hardware:**
- R1 = 10kΩ (pull-up)
- R2 = 20kΩ (pull-down)
- Factor = 1.5

**Recalibrar con gestor:**
```bash
python3 biofloc_manager.py
# [6] pH completa o [8] pH rápida
# [7] Temp completa o [9] Temp rápida
```

### ESP32 puede acceder a internet (SECURITY ISSUE!)

```bash
# Verificar firewall
sudo iptables -L FORWARD -v -n
# Debe mostrar: Chain FORWARD (policy DROP)

# Si no está DROP:
sudo iptables -P FORWARD DROP

# Guardar
sudo netfilter-persistent save
```

**Ver:** [MIGRATION_GUIDE_SECURE_GATEWAY.md](MIGRATION_GUIDE_SECURE_GATEWAY.md) para configuración completa.
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
