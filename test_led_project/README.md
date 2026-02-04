# 🎮 Proyecto de Prueba: Control de LED con micro-ROS

Proyecto de prueba simple para controlar un LED en ESP32 desde el teclado de la computadora usando ROS 2 y micro-ROS.

## 📋 Descripción

Este proyecto demuestra:
- ✅ Control de GPIO desde micro-ROS
- ✅ Suscripción a tópicos ROS
- ✅ Comunicación ESP32 ↔ Computadora via ROS
- ✅ Control interactivo por teclado

## 🔧 Hardware Requerido

- **ESP32** (cualquier modelo con WiFi)
- **LED** (o usar el LED integrado en GPIO 2)
- **Cable USB** para programación

### Conexión del LED

**Opción 1: LED Integrado (más fácil)**
- No requiere conexiones, usa el LED integrado del ESP32

**Opción 2: LED Externo**
```
ESP32 GPIO 2 ──►[LED]──►[220Ω]──► GND
```

## 📦 Requisitos de Software

### En la Computadora:
- ROS 2 Jazzy (o cualquier distribución)
- Python 3
- micro-ROS Agent

### En el ESP32:
- ESP-IDF (versión compatible con tu proyecto)
- Componente micro-ROS ESP-IDF

## 🚀 Instalación

### 1. Copiar el componente micro-ROS

Este proyecto necesita el componente `micro_ros_espidf_component`. Puedes:

**Opción A: Crear symlink (recomendado)**
```bash
cd test_led_project
mkdir -p components
ln -s ../components/micro_ros_espidf_component components/micro_ros_espidf_component
```

**Opción B: Copiar el componente**
```bash
cd test_led_project
mkdir -p components
cp -r ../components/micro_ros_espidf_component components/
```

### 2. Configurar WiFi

Edita `sdkconfig.defaults` y ajusta:
```
CONFIG_MICRO_ROS_AGENT_IP="192.168.1.100"  # IP de tu computadora
CONFIG_WIFI_SSID="TU_RED_WIFI"
CONFIG_WIFI_PASSWORD="TU_PASSWORD"
```

O usa menuconfig:
```bash
idf.py menuconfig
```

### 3. Compilar y Flashear

```bash
cd test_led_project

# Limpiar (opcional)
idf.py fullclean

# Compilar
idf.py build

# Flashear al ESP32
idf.py -p /dev/ttyUSB0 flash

# Monitorear salida
idf.py -p /dev/ttyUSB0 monitor
```

## 🎯 Uso

### Paso 1: Iniciar micro-ROS Agent

En una terminal de la computadora:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v
```

### Paso 2: Verificar que el ESP32 se conectó

Deberías ver en el monitor serie:
```
✅ Agente detectado
✅ Nodo 'led_controller' creado
✅ Suscrito a /led_control
🚀 Sistema listo!
```

### Paso 3: Controlar el LED

**Opción A: Script de teclado interactivo (RECOMENDADO)**

En otra terminal:
```bash
source /opt/ros/jazzy/setup.bash
cd test_led_project
python3 keyboard_led_control.py
```

Presiona las teclas:
- **E** → Encender LED 🟢
- **A** → Apagar LED 🔴
- **T** → Alternar LED 🔄
- **Q** → Salir

**Opción B: Comandos ROS directos**

```bash
source /opt/ros/jazzy/setup.bash

# Encender
ros2 topic pub /led_control std_msgs/msg/String "data: 'ON'" --once

# Apagar
ros2 topic pub /led_control std_msgs/msg/String "data: 'OFF'" --once

# Alternar
ros2 topic pub /led_control std_msgs/msg/String "data: 'TOGGLE'" --once
```

## 📊 Verificación

### Ver tópicos activos:
```bash
ros2 topic list
```
Deberías ver: `/led_control`

### Ver mensajes en tiempo real:
```bash
ros2 topic echo /led_control
```

### Ver nodos activos:
```bash
ros2 node list
```
Deberías ver: `/led_controller`

## 🔍 Solución de Problemas

### ❌ El ESP32 no se conecta al agente

1. Verifica que el agente esté corriendo en el puerto correcto
2. Confirma la IP en `sdkconfig.defaults`
3. Verifica que estén en la misma red WiFi
4. Revisa el firewall de tu computadora

### ❌ El LED no responde

1. Verifica que el ESP32 esté suscrito: `ros2 topic info /led_control`
2. Confirma el GPIO correcto (GPIO 2 por defecto)
3. Prueba con comandos ROS directos primero

### ❌ Error de compilación

1. Verifica que el componente micro-ROS esté presente
2. Ejecuta: `idf.py fullclean` y vuelve a compilar

## 🎓 Aprendizaje

Este proyecto es ideal para:
- Entender la comunicación micro-ROS básica
- Aprender a suscribirse a tópicos
- Practicar control de GPIO desde ROS
- Base para proyectos más complejos

## 📝 Comandos Soportados

El ESP32 acepta estos comandos en `/led_control`:
- `"ON"`, `"on"`, `"1"` → Encender
- `"OFF"`, `"off"`, `"0"` → Apagar  
- `"TOGGLE"`, `"toggle"` → Alternar

## 🔗 Relación con el Proyecto Principal

Este es un proyecto **completamente independiente** que no afecta el firmware principal de Biofloc. Usa:
- ✅ GPIO diferente (GPIO 2 vs GPIO 34/36 del proyecto principal)
- ✅ Tópico diferente (`/led_control` vs `/biofloc/sensor_data`)
- ✅ Nodo diferente (`led_controller` vs `biofloc_telemetry`)

## 📚 Próximos Pasos

Una vez que esto funcione, puedes:
1. Agregar más LEDs en diferentes GPIOs
2. Crear un publisher que envíe el estado del LED
3. Agregar un botón físico que también controle el LED
4. Integrar sensores adicionales

## 🤝 Soporte

Para más información sobre el proyecto principal, ver:
- `../README.md` - Proyecto Biofloc principal
- `../DOCUMENTATION_INDEX.md` - Documentación completa

---

**Versión:** 1.0.0  
**Fecha:** Febrero 2026
