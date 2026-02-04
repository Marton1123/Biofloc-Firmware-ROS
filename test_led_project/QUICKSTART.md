# 🚀 Guía Rápida - Control LED micro-ROS

Guía de inicio rápido para usar el proyecto de prueba LED con micro-ROS.

## ⚡ Inicio Rápido (5 minutos)

### 1. Flashear ESP32
```bash
cd /home/Biofloc-Firmware-ROS/test_led_project
source /home/lab-ros2/esp/v5.3.4/esp-idf/export.sh
idf.py -p /dev/ttyUSB1 flash
```

### 2. Iniciar Agente micro-ROS (si no está corriendo)
```bash
# En otra terminal
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v
```

### 3. Controlar el LED
```bash
# Opción A: Script interactivo
source /opt/ros/jazzy/setup.bash
python3 keyboard_led_control.py

# Opción B: Comandos directos
ros2 topic pub /led_control std_msgs/msg/String "data: 'ON'"
ros2 topic pub /led_control std_msgs/msg/String "data: 'OFF'"
```

## 🎮 Controles

| Tecla | Acción |
|-------|--------|
| `E` | Encender 🟢 |
| `A` | Apagar 🔴 |
| `T` | Alternar 🔄 |
| `Q` | Salir ❌ |

## 🔍 Verificación Rápida

```bash
# Ver si el nodo está activo
ros2 node list | grep led_controller

# Ver mensajes en tiempo real
ros2 topic echo /led_control
```

## ⚠️ Troubleshooting Express

**ESP32 no conecta al agente:**
```bash
# Verificar agente corriendo
ps aux | grep micro_ros_agent

# Ver log del ESP32
idf.py -p /dev/ttyUSB1 monitor
```

**Error de numpy:**
```bash
pip3 install numpy
```

## 📝 Comandos Útiles

```bash
# Recompilar
idf.py build

# Flash + Monitor en un comando
idf.py -p /dev/ttyUSB1 flash monitor

# Solo monitorear
idf.py -p /dev/ttyUSB1 monitor

# Borrar flash completo
esptool.py --port /dev/ttyUSB1 erase_flash
```

## 🌐 Configuración de Red

Si cambias de red WiFi, edita `sdkconfig.defaults`:
```ini
CONFIG_ESP_WIFI_SSID="TU_RED"
CONFIG_ESP_WIFI_PASSWORD="TU_PASSWORD"
CONFIG_MICRO_ROS_AGENT_IP="192.168.X.X"
```

Luego recompila y flashea.

## 📊 Información del Sistema

- **ESP32 IP:** 192.168.0.69
- **Agente IP:** 192.168.0.76:8888
- **Nodo ROS:** `/led_controller`
- **Tópico:** `/led_control`
- **GPIO LED:** 2

## 🔗 Más Información

- `README.md` - Documentación completa
- `DESARROLLO.md` - Historial del proyecto
- `SOLUCION_PROBLEMA_CONEXION.md` - Solución al bug crítico

---

¿Problemas? Revisa `SOLUCION_PROBLEMA_CONEXION.md` para el bug más común.
