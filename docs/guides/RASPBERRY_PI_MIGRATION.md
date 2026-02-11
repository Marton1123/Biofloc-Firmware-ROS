# Migración de NUC a Raspberry Pi 3

**Versión:** 3.0.0  
**Fecha:** Febrero 2026  
**Autor:** @Marton1123

---

## 📋 Resumen de Cambios

Se ha migrado el nodo central ROS de una **NUC Ubuntu 24.04** a una **Raspberry Pi 3 con Ubuntu Server 24.04**. La Raspberry Pi actúa exclusivamente como **Gateway/Agente micro-ROS**, sin capacidad de compilación de firmware ESP32.

### Cambios de Configuración

| Concepto | NUC (Anterior) | Raspberry Pi 3 (Actual) |
|----------|----------------|-------------------------|
| **WiFi Interface** | `wlp0s20f3` | `wlan0` |
| **SSID Gateway** | `old-ssid` | `<tu-nuevo-ssid>` |
| **Password** | `old-password` | `<tu-password-seguro>` |
| **Gateway IP** | `10.42.0.1` | `10.42.0.1` (sin cambio) |
| **Agent Port** | `8888` | `8888` (sin cambio) |
| **ESP-IDF** | ✅ Instalado | ❌ No requerido |
| **Rol** | Gateway + Dev | Gateway únicamente |

---

## 🚀 Instalación en Raspberry Pi 3

### Requisitos Previos

- Raspberry Pi 3 Model B/B+
- Ubuntu Server 24.04 LTS (ARM64)
- MicroSD 32GB+ (Clase 10)
- Conexión Ethernet para internet
- ESP32 ya flasheado con firmware actualizado

### 1. Preparar Sistema Base

```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade -y

# Instalar herramientas básicas
sudo apt install -y \
  software-properties-common \
  curl \
  git \
  python3-pip \
  python3-venv \
  network-manager \
  iptables \
  iptables-persistent
```

### 2. Instalar ROS 2 Jazzy

```bash
# Agregar repositorios ROS 2
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Agregar llave GPG
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Agregar repositorio ROS 2
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS 2 Jazzy (base + tools)
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools

# Verificar
source /opt/ros/jazzy/setup.bash
ros2 --version
```

### 3. Instalar micro-ROS Workspace

```bash
# Instalar dependencias
sudo apt install -y python3-colcon-common-extensions

# Crear workspace
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

# Clonar micro-ROS setup
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git

# Compilar workspace
cd ~/microros_ws
source /opt/ros/jazzy/setup.bash
colcon build

# Source el workspace
source install/local_setup.bash

# Crear agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

### 4. Clonar Repositorio Biofloc

```bash
# Clonar repo
cd ~
git clone https://github.com/Marton1123/Biofloc-Firmware-ROS.git
cd Biofloc-Firmware-ROS

# Copiar configuración
cp .env.example .env

# Editar .env con tus credenciales MongoDB
nano .env
```

**Configurar `.env`:**
```bash
# MongoDB (pegar tu URI real)
MONGODB_URI=mongodb+srv://usuario:password@cluster.mongodb.net/...
MONGODB_DATABASE=SistemasLab
MONGODB_COLLECTION=telemetria
MONGODB_COLLECTION_DEVICES=devices

# Gateway Raspberry Pi (YA CONFIGURADO)
GATEWAY_IP=10.42.0.1
GATEWAY_WIFI_INTERFACE=wlan0
GATEWAY_WIFI_SSID=Biofloc-Gateway
GATEWAY_WIFI_PASSWORD=<tu-password-seguro>
GATEWAY_NETWORK=10.42.0.0/24

# ESP32 (tu dispositivo actual)
ESP32_MAC=XX:XX:XX:XX:XX:XX
ESP32_IP=10.42.0.x
```

### 5. Instalar Dependencias Python

```bash
cd ~/Biofloc-Firmware-ROS

# Opción 1: Sistema (no recomendado)
pip3 install pymongo python-dotenv --break-system-packages

# Opción 2: Virtual Environment (RECOMENDADO)
python3 -m venv venv
source venv/bin/activate
pip install pymongo python-dotenv
```

### 6. Configurar Hotspot WiFi

```bash
# Crear conexión hotspot
sudo nmcli connection add \
  type wifi \
  ifname wlan0 \
  con-name Biofloc-Gateway \
  autoconnect yes \
  ssid "Biofloc-Gateway"

# Configurar como Access Point
sudo nmcli connection modify Biofloc-Gateway \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  ipv4.method shared \
  ipv4.addresses 10.42.0.1/24

# Agregar seguridad WPA2
sudo nmcli connection modify Biofloc-Gateway \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "<tu-password-seguro>"

# Activar hotspot
sudo nmcli connection up Biofloc-Gateway

# Verificar
ip addr show wlan0  # Debe mostrar 10.42.0.1
```

### 7. Configurar Firewall (ESP32 sin Internet)

```bash
# Bloquear acceso ESP32 → Internet
sudo iptables -A FORWARD -i wlan0 -o eth0 -j DROP

# Permitir comunicación interna (ESP32 ↔ Gateway)
sudo iptables -A FORWARD -i wlan0 -o wlan0 -j ACCEPT

# Permitir UDP 8888 (micro-ROS Agent)
sudo iptables -A INPUT -i wlan0 -p udp --dport 8888 -j ACCEPT

# Guardar reglas permanentemente
sudo netfilter-persistent save

# Verificar
sudo iptables -L -v -n
```

### 8. Configurar Auto-sourcing

```bash
# Editar .bashrc
nano ~/.bashrc

# Agregar al final:
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0

# Recargar
source ~/.bashrc
```

---

## 🔧 Flashear ESP32 con Nuevas Credenciales

**⚠️ IMPORTANTE:** El ESP32 debe flashearse desde un PC con ESP-IDF (no en la Raspberry Pi).

### En tu PC de Desarrollo:

```bash
# Clonar repo actualizado
git clone https://github.com/tuusuario/Biofloc-Firmware-ROS.git
cd Biofloc-Firmware-ROS

# Verificar credenciales en sdkconfig.defaults
grep "CONFIG_ESP_WIFI" sdkconfig.defaults

# Debe mostrar:
# CONFIG_ESP_WIFI_SSID="Biofloc-Gateway"
# CONFIG_ESP_WIFI_PASSWORD="<tu-password-seguro>"

# Conectar ESP32 por USB
ls /dev/ttyUSB*

# Compilar y flashear
source ~/esp/v5.3.4/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

**Verificar en el Monitor:**
```
I (3245) WIFI: Connected to Biofloc-Gateway
I (3250) WIFI: Got IP: 10.42.0.x
I (3255) MICROROS: Agent IP: 10.42.0.1:8888
```

---

## ✅ Verificación del Sistema

### En la Raspberry Pi:

```bash
cd ~/Biofloc-Firmware-ROS

# Usar el gestor unificado
python3 biofloc_manager.py
```

**Seleccionar Opción 1: Verificar Estado**

Debe mostrar:
- ✅ **Raspberry Pi detectada** - Modo Gateway (sin compilación firmware)
- ✅ **Workspace micro-ROS**: OK
- ✅ **Directorio scripts**: OK
- ✅ **ESP32**: Conectado en 10.42.0.x
- ✅ **ROS Topics**: `/biofloc/sensor_data` activo

### Iniciar Sistema Completo:

**Terminal 1: micro-ROS Agent**
```bash
python3 biofloc_manager.py
# Opción 2: Iniciar micro-ROS Agent
```

**Terminal 2: Sensor Bridge**
```bash
python3 biofloc_manager.py
# Opción 3: Iniciar Sensor DB Bridge
```

**Terminal 3: Monitoreo (opcional)**
```bash
ros2 topic echo /biofloc/sensor_data
```

---

## 📊 Comparativa de Rendimiento

| Métrica | NUC | Raspberry Pi 3 |
|---------|-----|----------------|
| **CPU** | Intel i3/i5 | ARM Cortex-A53 |
| **RAM** | 8-32GB | 1GB |
| **Latencia Agent** | ~5ms | ~15ms |
| **Throughput** | 1000 msg/s | 200 msg/s |
| **Consumo** | ~20W | ~5W ⚡ |
| **Costo** | $$$$ | $ |
| **Compilación ESP32** | ✅ Sí | ❌ No |

**Veredicto:** Rendimiento suficiente para telemetría cada 4 segundos (0.25 Hz). Para mayor frecuencia, considerar Raspberry Pi 4.

---

## 🐛 Troubleshooting

### Problema: ESP32 no se conecta al hotspot

**Solución:**
```bash
# Verificar hotspot activo
nmcli connection show --active | grep Biofloc-Gateway

# Reiniciar hotspot
sudo nmcli connection down Biofloc-Gateway
sudo nmcli connection up Biofloc-Gateway

# Verificar IP del gateway
ip addr show wlan0 | grep "inet "
```

### Problema: Agent no recibe datos

**Solución:**
```bash
# Verificar firewall
sudo iptables -L -v -n | grep 8888

# Si no está, agregar:
sudo iptables -A INPUT -i wlan0 -p udp --dport 8888 -j ACCEPT
sudo netfilter-persistent save

# Verificar ESP32 en red
ping 10.42.0.x -c 3
```

### Problema: Bridge no guarda a MongoDB

**Solución:**
```bash
# Verificar conectividad internet (Raspberry debe tener ethernet)
ping 8.8.8.8 -c 3

# Verificar MongoDB URI en .env
grep MONGODB_URI ~/Biofloc-Firmware-ROS/.env

# Probar conexión directa
cd ~/Biofloc-Firmware-ROS/scripts
python3 check_mongodb.py
```

---

## 📚 Referencias

- [Ubuntu Server 24.04 para Raspberry Pi](https://ubuntu.com/download/raspberry-pi)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [micro-ROS for ESP32](https://micro.ros.org/)
- [Biofloc Repository](https://github.com/Marton1123/Biofloc-Firmware-ROS)

---

## 📝 Changelog

### v3.0.0 - Migración Raspberry Pi 3 (Febrero 2026)

- ✅ Migrado nodo central de NUC a Raspberry Pi 3
- ✅ Nuevo SSID: `<tu-ssid>` (password: `<tu-password-seguro>`)
- ✅ Detección automática de arquitectura ARM en `biofloc_manager.py`
- ✅ Verificación de ESP-IDF opcional en Raspberry Pi
- ✅ Actualizado `.env.example` con interfaz `wlan0`
- ✅ Documentación completa de instalación

**Compatibilidad:** Ubuntu Server 24.04 ARM64, ROS 2 Jazzy, ESP-IDF v5.3

---

**Licencia:** MIT  
**Última actualización:** Febrero 11, 2026
