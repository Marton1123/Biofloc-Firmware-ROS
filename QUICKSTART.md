# 🚀 Guía Rápida de Inicio - Biofloc Firmware ROS

Esta guía te llevará desde cero hasta tener el sistema funcionando en **30 minutos**.

---

## ⚡ Quick Start (Si ya tienes todo instalado)

```bash
# 1. Exportar ESP-IDF
. /home/lab-ros2/esp/v5.3.4/esp-idf/export.sh

# 2. Configurar WiFi y Agent
cd /home/Biofloc-Firmware-ROS
idf.py menuconfig
# → Biofloc Configuration → WiFi/Agent

# 3. Compilar y flashear
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# 4. En otra terminal: iniciar micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 5. En otra terminal: iniciar MongoDB bridge
cd /home/Biofloc-Firmware-ROS
source scripts/.venv/bin/activate
python3 scripts/sensor_db_bridge.py
```

**Listo! Los datos deberían estar llegando a MongoDB.**

---

## 📦 Instalación Completa (Primera Vez)

### 1. Prerequisitos

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip \
    python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util \
    libusb-1.0-0 python3-setuptools
```

### 2. Instalar ESP-IDF v5.3.4

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v5.3.4 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
. ./export.sh
```

**Verifica:** `idf.py --version` debería mostrar `v5.3.4`

### 3. Instalar ROS 2 Jazzy

```bash
# Agregar repositorio ROS 2
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Instalar ROS 2 Jazzy Desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Configurar entorno
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

# Instalar micro-ROS Agent
sudo apt install -y ros-jazzy-micro-ros-agent
```

**Verifica:** `ros2 --version` debería mostrar Jazzy

### 4. Clonar Repositorio

```bash
cd ~
git clone --recursive https://github.com/Marton1123/Biofloc-Firmware-ROS.git
cd Biofloc-Firmware-ROS
```

### 5. Configurar Python para Scripts

```bash
cd scripts
python3 -m venv .venv
source .venv/bin/activate
pip install pymongo python-dotenv numpy
```

### 6. Configurar MongoDB

Crear archivo `scripts/.env`:

```bash
MONGODB_URI=mongodb+srv://usuario:PASSWORD@cluster.mongodb.net/?retryWrites=true&w=majority
MONGODB_DB=SistemasLab
MONGODB_COLLECTION=telemetria
```

---

## 🔧 Configuración del Firmware

### Configurar WiFi y micro-ROS Agent

```bash
cd /home/Biofloc-Firmware-ROS
. ~/esp/esp-idf/export.sh
idf.py menuconfig
```

**Navega a:** `Biofloc Configuration`

**Configura:**
- WiFi SSID: `tu_red_wifi`
- WiFi Password: `tu_contraseña`
- Agent IP: `192.168.1.100` (IP de tu PC con ROS 2)
- Agent Port: `8888`
- Location: `tanque_01` (identificador del tanque)

**Guarda:** Presiona `S` → Enter → `Q`

### Compilar y Flashear

```bash
# Compilar
idf.py build

# Conectar ESP32 y verificar puerto
ls /dev/ttyUSB*

# Flashear
idf.py -p /dev/ttyUSB0 flash

# Monitorear (opcional)
idf.py -p /dev/ttyUSB0 monitor
# Salir con Ctrl+]
```

---

## 📊 Verificar Funcionamiento

### Terminal 1: micro-ROS Agent

```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Deberías ver:**
```
[INFO] [micro_ros_agent]: Successfully connected to client
```

### Terminal 2: Verificar Topics

```bash
source /opt/ros/jazzy/setup.bash

# Listar topics
ros2 topic list
# Debería aparecer: /biofloc/sensor_data

# Ver datos
ros2 topic echo /biofloc/sensor_data
```

**Deberías ver:**
```json
{
  "timestamp": "2026-01-21T17:30:00-0300",
  "ph": 7.08,
  "temperature": 2.5,
  "device_id": "biofloc_esp32_xxxx",
  "location": "tanque_01"
}
```

### Terminal 3: MongoDB Bridge

```bash
cd /home/Biofloc-Firmware-ROS
source scripts/.venv/bin/activate
python3 scripts/sensor_db_bridge.py
```

**Deberías ver:**
```
✓ Connected to MongoDB
[INFO] [sensor_db_bridge]: [biofloc_esp32_xxxx@tanque_01] pH: 7.08 ✓ | Temp: 2.5°C ✓
Stats: received=10, saved=10, failed=0, success_rate=100.0%
```

---

## 🎯 Calibración del Sensor (Importante!)

**⚠️ El sensor DEBE calibrarse antes de uso en producción.**

### Materiales Necesarios
- Soluciones buffer pH 4.01, 6.86, 9.18
- Agua desmineralizada para enjuague
- Multímetro digital (opcional pero recomendado)

### Proceso de Calibración

#### Paso 1: Verificar Divisor de Voltaje (si es necesario)

```bash
# Medir pH del agua con sensor manual
# pH medido: X.XX

# Ejecutar diagnóstico
python3 scripts/fix_voltage_divider.py
# Ingresar: voltaje del GPIO (multímetro) y pH medido
# El script calculará el factor correcto

# Si el factor es diferente a 1.474:
idf.py menuconfig
# → Biofloc Configuration → pH Voltage Divider Factor
# Ingresar nuevo valor × 1000 (ej: 1.474 → 1474)
# Recompilar: idf.py build flash
```

#### Paso 2: Calibración de 3 Puntos

```bash
# Ejecutar script de calibración
python3 scripts/calibrate_ph_3points.py

# El script pedirá:
# 1. Sumergir sensor en buffer pH 4.01 → Esperar 3-5 min
# 2. Enjuagar con agua desmineralizada
# 3. Sumergir sensor en buffer pH 6.86 → Esperar 3-5 min
# 4. Enjuagar con agua desmineralizada
# 5. Sumergir sensor en buffer pH 9.18 → Esperar 3-5 min

# Al final mostrará:
# Slope: 2.559823
# Offset: 0.469193
# R²: 0.9997
# Max Error: 0.049 pH
```

#### Paso 3: Aplicar Calibración al Firmware

```bash
# Editar main/main.c, buscar:
ret = sensors_calibrate_ph_manual(2.559823f, 0.469193f);

# Reemplazar con los valores obtenidos
# Recompilar y flashear:
idf.py build flash
```

#### Paso 4: Verificar

```bash
# Monitorear voltaje
python3 scripts/monitor_voltage.py

# Probar en agua de pH conocido
# Error esperado: <0.05 pH
```

**Guía completa:** Ver `docs/CALIBRATION.md`

---

## 🐛 Troubleshooting Común

### ❌ "WiFi connection failed"
```bash
# Verificar SSID y contraseña en menuconfig
idf.py menuconfig → Biofloc Configuration → WiFi

# Verificar que la red es 2.4GHz (ESP32 no soporta 5GHz)
```

### ❌ "Agent unreachable"
```bash
# Verificar que Agent está ejecutando
ps aux | grep micro_ros_agent

# Verificar IP del Agent
ifconfig  # o ip addr

# Actualizar IP en firmware:
idf.py menuconfig → Biofloc Configuration → Agent IP
```

### ❌ "MongoDB connection refused"
```bash
# Verificar .env
cat scripts/.env

# Probar conexión (reemplaza con tu cluster)
ping <tu-cluster>.mongodb.net

# Verificar IP whitelisting en MongoDB Atlas
# Dashboard → Network Access → Add IP Address → 0.0.0.0/0 (testing)
```

### ❌ "pH reading out of range (>14 or <0)"
```bash
# Verificar voltage divider factor
python3 scripts/fix_voltage_divider.py

# Recalibrar sensor
python3 scripts/calibrate_ph_3points.py
```

---

## 📚 Próximos Pasos

1. **Leer documentación completa:** `README.md`
2. **Calibrar el sensor:** `docs/CALIBRATION.md`
3. **Configurar producción:** `TECHNICAL_SUMMARY.md`
4. **Revisar changelog:** `CHANGELOG.md`

---

## 💡 Tips Útiles

- **Atajos de teclado:**
  - Salir de monitor: `Ctrl + ]`
  - Detener script Python: `Ctrl + C`
  - Limpiar terminal: `Ctrl + L`

- **Comandos útiles:**
  ```bash
  # Limpiar build
  idf.py fullclean
  
  # Ver tamaño de binario
  idf.py size
  
  # Solo flashear (sin compilar)
  idf.py flash
  
  # Monitor sin flashear
  idf.py monitor
  ```

- **Mantener el sistema:**
  - Recalibrar mensualmente con 1 punto (pH 6.86)
  - Recalibración completa trimestral
  - Backup de configuración periódico

---

## 🆘 Soporte

**Documentación:** README.md, docs/guides/CALIBRATION.md  
**Issues:** GitHub Issues  
**Autor:** [@Marton1123](https://github.com/Marton1123)  
**Repositorio:** [Biofloc-Firmware-ROS](https://github.com/Marton1123/Biofloc-Firmware-ROS)  
**Licencia:** MIT  

---

**¡Éxito con tu sistema de telemetría!** 🌊
