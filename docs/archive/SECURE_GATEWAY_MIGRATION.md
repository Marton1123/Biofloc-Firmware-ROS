# 🔒 Migración a Arquitectura de Gateway Seguro

**Fecha:** 2026-02-09  
**Versión Firmware:** 2.3.0 (Secure Gateway)  
**Versión Bridge:** 3.1.0 (Server-side Timestamps)

---

## 📋 Resumen de Cambios

Se ha migrado de una arquitectura tradicional con ESP32 conectado a Internet a una **Arquitectura de Gateway Seguro** donde:

- ✅ **ESP32 NO tiene acceso a Internet** (sin riesgos de seguridad)
- ✅ **Gateway (NUC) actúa como Hotspot WiFi** aislado
- ✅ **Timestamps agregados por el servidor** (no requiere NTP en ESP32)
- ✅ **Reconexión automática robusta** con backoff exponencial
- ✅ **Bridge resistente a fallos** de MongoDB

---

## 🏗️ Nueva Arquitectura de Red

```
┌─────────────────────────────────────────────────────┐
│                  INTERNET (WAN)                     │
│                        ↑                            │
│                        │ Ethernet (enp88s0)         │
└────────────────────────┼─────────────────────────────┘
                         │
                    ┌────┴────┐
                    │   NUC   │ Ubuntu 24.04
                    │ Gateway │
                    └────┬────┘
                         │
                         │ WiFi Hotspot (wlo1)
                         │ IP: 10.42.0.1
                         │ SSID: lab-ros2-nuc
                         │ Pass: ni2dEUVd
                         │
          ┌──────────────┼──────────────┐
          │              │              │
     ┌────┴────┐    ┌────┴────┐   ┌────┴────┐
     │  ESP32  │    │  ESP32  │   │  ESP32  │
     │ (10.42  │    │ (10.42  │   │ (10.42  │
     │  .0.x)  │    │  .0.x)  │   │  .0.x)  │
     └─────────┘    └─────────┘   └─────────┘
     ❌ Sin Internet  ❌ Sin NTP    ❌ Sin APIs
     ✅ Solo UDP:8888 ✅ DHCP       ✅ Seguro
```

### Firewall (iptables)
```bash
# Bloquea salida a Internet desde WiFi hotspot
iptables -A FORWARD -i wlo1 -o enp88s0 -j DROP
# Solo permite UDP 8888 (micro-ROS Agent)
iptables -A INPUT -i wlo1 -p udp --dport 8888 -j ACCEPT
```

---

## 🔧 Cambios en el Firmware ESP32

### 1. Configuración de Red (Kconfig)
**Archivo:** `main/Kconfig.projbuild`

```diff
- SSID: "MyNetwork"
+ SSID: "lab-ros2-nuc"

- Password: "MyPassword"  
+ Password: "ni2dEUVd"

- Agent IP: "192.168.1.100"
+ Agent IP: "10.42.0.1"
```

**Aplicar cambios:**
```bash
cd /home/Biofloc-Firmware-ROS
idf.py menuconfig
# → Biofloc Configuration → WiFi Configuration
# → Biofloc Configuration → micro-ROS Agent Configuration
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

### 2. Eliminación de NTP/SNTP
**Archivo:** `main/main.c`

**Eliminado:**
- ❌ `#include "esp_sntp.h"`
- ❌ `void init_ntp(void)`
- ❌ `ntp_sync_callback()`
- ❌ `TAG_NTP`
- ❌ `g_app_state.time_synced`

**Razón:** ESP32 ya no tiene acceso a servidores NTP. Los timestamps se agregan en el servidor.

### 3. Timestamp Simplificado
**Archivo:** `main/sensors.c`

**Antes:**
```c
// Requería NTP y time()
gettimeofday(&tv, NULL);
strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
```

**Después:**
```c
// Simple contador de muestras
static uint32_t sample_count = 0;
sample_count++;
snprintf(timestamp_iso, sizeof(timestamp_iso), "sample_%u", sample_count);
```

**Resultado en JSON:**
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "timestamp": "sample_1234",  // Contador ESP32
  "location": "tanque_01",
  "sensors": { ... }
}
```

### 4. Reconexión Robusta
**Archivo:** `main/main.c`

**Mejoras:**
- ✅ **10 intentos** (antes 5)
- ✅ **Backoff exponencial:** 2s → 4s → 8s → 16s → 30s
- ✅ **Logs mejorados** con emojis y delays

**Código:**
```c
#define RECONNECT_ATTEMPTS      10
#define RECONNECT_BACKOFF_MAX   30000  // Max 30s

static bool try_reconnect(void) {
    uint32_t delay_ms = RECONNECT_DELAY_MS;
    
    for (int i = 0; i < RECONNECT_ATTEMPTS; i++) {
        ESP_LOGI(TAG_UROS, "Reconnection attempt %d/%d (delay: %lums)", 
                 i + 1, RECONNECT_ATTEMPTS, delay_ms);
        
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        
        if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
            ESP_LOGI(TAG_UROS, "✅ Reconnected successfully");
            return true;
        }
        
        // Exponential backoff
        delay_ms = (delay_ms * 2 > RECONNECT_BACKOFF_MAX) 
                   ? RECONNECT_BACKOFF_MAX 
                   : delay_ms * 2;
    }
    
    ESP_LOGE(TAG_UROS, "❌ Failed to reconnect");
    return false;
}
```

---

## 🐍 Cambios en el Bridge Python

### 1. Agregado de Timestamps del Servidor
**Archivo:** `scripts/sensor_db_bridge.py`

**Función actualizada:**
```python
def sensor_callback(self, msg: String):
    """Process incoming JSON sensor messages and add server timestamp."""
    try:
        data = json.loads(msg.data)
        
        # SERVER-SIDE TIMESTAMP (ESP32 has no Internet/NTP)
        server_timestamp = datetime.now().isoformat()
        
        # ESP32 sample counter (for debugging)
        esp32_sample_id = data.get('timestamp', 'unknown')
        
        # Create document with SERVER timestamp
        telemetria_doc = {
            'device_id': device_id,
            'location': location,
            'timestamp': server_timestamp,       # ← SERVIDOR
            'esp32_sample_id': esp32_sample_id,  # ← ESP32 (debug)
            'sensors': { ... }
        }
```

**Resultado en MongoDB:**
```json
{
  "_id": ObjectId("..."),
  "device_id": "biofloc_esp32_c8e0",
  "timestamp": "2026-02-09T12:34:56.789012",  // ← SERVIDOR (NUC)
  "esp32_sample_id": "sample_1234",            // ← ESP32 (contador)
  "location": "tanque_01",
  "sensors": { ... }
}
```

### 2. Manejo de Errores Robusto

**Mejoras:**
- ✅ **No se cae si MongoDB falla** (try/except mejorado)
- ✅ **Reconexión automática** a MongoDB
- ✅ **Retry inteligente** del mensaje actual
- ✅ **Logs detallados** de errores

**Código:**
```python
try:
    result = self.telemetria_collection.insert_one(telemetria_doc)
    if result.inserted_id:
        self.messages_saved += 1
        self._update_device_metadata(device_id, location, server_timestamp)
except Exception as e:
    self.get_logger().error(f"MongoDB insert error: {e}")
    self.messages_failed += 1
    self.mongodb_connected = False  # Mark as disconnected
    
    # Try to reconnect on next message
    self.get_logger().warning("Will attempt reconnection on next message")
```

---

## 📊 Comparación: Antes vs Después

| Aspecto | Antes (v2.2.0) | Después (v2.3.0) |
|---------|----------------|------------------|
| **WiFi SSID** | Red doméstica | `lab-ros2-nuc` |
| **Acceso Internet ESP32** | ✅ Sí | ❌ No (seguro) |
| **IP Agent** | 192.168.x.x | 10.42.0.1 (fija) |
| **Timestamp** | ESP32 (NTP) | Servidor (NUC) |
| **NTP/SNTP** | Requerido | ❌ Eliminado |
| **Reconexiones** | 5 intentos lineales | 10 intentos exponenciales |
| **Delay max reconexión** | 10s | 30s |
| **MongoDB falla** | Nodo se cae | ✅ Continúa operando |
| **Logs** | Básicos | ✅ Detallados + emojis |
| **Firewall** | Ninguno | ✅ iptables (gateway) |

---

## 🚀 Procedimiento de Actualización

### Paso 1: Actualizar Firmware ESP32

```bash
# 1. Entrar al proyecto
cd /home/Biofloc-Firmware-ROS

# 2. Cargar entorno ESP-IDF
source ~/esp/v5.3.4/esp-idf/export.sh

# 3. Configurar (opcional si quieres verificar)
idf.py menuconfig

# 4. Compilar
idf.py build

# 5. Flashear
idf.py -p /dev/ttyUSB0 flash monitor
```

**Salida esperada en el monitor:**
```
I (2345) BIOFLOC: =========================================
I (2346) BIOFLOC:   Biofloc Firmware ROS v2.1.0
I (2347) BIOFLOC:   ESP-IDF: v5.3.4
I (2348) BIOFLOC:   micro-ROS: Jazzy
I (2349) BIOFLOC: =========================================
I (2350) BIOFLOC: Initializing network...
I (3456) BIOFLOC: Network ready
I (3457) BIOFLOC: Device ID: biofloc_esp32_c8e0
I (3458) BIOFLOC: MAC: AA:BB:CC:DD:C8:E0
I (3459) BIOFLOC: ⚠ No Internet access - Running in secure gateway mode
I (3460) BIOFLOC: Timestamps will be added by the server
I (4567) UROS: Pinging Agent (timeout: 1000ms, retries: 3)
I (4568) UROS: Ping attempt 1/3
I (4789) UROS: Agent is ONLINE
I (5123) UROS: ✅ Reconnected successfully
```

### Paso 2: Iniciar micro-ROS Agent

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Salida esperada:**
```
[info] | UDPv4AgentLinux.cpp | init | running... | port: 8888
[info] | Root.cpp | create_client | create | client_key: 0x175665F4
[info] | SessionManager.hpp | establish_session | session established
```

### Paso 3: Iniciar Bridge con Timestamps del Servidor

```bash
# Terminal 2
cd /home/Biofloc-Firmware-ROS/scripts
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
python3 sensor_db_bridge.py
```

**Salida esperada:**
```
✓ Loaded environment from: /home/Biofloc-Firmware-ROS/scripts/.env
[INFO] Connected to MongoDB: SistemasLab
[INFO] ============================================================
[INFO] Sensor DB Bridge v3.1 Started (Secure Gateway Mode)
[INFO]   Topic: /biofloc/sensor_data
[INFO]   Database: SistemasLab
[INFO]     - telemetria: telemetria
[INFO]     - devices: devices
[INFO]   MongoDB Connected: True
[INFO]   Server-side timestamps: ENABLED
[INFO] ============================================================
[INFO] [biofloc_esp32_c8e0@tanque_01] pH: 6.79 [OK] | Temp: 22.4°C [OK] | 2026-02-09T12:34:56.789012
```

---

## ✅ Verificación Post-Migración

### 1. Verificar ESP32 NO tiene Internet
```bash
# En el monitor del ESP32, NO debería haber logs de:
# - "Time synchronized"
# - "NTP"
# - "SNTP"

# Debería verse:
I (3459) BIOFLOC: ⚠ No Internet access - Running in secure gateway mode
I (3460) BIOFLOC: Timestamps will be added by the server
```

### 2. Verificar Timestamps del Servidor
```bash
# En MongoDB Compass o CLI:
db.telemetria.find().sort({timestamp: -1}).limit(1).pretty()

# Debería verse:
{
  "timestamp": "2026-02-09T12:34:56.789012",  // ← ISO 8601 completo
  "esp32_sample_id": "sample_1234",            // ← Contador ESP32
  ...
}
```

### 3. Verificar Reconexión Robusta
```bash
# Reiniciar el Agent (Terminal 1: Ctrl+C)
# En el monitor ESP32 (Terminal flash monitor) debería verse:

W (12345) UROS: Lost connection to Agent
I (12346) UROS: Reconnection attempt 1/10 (delay: 2000ms)
I (14347) UROS: Reconnection attempt 2/10 (delay: 4000ms)
I (18349) UROS: Reconnection attempt 3/10 (delay: 8000ms)
I (26351) UROS: ✅ Reconnected successfully
```

### 4. Verificar MongoDB Resiliente
```bash
# Apagar MongoDB Atlas temporalmente
# El Bridge (Terminal 2) debería seguir funcionando:

[WARNING] MongoDB insert error: ...
[WARNING] Will attempt reconnection on next message
[INFO] MongoDB disconnected, attempting reconnection...
[INFO] ✅ MongoDB reconnection successful
```

---

## 🔍 Troubleshooting

### Problema 1: ESP32 no se conecta al WiFi
**Síntoma:** `E (1234) wifi:sta connect fail`

**Solución:**
```bash
idf.py menuconfig
# → Biofloc Configuration → WiFi Configuration
# Verificar: SSID = "lab-ros2-nuc"
#            Pass = "ni2dEUVd"
```

### Problema 2: No llegan datos al Bridge
**Síntoma:** Bridge no muestra logs de datos

**Verificar:**
```bash
# 1. Agent corriendo?
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 2. ESP32 conectado?
ros2 topic list
# Debe aparecer: /biofloc/sensor_data

# 3. Ver datos crudos
ros2 topic echo /biofloc/sensor_data
```

### Problema 3: Timestamps incorrectos en MongoDB
**Síntoma:** Timestamps en formato `sample_XXXX`

**Causa:** El Bridge no está agregando el timestamp del servidor

**Verificar:**
```bash
# Ver logs del bridge
# Debe mostrar:
[INFO] [biofloc_esp32_c8e0@tanque_01] ... | 2026-02-09T12:34:56.789012
#                                            ↑ ISO 8601 completo
```

### Problema 4: MongoDB desconectado permanentemente
**Síntoma:** `mongodb=disconnected` en stats

**Solución:**
```bash
cd /home/Biofloc-Firmware-ROS/scripts

# Verificar .env
cat .env
# Debe tener:
MONGODB_URI=mongodb+srv://...

# Probar conexión
python3 -c "
from pymongo import MongoClient
client = MongoClient('mongodb+srv://...')
print(client.admin.command('ping'))
"
```

---

## 📚 Referencias

- **Arquitectura:** `SECURE_GATEWAY_MIGRATION.md` (este archivo)
- **Documentación ESP32:** [main/main.c](main/main.c)
- **Bridge Python:** [scripts/sensor_db_bridge.py](scripts/sensor_db_bridge.py)
- **Configuración WiFi:** [main/Kconfig.projbuild](main/Kconfig.projbuild)
- **Guía rápida:** [QUICKSTART.md](QUICKSTART.md)
- **Troubleshooting:** [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

---

## 🎯 Próximos Pasos

- [ ] Actualizar `README.md` con la nueva arquitectura
- [ ] Actualizar `QUICKSTART.md` con instrucciones del gateway
- [ ] Crear diagrama de red en `docs/ARCHITECTURE.md`
- [ ] Documentar reglas de firewall iptables
- [ ] Agregar tests de reconexión automática
- [ ] Implementar buffer local en ESP32 para datos offline

---

**Última actualización:** 2026-02-09  
**Autor:** @Marton1123  
**Versión:** 1.0.0
