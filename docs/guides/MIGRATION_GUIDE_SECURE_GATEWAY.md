# Guía de Migración: Arquitectura Antigua → Gateway Seguro IoT

## Resumen de la Migración

**ANTES (Arquitectura Antigua):**
- ESP32 conectado a WiFi con internet directo
- ESP32 puede acceder a servicios cloud (NTP, APIs, etc.)
- Agente micro-ROS puede estar en cualquier máquina de la red
- MongoDB/servicios pueden ser accedidos directamente

**DESPUÉS (Arquitectura Segura con Gateway):**
- ESP32 conectado a WiFi SIN internet (hotspot del gateway)
- ESP32 SOLO puede comunicarse con el gateway
- Gateway tiene doble conexión: WiFi (para ESP32) + Ethernet (para internet)
- Firewall iptables bloquea ESP32 → Internet, solo permite UDP al agente

## Arquitectura de Red Nueva

```
Internet
   |
   | (Ethernet - enp88s0 o similar)
   |
[GATEWAY - Raspberry Pi 3 / Intel NUC / PC Linux]
   | - Ubuntu Server 24.04+ (ARM64 para RPi3)
   | - iptables FORWARD DROP (firewall)
   | - Agente micro-ROS en UDP 8888
   | - Scripts Python (bridge, monitor)
   | - MongoDB cliente (conecta a cloud)
   |
   | (WiFi - wlan0 en RPi3, wlo1 en NUC - Hotspot)
   | Red: 10.42.0.0/24
   | Gateway: 10.42.0.1
   |
[ESP32]
   | IP: 10.42.0.x (asignada por DHCP)
   | SIN acceso a internet
   | Solo UDP a 10.42.0.1:8888
```

## 1. Configuración del Gateway (Hacer PRIMERO)

### 1.1. Crear Hotspot WiFi (NetworkManager)

```bash
# Crear hotspot en interfaz WiFi (wlan0 en RPi3, wlo1 en NUC)
nmcli device wifi hotspot \
  ifname wlan0 \
  ssid "tu-gateway-iot" \
  password "tu-password-seguro"

# Verificar que se creó
nmcli connection show
# Debe aparecer algo como "Hotspot" o el nombre del SSID

# Configurar para que inicie automáticamente
nmcli connection modify Hotspot connection.autoconnect yes
```

**Verificar red creada:**
```bash
ip addr show wlo1
# Debe mostrar algo como: inet 10.42.0.1/24
```

### 1.2. Configurar Firewall iptables

**Script de firewall** (guardar como `/home/tu-usuario/setup_iot_firewall.sh`):

```bash
#!/bin/bash
# Configuración de firewall para Gateway IoT Seguro

echo "Configurando firewall para Gateway IoT..."

# Limpiar reglas previas
iptables -F
iptables -t nat -F
iptables -X

# Políticas por defecto
iptables -P INPUT ACCEPT
iptables -P OUTPUT ACCEPT
iptables -P FORWARD DROP  # ← CRÍTICO: bloquea todo forwarding por defecto

# Permitir loopback
iptables -A INPUT -i lo -j ACCEPT
iptables -A OUTPUT -o lo -j ACCEPT

# Permitir conexiones establecidas
iptables -A FORWARD -m state --state ESTABLISHED,RELATED -j ACCEPT

# PERMITIR: ESP32 → Gateway (UDP 8888 para micro-ROS)
iptables -A FORWARD -i wlo1 -o lo -p udp --dport 8888 -j ACCEPT
iptables -A INPUT -i wlo1 -p udp --dport 8888 -j ACCEPT

# BLOQUEAR: ESP32 → Internet (ya bloqueado por FORWARD DROP)
# No se necesita regla explícita, DROP por defecto lo maneja

# Permitir NAT para el gateway mismo (no para forwarding)
iptables -t nat -A POSTROUTING -o enp88s0 -j MASQUERADE

echo "Firewall configurado correctamente"
echo "- ESP32 puede enviar UDP a 10.42.0.1:8888"
echo "- ESP32 NO puede acceder a internet"
echo ""
echo "Verificar con: iptables -L -v -n"
```

**Hacer ejecutable y correr:**
```bash
chmod +x setup_iot_firewall.sh
sudo ./setup_iot_firewall.sh
```

**Hacer persistente (Ubuntu/Debian):**
```bash
sudo apt install iptables-persistent
sudo netfilter-persistent save
```

### 1.3. Verificar Gateway

```bash
# 1. Verificar hotspot activo
nmcli connection show --active | grep -i hotspot

# 2. Verificar IP del gateway
ip addr show wlo1 | grep "inet "
# Debe mostrar: inet 10.42.0.1/24

# 3. Verificar firewall
sudo iptables -L FORWARD -v -n
# Debe mostrar: Chain FORWARD (policy DROP ...)

# 4. Verificar que gateway tiene internet
ping -c 3 8.8.8.8
```

## 2. Cambios en el Código del ESP32 (Firmware)

### 2.1. WiFi: Cambiar Credenciales

**BUSCAR en tu código:**
- Archivos: `main.c`, `wifi.c`, `sdkconfig`, `sdkconfig.defaults`, `Kconfig.projbuild`
- Variables: `WIFI_SSID`, `WIFI_PASSWORD`, `CONFIG_ESP_WIFI_SSID`, etc.

**ANTES:**
```c
#define WIFI_SSID "tu-red-con-internet"
#define WIFI_PASSWORD "password-red-normal"
```

**DESPUÉS:**
```c
#define WIFI_SSID "tu-gateway-iot"  // ← SSID del hotspot del gateway
#define WIFI_PASSWORD "tu-password-seguro"
```

**Si usas sdkconfig.defaults:**
```
CONFIG_ESP_WIFI_SSID="tu-gateway-iot"
CONFIG_ESP_WIFI_PASSWORD="tu-password-seguro"
```

**⚠️ IMPORTANTE para micro-ROS:** Si tu proyecto usa micro-ROS, pueden existir DOS sets de credenciales WiFi:
- `CONFIG_ESP_WIFI_SSID` / `CONFIG_ESP_WIFI_PASSWORD` (usado por el componente micro_ros)
- `CONFIG_TU_APP_WIFI_SSID` / `CONFIG_TU_APP_WIFI_PASSWORD` (usado por tu aplicación)

**AMBOS deben cambiarse** para que la conexión funcione correctamente.

### 2.2. Micro-ROS: Cambiar IP del Agente

**BUSCAR en tu código:**
- Archivos: `main.c`, `ros_config.c`, configuraciones de micro-ROS
- Variables: `AGENT_IP`, `RMW_AGENT_ADDRESS`, hardcoded IPs

**ANTES:**
```c
#define AGENT_IP "192.168.1.100"  // IP en red externa
// o
#define AGENT_IP "tu-servidor.com"
```

**DESPUÉS:**
```c
#define AGENT_IP "10.42.0.1"  // IP del gateway en la red interna
```

**Si usas variables de entorno ESP-IDF:**
```c
// sdkconfig.defaults
CONFIG_MICRO_ROS_AGENT_IP="10.42.0.1"
CONFIG_MICRO_ROS_AGENT_PORT=8888
```

**Puerto:** Mantener `8888` (estándar micro-ROS UDP)

### 2.3. NTP: Eliminar Completamente

El ESP32 **NO tiene internet**, por lo tanto **NO puede usar NTP**.

**BUSCAR y ELIMINAR:**
- Archivos: `main.c`, `time.c`, `ntp.c`, `sntp_*.c`
- Funciones: `sntp_init()`, `sntp_setservername()`, `esp_sntp_*`
- Includes: `#include "esp_sntp.h"`, `#include "lwip/apps/sntp.h"`

**ANTES:**
```c
#include "esp_sntp.h"

void init_time() {
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    
    // Esperar sincronización
    time_t now = 0;
    while (now < 1000000000) {
        time(&now);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void send_data() {
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    // Enviar con timestamp...
}
```

**DESPUÉS (Opción A - Contador Simple):**
```c
// NO incluir esp_sntp.h

static uint32_t sample_counter = 0;

void send_data() {
    char timestamp[32];
    snprintf(timestamp, sizeof(timestamp), "sample_%lu", sample_counter++);
    // Enviar con timestamp de contador...
}
```

**DESPUÉS (Opción B - Millis desde Boot):**
```c
#include "esp_timer.h"

void send_data() {
    int64_t uptime_ms = esp_timer_get_time() / 1000;  // microsegundos → ms
    
    char timestamp[32];
    snprintf(timestamp, sizeof(timestamp), "%lld", uptime_ms);
    // Enviar con timestamp de uptime...
}
```

**⚠️ IMPORTANTE:** Los timestamps reales se agregarán en el **servidor** (gateway), no en el ESP32.

### 2.4. Manejo de Errores WiFi

Sin internet, el ESP32 no debe intentar conectarse a servicios externos.

**BUSCAR:**
- Intentos de HTTP/HTTPS
- Llamadas a APIs externas
- Verificaciones de conectividad a internet

**MODIFICAR:**
```c
// ANTES: Verificar conectividad a internet
esp_err_t check_internet() {
    esp_http_client_config_t config = {
        .url = "http://www.google.com",
    };
    // ... intenta HTTP GET
}

// DESPUÉS: Solo verificar WiFi local
esp_err_t check_wifi() {
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Conectado a: %s, RSSI: %d", 
                 ap_info.ssid, ap_info.rssi);
        return ESP_OK;
    }
    return ESP_FAIL;
}
```

### 2.5. Reconexión WiFi

**Agregar backoff exponencial** para reconexiones sin bombardear el gateway:

```c
#define MAX_RETRY 15
#define RETRY_BASE_DELAY_MS 1000
#define RETRY_MAX_DELAY_MS 60000

static int s_retry_num = 0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            // Backoff exponencial
            uint32_t delay = RETRY_BASE_DELAY_MS * (1 << s_retry_num);
            if (delay > RETRY_MAX_DELAY_MS) {
                delay = RETRY_MAX_DELAY_MS;
            }
            
            ESP_LOGI(TAG, "Reintentando conexión en %lu ms (intento %d/%d)",
                     delay, s_retry_num + 1, MAX_RETRY);
            
            vTaskDelay(pdMS_TO_TICKS(delay));
            esp_wifi_connect();
            s_retry_num++;
        } else {
            ESP_LOGI(TAG, "Máximo de reintentos alcanzado, reiniciando...");
            esp_restart();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;  // Reset en conexión exitosa
        ESP_LOGI(TAG, "Conectado exitosamente al gateway");
    }
}
```

## 3. Cambios en el Código del Gateway (Scripts Python/ROS)

### 3.1. Timestamps: Agregar en Servidor

**Si tienes un script que guarda datos a MongoDB:**

**ANTES (recibe timestamp del ESP32):**
```python
def callback(msg):
    data = {
        'device_id': msg.device_id,
        'ph': msg.ph,
        'temperature': msg.temperature,
        'timestamp': msg.timestamp  # ← viene del ESP32 (posiblemente incorrecto)
    }
    collection.insert_one(data)
```

**DESPUÉS (genera timestamp en servidor):**
```python
from datetime import datetime

def callback(msg):
    data = {
        'device_id': msg.device_id,
        'ph': msg.ph,
        'temperature': msg.temperature,
        'timestamp_esp32': msg.timestamp,  # ← guardar por referencia
        'timestamp': datetime.utcnow().isoformat()  # ← timestamp real del servidor
    }
    collection.insert_one(data)
```

### 3.2. Manejo de Conexión MongoDB

El gateway SÍ tiene internet, pero debe manejar desconexiones:

```python
from pymongo import MongoClient
from pymongo.errors import ConnectionFailure, ServerSelectionTimeoutError
import time

def connect_to_mongodb():
    max_retries = 5
    retry_delay = 5
    
    for attempt in range(max_retries):
        try:
            client = MongoClient(
                mongo_uri,
                serverSelectionTimeoutMS=5000,
                connectTimeoutMS=10000
            )
            # Verificar conexión
            client.admin.command('ping')
            print(f"✓ Conectado a MongoDB")
            return client
            
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            print(f"✗ Error MongoDB (intento {attempt+1}/{max_retries}): {e}")
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
            else:
                print("⚠ Continuando sin MongoDB (modo offline)")
                return None
    
    return None

# Uso en callback
def callback(msg):
    if mongo_client is not None:
        try:
            collection.insert_one(data)
        except Exception as e:
            print(f"⚠ Error guardando a MongoDB: {e}")
    else:
        print("⚠ MongoDB no disponible, datos no guardados")
```

## 4. Comandos para Verificar Conectividad

### 4.1. Desde el Gateway

**Verificar que ESP32 está conectado al WiFi:**
```bash
# Ver leases DHCP
cat /var/lib/NetworkManager/dnsmasq-wlo1.leases
# Buscar la MAC del ESP32

# Ver tabla ARP (dispositivos conectados)
ip neigh show dev wlo1
# Buscar la IP del ESP32 (ej: 10.42.0.123)

# Hacer ping al ESP32
ping -c 3 10.42.0.123
```

**Verificar comunicación ROS:**
```bash
# Listar topics (debe aparecer el del ESP32)
source /opt/ros/jazzy/setup.bash  # o tu versión de ROS
ros2 topic list | grep tu_topic

# Ver mensajes (timeout 8s para verificación rápida)
timeout 8 ros2 topic echo /tu_topic --once

# Ver tasa de publicación
timeout 20 ros2 topic hz /tu_topic
```

**Verificar firewall:**
```bash
# Verificar política FORWARD DROP
sudo iptables -L FORWARD -v -n | head -5
# Debe mostrar: Chain FORWARD (policy DROP)

# Intentar ping desde ESP32 a internet (debe fallar)
# En el gateway, monitorear:
sudo tcpdump -i wlo1 icmp
# Flashear ESP32 con código que haga ping a 8.8.8.8
# NO deben verse paquetes saliendo por enp88s0
```

### 4.2. Desde el ESP32 (para debugging)

**Agregar comandos de diagnóstico** (solo para desarrollo):

```c
void print_network_info() {
    tcpip_adapter_ip_info_t ip_info;
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
    
    ESP_LOGI(TAG, "=== Network Info ===");
    ESP_LOGI(TAG, "IP:      " IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip_info.gw));
    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&ip_info.netmask));
    
    // Verificar que gateway es 10.42.0.1
    if (ip_info.gw.addr != ipaddr_addr("10.42.0.1")) {
        ESP_LOGW(TAG, "⚠ Gateway NO es 10.42.0.1!");
    }
}
```

## 5. Checklist de Migración

### Antes de Empezar:
- [ ] Hacer backup completo del proyecto
- [ ] Documentar configuración actual (IPs, puertos, credenciales)
- [ ] Tomar nota de todos los servicios externos que usa el ESP32

### Gateway:
- [ ] Ubuntu 24.04+ instalado
- [ ] Dos interfaces de red: WiFi + Ethernet
- [ ] Hotspot WiFi creado (NetworkManager)
- [ ] Verificado que gateway tiene IP 10.42.0.1
- [ ] iptables configurado (FORWARD DROP)
- [ ] iptables persistente (iptables-persistent)
- [ ] Firewall verificado (ESP32 NO llega a internet)
- [ ] Agente micro-ROS instalado
- [ ] ROS 2 instalado y configurado

### Código ESP32:
- [ ] WiFi SSID/password cambiado a hotspot del gateway
- [ ] Dual WiFi credentials actualizadas (si aplica micro-ROS)
- [ ] IP del agente micro-ROS = 10.42.0.1
- [ ] Puerto del agente = 8888
- [ ] NTP eliminado completamente
- [ ] Timestamps cambiados a contador/uptime
- [ ] Llamadas HTTP/HTTPS eliminadas
- [ ] Verificaciones de internet eliminadas
- [ ] Backoff exponencial en reconexión WiFi
- [ ] Logs de diagnóstico agregados

### Scripts del Gateway:
- [ ] Timestamps agregados en servidor
- [ ] Manejo robusto de MongoDB (try-except)
- [ ] Variables de entorno para credenciales
- [ ] Scripts de verificación creados
- [ ] Logs configurados correctamente

### Testing:
- [ ] ESP32 se conecta al hotspot
- [ ] ESP32 recibe IP 10.42.0.x
- [ ] ESP32 NO puede hacer ping a 8.8.8.8
- [ ] ESP32 envía datos al agente micro-ROS
- [ ] Agente recibe datos del ESP32
- [ ] Topics ROS visibles en gateway
- [ ] Bridge guarda datos a MongoDB
- [ ] Timestamps correctos en MongoDB

### Documentación:
- [ ] Diagrama de red actualizado
- [ ] IPs documentadas (gateway, ESP32)
- [ ] Credenciales documentadas (WiFi, MongoDB)
- [ ] Procedimientos de troubleshooting
- [ ] Comandos de verificación listados

## 6. Troubleshooting Común

### Problema: ESP32 no se conecta al hotspot

**Verificar:**
```bash
# En gateway, ver si ESP32 intenta conectarse
sudo journalctl -u NetworkManager -f
# Debe mostrar intentos de conexión

# Verificar que hotspot está activo
nmcli connection show --active | grep -i hotspot

# Ver configuración del hotspot
nmcli connection show Hotspot | grep -E "ssid|password|ipv4"
```

**Solución:**
- Verificar SSID/password en código ESP32
- Verificar que hotspot usa 2.4GHz (ESP32 no soporta 5GHz)
- Reiniciar hotspot: `nmcli connection down Hotspot && nmcli connection up Hotspot`

### Problema: ESP32 conectado pero no publica datos

**Verificar:**
```bash
# Ver si agente está corriendo
ps aux | grep micro_ros_agent

# Iniciar agente si no está corriendo
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

**Solución:**
- Verificar IP del agente en ESP32 = 10.42.0.1
- Verificar puerto = 8888
- Verificar firewall permite UDP 8888: `sudo iptables -L INPUT -v -n | grep 8888`

### Problema: Datos llegan a ROS pero no a MongoDB

**Verificar:**
```bash
# Ver si bridge está corriendo
ps aux | grep sensor_db_bridge

# Ver logs del bridge
journalctl -u tu-bridge-service -f
```

**Solución:**
- Verificar que gateway tiene internet: `ping -c 3 8.8.8.8`
- Verificar credenciales MongoDB en `.env`
- Verificar que bridge subscribe al topic correcto

### Problema: ESP32 SÍ puede acceder a internet (security issue!)

**Verificar:**
```bash
# Ver política de FORWARD
sudo iptables -L FORWARD -v -n
# DEBE mostrar: Chain FORWARD (policy DROP)

# Ver todas las reglas de FORWARD
sudo iptables -L FORWARD -v -n --line-numbers

# Buscar reglas permisivas
sudo iptables -S FORWARD | grep ACCEPT
```

**Solución:**
```bash
# Limpiar y reconfigurar
sudo iptables -P FORWARD DROP
sudo iptables -F FORWARD

# Agregar SOLO las reglas necesarias
sudo iptables -A FORWARD -m state --state ESTABLISHED,RELATED -j ACCEPT
sudo iptables -A INPUT -i wlo1 -p udp --dport 8888 -j ACCEPT

# Guardar
sudo netfilter-persistent save
```

## 7. Ventajas de la Nueva Arquitectura

✅ **Seguridad:** ESP32 no puede ser comprometido desde internet  
✅ **Control:** Todo el tráfico pasa por el gateway (auditable)  
✅ **Confiabilidad:** Timestamps correctos (del servidor, no del ESP32)  
✅ **Mantenibilidad:** Cambios en servicios cloud no requieren reflashear ESP32  
✅ **Escalabilidad:** Múltiples ESP32 pueden conectarse al mismo gateway  
✅ **Debugging:** Gateway puede monitorear/interceptar todo el tráfico  
✅ **Offline:** Sistema funciona aunque MongoDB esté caído

## 8. Para Pasar a una IA de Código

Si vas a usar una IA para hacer estos cambios, dale este prompt:

```
Necesito migrar mi proyecto ESP32 + micro-ROS de arquitectura antigua (WiFi con internet directo) 
a arquitectura segura con gateway IoT (ESP32 sin internet).

ARQUITECTURA NUEVA:
- Gateway Linux con doble red: WiFi hotspot (10.42.0.1/24) para ESP32 + Ethernet para internet
- ESP32 conecta a hotspot (SSID: "tu-gateway-iot")
- ESP32 SIN acceso a internet (bloqueado por iptables)
- Agente micro-ROS en gateway (10.42.0.1:8888)

CAMBIOS NECESARIOS:
1. WiFi: Cambiar SSID/password a los del hotspot del gateway
2. Agente: Cambiar IP del agente micro-ROS a 10.42.0.1
3. NTP: Eliminar completamente (sin internet)
4. Timestamps: Usar contador o uptime en ESP32, timestamps reales en servidor
5. HTTP/APIs: Eliminar llamadas a servicios externos
6. Reconexión: Implementar backoff exponencial

Analiza mi código y lista todos los cambios específicos necesarios, con referencias a archivos 
y líneas exactas. Luego implementa los cambios manteniendo la funcionalidad existente.
```

---

## Contacto y Soporte

Para dudas sobre esta migración:
- Revisar logs del gateway: `journalctl -xe`
- Revisar logs del ESP32: Monitor serial + `idf.py monitor`
- Verificar firewall: `sudo iptables -L -v -n`
- Verificar conectividad: Scripts de este documento

**¡Buena suerte con la migración!** 🚀
