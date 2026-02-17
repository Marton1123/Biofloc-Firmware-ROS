# Deployment Guide - Firmware v3.2.0 (Telemetría Embebida)

**Fecha:** 2026-02-17  
**Versión:** 3.2.0 - Arquitectura de Auto-Recuperación y Diagnóstico Remoto

---

## 🎯 Resumen de Cambios

### Problema Resuelto
- **ESP32 se congelaba (zombie state)** tras ~2 horas de operación continua
- Sin acceso físico al monitor serial (caja eléctrica cerrada)
- Imposible diagnosticar memory leaks o causas de reinicio remotamente

### Solución Implementada
1. **Hardware Task Watchdog Timer**: Reset automático si el sistema se bloquea > 20s
2. **Telemetría Embebida**: Monitoreo de RAM, uptime y reset_reason sin tópicos extra
3. **Enrutamiento Inteligente**: Bridge separa datos biológicos de telemetría del sistema
4. **Memory Leak Prevention**: Código revisado y validado

---

## 🚀 Pasos de Deployment

### 1️⃣ Compilar Firmware v3.2.0 (En NUC/PC con ESP-IDF)

```bash
cd /home/Biofloc-Firmware-ROS

# Limpiar build anterior (opcional pero recomendado)
idf.py fullclean

# Compilar firmware v3.2.0
idf.py build

# Verificar que la compilación fue exitosa
# Output esperado: "Project build complete. To flash, run: idf.py flash"
```

**Verificaciones**:
- ✅ Versión: `Biofloc Firmware ROS v3.2.0`
- ✅ Tamaño esperado: ~810-850 KB
- ✅ No debe haber errores de compilación

---

### 2️⃣ Flashear ESP32 (Conexión USB Temporal)

**IMPORTANTE**: Desconectar alimentación externa antes de conectar USB.

```bash
# Conectar ESP32 via USB (/dev/ttyUSB0)
idf.py -p /dev/ttyUSB0 flash

# Monitorear logs por 30 segundos para verificar:
idf.py -p /dev/ttyUSB0 monitor
```

**Logs Esperados** (primeros 10 segundos):
```
I (234) BIOFLOC: =========================================
I (238) BIOFLOC:   Biofloc Firmware ROS v3.2.0
I (242) BIOFLOC:   ESP-IDF: v5.3.4
I (246) BIOFLOC:   micro-ROS: Jazzy
I (250) BIOFLOC: =========================================
I (254) BIOFLOC: Reset reason: POWER_ON
I (258) BIOFLOC: Initializing hardware watchdog (timeout: 20s)
I (264) BIOFLOC: ✓ Watchdog initialized - will hard reset if task blocks > 20s
I (275) BIOFLOC: Initializing network...
I (3234) BIOFLOC: Network ready
I (3238) BIOFLOC: Device ID: biofloc_esp32_c8e0
I (3242) BIOFLOC: MAC: 24:0A:C4:60:C8:E0
I (3246) SENSOR: Sensor task started
I (3250) SENSOR: Subscribing to watchdog (timeout: 20s)
I (3256) SENSOR: ✓ Watchdog subscribed - will reset if blocked > 20s
```

**Verificar**:
- ✅ Watchdog inicializado correctamente
- ✅ Reset reason mostrado (probablemente `POWER_ON` en el primer boot)
- ✅ Sensor task subscrita al watchdog
- ✅ WiFi conectado (IP: 10.42.0.123)
- ✅ micro-ROS conectado al Agent

**Presionar Ctrl+] para salir del monitor**

---

### 3️⃣ Reconectar a Alimentación Externa

1. Desconectar cable USB del ESP32
2. Conectar alimentación externa 5V
3. ESP32 debe bootear automáticamente
4. Presionar botón "EN" si es necesario (hard reset)

---

### 4️⃣ Actualizar Bridge Python en Raspberry Pi 4

**Desde la Raspberry Pi Gateway**:

```bash
cd ~/Biofloc-Firmware-ROS

# Pull cambios del repositorio
git pull origin main

# Verificar versión del bridge
grep "Version:" scripts/sensor_db_bridge.py
# Output esperado: Version: 3.1.0 (Three-collection architecture with telemetry routing)

# Reiniciar el servicio del bridge (si está como systemd)
sudo systemctl restart sensor_db_bridge

# O ejecutar manualmente para ver logs en vivo
cd scripts
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
python3 sensor_db_bridge.py
```

**Logs Esperados del Bridge**:
```
[INFO] [sensor_db_bridge]: ============================================================
[INFO] [sensor_db_bridge]: Sensor DB Bridge v3.1 Started
[INFO] [sensor_db_bridge]:   Topic: /biofloc/sensor_data
[INFO] [sensor_db_bridge]:   Database: SistemasLab
[INFO] [sensor_db_bridge]:     - telemetria: telemetria
[INFO] [sensor_db_bridge]:     - devices: devices
[INFO] [sensor_db_bridge]:     - system_health: system_health
[INFO] [sensor_db_bridge]:   MongoDB Connected: True
[INFO] [sensor_db_bridge]: ============================================================
```

**Logs Esperados (Datos con Telemetría)**:
```
[INFO] [sensor_db_bridge]: [biofloc_esp32_c8e0@tanque_01] pH: 4.87 [OK] | Temp: 23.5°C [OK] | Heap: 146.5KB | Uptime: 2.3min | Reset: POWER_ON
[DEBUG] [sensor_db_bridge]: System health saved: biofloc_esp32_c8e0 | heap: 150000 | uptime: 138s | reset: POWER_ON
```

---

### 5️⃣ Verificar MongoDB Atlas

**Colecciones Esperadas**:

#### Colección: `telemetria` (Datos Biológicos LIMPIOS)
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "location": "tanque_01",
  "timestamp": "2026-02-17T14:23:45.123456",
  "esp32_sample_id": "sample_123",
  "sensors": {
    "ph": {
      "value": 4.87,
      "voltage": 1.717,
      "unit": "pH",
      "valid": true
    },
    "temperature": {
      "value": 23.5,
      "voltage": 2.345,
      "unit": "C",
      "valid": true
    }
  },
  "_ros_topic": "/biofloc/sensor_data"
}
```

#### Colección: `system_health` (Telemetría del ESP32) ⭐ **NUEVA**
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "timestamp_gw": "2026-02-17T14:23:45.123456",
  "timestamp_esp32": "sample_123",
  "location": "tanque_01",
  "free_heap": 150000,
  "uptime_sec": 138,
  "reset_reason": "POWER_ON",
  "_ros_topic": "/biofloc/sensor_data"
}
```

#### Colección: `devices` (Metadata)
```json
{
  "_id": "biofloc_esp32_c8e0",
  "alias": "ESP32-c8e0",
  "location": "tanque_01",
  "estado": "activo",
  "firmware_version": "3.2.0",
  "conexion": {
    "primera": "2026-02-17T14:20:00.000000",
    "ultima": "2026-02-17T14:23:45.123456",
    "total_lecturas": 58
  }
}
```

---

## 📊 Monitoreo y Diagnóstico

### Verificar Telemetría en Tiempo Real

**Desde Raspberry Pi**:
```bash
# Ver topic completo con telemetría
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
ros2 topic echo --once --full-length /biofloc/sensor_data
```

**Output Esperado**:
```yaml
data: '{"device_id":"biofloc_esp32_c8e0","timestamp":"sample_456","location":"tanque_01","sensors":{"ph":{"value":4.87,"voltage":1.717,"unit":"pH","valid":true},"temperature":{"value":23.5,"voltage":2.345,"unit":"C","valid":true}},"system":{"free_heap":150000,"uptime_sec":1845,"reset_reason":"POWER_ON"}}'
```

### Consultas MongoDB para Diagnóstico

**1. Verificar Free Heap (Memory Leak Detection)**:
```javascript
// MongoDB Shell o Compass
db.system_health.find(
  { device_id: "biofloc_esp32_c8e0" },
  { timestamp_gw: 1, free_heap: 1, uptime_sec: 1, _id: 0 }
).sort({ timestamp_gw: -1 }).limit(50)
```

**Esperado**: Free heap debe mantenerse estable (~145-155 KB)  
**⚠️ ALERTA**: Si free_heap decrece continuamente → Memory leak detectado

**2. Verificar Causas de Reinicio**:
```javascript
db.system_health.aggregate([
  { $match: { device_id: "biofloc_esp32_c8e0" } },
  { $group: { 
      _id: "$reset_reason", 
      count: { $sum: 1 },
      last_seen: { $max: "$timestamp_gw" }
  }}
])
```

**Posibles valores**:
- `POWER_ON`: Inicio normal (alimentación)
- `TASK_WDT`: **Watchdog detectó congelamiento** ✅ (funcionando correctamente)
- `PANIC`: Crash del sistema (revisar logs)
- `BROWNOUT`: Voltaje insuficiente

**3. Tiempo de Operación Continua (Uptime)**:
```javascript
db.system_health.find(
  { device_id: "biofloc_esp32_c8e0" }
).sort({ uptime_sec: -1 }).limit(1)
```

**Meta**: Uptime > 7200s (2 horas) sin reinicios

---

## 🔧 Troubleshooting

### Problema: Watchdog Resetea el ESP32 Constantemente

**Síntoma**: `reset_reason: "TASK_WDT"` cada ~30 segundos

**Diagnóstico**:
1. Verificar que el Agent esté corriendo en Raspberry Pi
2. Revisar latencia de red (ping 10.42.0.123)
3. Verificar que `SAMPLE_INTERVAL_MS` no sea muy corto

**Solución Temporal**:
```c
// En main/main.c, línea 50, aumentar timeout:
#define WATCHDOG_TIMEOUT_SEC    30  // De 20s a 30s
```

---

### Problema: Free Heap Decrece Continuamente

**Síntoma**: En `system_health`, `free_heap` baja de 150KB → 100KB → 50KB...

**Diagnóstico**: Memory leak confirmado

**Posibles Causas**:
1. cJSON_Delete() faltante en algún path de calibration_callback
2. Buffers no liberados en sensor_task
3. Micro-ROS acumulando mensajes no procesados

**Solución**:
1. Revisar logs de calibración (¿se llamó recientemente?)
2. Flashear versión v3.1.1 (anti-bootloop sin telemetría)
3. Reportar issue en GitHub con dump de `system_health`

---

### Problema: Colección `system_health` No se Crea

**Síntoma**: Bridge corre OK pero MongoDB solo tiene `telemetria` y `devices`

**Diagnóstico**:
```bash
# Verificar logs del bridge
journalctl -u sensor_db_bridge -f
```

**Buscar**: `"System health saved"` en logs

**Solución**:
1. Verificar que el ESP32 esté enviando el objeto `"system"` en JSON
2. Reiniciar bridge después de `git pull`
3. Verificar permisos de escritura en MongoDB Atlas

---

## ✅ Validación Final

**Checklist de Deployment**:

- [ ] Firmware v3.2.0 compilado sin errores
- [ ] ESP32 flasheado y conectado a WiFi (10.42.0.123)
- [ ] Watchdog inicializado (logs: "✓ Watchdog initialized")
- [ ] Bridge Python v3.1.0 actualizado y corriendo
- [ ] Colección `telemetria` recibe datos cada 4 segundos
- [ ] Colección `system_health` recibe telemetría cada 4 segundos
- [ ] Free heap estable (~150 KB) durante 15 minutos
- [ ] Reset reason = "POWER_ON" (o esperado si hubo reinicio manual)
- [ ] Logs del bridge muestran: "Heap: XXX.XKB | Uptime: X.Xmin | Reset: ..."

**Prueba de Stress** (opcional):
```bash
# Dejar corriendo 24 horas y verificar:
# 1. Uptime alcanza > 86400 segundos (24h)
# 2. Free heap NO decrece más de 5KB
# 3. Cero reinicios por TASK_WDT (salvo desconexión intencional del Agent)
```

---

## 📝 Notas Importantes

1. **Watchdog Timeout (20s)**: No aumentar innecesariamente. Si se dispara, HAY un problema real (red bloqueada, loop infinito, etc.). El timeout debe ser suficiente para 3-4 ciclos de sensores (4s cada uno) + latencia de red.

2. **Free Heap Inicial**: Depende de la configuración. En ESP32-WROOM-32D con WiFi + micro-ROS + sensores, ~145-155 KB es normal. Valores < 100 KB son preocupantes.

3. **Reset Reason Logging**: Es crítico para diagnóstico. Si ves `TASK_WDT` frecuentemente, el watchdog está funcionando pero hay un bug subyacente.

4. **Colección `system_health`**: Se crea automáticamente al insertar el primer documento. No requiere configuración manual.

5. **Telemetría en Logs**: Si `LOG_DATA=true` en `.env`, verás telemetría en cada mensaje. Útil para debugging, pero puede saturar logs en producción.

---

## 🎯 Objetivos Post-Deployment

**Corto Plazo (24-48 horas)**:
- ✅ Verificar que el ESP32 NO se congela después de 2 horas
- ✅ Confirmar que free_heap se mantiene estable
- ✅ Validar que no hay reinicios inesperados (TASK_WDT o PANIC)

**Mediano Plazo (1 semana)**:
- ✅ Uptime continuo > 168 horas (1 semana)
- ✅ Gráficas de free_heap en dashboard (MongoDB Charts)
- ✅ Alertas automáticas si free_heap < 100 KB

**Largo Plazo (1 mes)**:
- ✅ Sistema operando 24/7 sin intervención manual
- ✅ Calibración remota funcionando perfectamente
- ✅ Dashboard de telemetría para operadores

---

**Fecha de Deployment**: _______________  
**Operador**: _______________  
**Resultado**: ☐ Exitoso  ☐ Con observaciones  ☐ Fallido  

---

**🔗 Referencias**:
- Firmware v3.2.0: [FIRMWARE_v3.2.0_TELEMETRY.md](./FIRMWARE_v3.2.0_TELEMETRY.md)
- Commit: `7907fa1` (2026-02-17)
- Repositorio: https://github.com/Marton1123/Biofloc-Firmware-ROS
