# Resumen Técnico — Sistema de Telemetría pH v3.0.0

> **Estándares:** ISO/IEC 25010 (Calidad de Software) | IEEE 829 (Documentación de Pruebas)

| Metadatos | Valor |
|-----------|-------|
| **Versión Firmware** | 3.0.0 (Secure Gateway) |
| **Versión Gestor** | 1.0.0 (biofloc_manager.py) |
| **Fecha** | 2026-02-10 |
| **Estado** | ✅ Operacional (Gateway Seguro) |
| **Precisión pH** | ±0.05 pH (99.4%) |
| **Precisión Temp** | ~1.6°C error residual (ajustable) |

---

## 📊 Parámetros de Calibración Actuales

### Divisor de Voltaje (Hardware Verificado desde PCB)

**Hardware (AMBOS sensores pH y Temperatura):**
```
R1 = 10 kΩ (pull-up, desde Vin del sensor)
R2 = 20 kΩ (pull-down a GND)
R3 = 470 Ω (protección serie)
C1 = 100 nF (filtro paralelo)

Factor = (R1 + R2) / R2 = (10k + 20k) / 20k = 1.5
```

**Configuración en firmware:**
```ini
# sdkconfig.defaults
CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR=1500    # 1.5 × 1000
CONFIG_BIOFLOC_TEMP_VOLTAGE_DIVIDER_FACTOR=1500  # 1.5 × 1000
```

**⚠️ Historia de Errores Corregidos:**
1. **Error 1 (pH):** Asumido R1=20k/R2=10k → factor=3.0 → **CORREGIDO a 1.5**
2. **Error 2 (Temp):** Mismo error en divisor → **CORREGIDO a 1.5**
3. **Error 3 (Temp):** Offset sign negativo (-423) → **CORREGIDO a +1382**
4. **Hardware verificado:** Foto de PCB confirmó R1=10k, R2=20k

**Verificación:**
- V₂₈₂₈ (sensor output): 0-5V
- V₂₇₃₂ (ADC input): 0-3.3V
- V₂₇₃₂ = V₂₈₂₈ / 1.5
- V₂₈₂₈ = V₂₇₃₂ × 1.5 ✓

### Calibración del Sensor (3 puntos)

**Parámetros aplicados en firmware:**
```c
sensors_calibrate_ph_manual(2.559823f, 0.469193f);
```

**Fórmula resultante:**
```
pH = 2.559823 × V_sensor + 0.469193
```

**Calidad de calibración:**
| Métrica | Valor |
|---------|-------|
| R² | 0.9997 |
| Error máximo | 0.049 pH |
| Error RMS | 0.033 pH |
| Puntos calibrados | pH 4.01, 6.86, 9.18 |

**Resultados por punto:**
| Buffer | Voltaje | pH esperado | pH calculado | Error |
|--------|---------|-------------|--------------|-------|
| 4.01 | 1.3872V | 4.01 | 4.031 | +0.021 |
| 6.86 | 2.5032V | 6.86 | 6.909 | +0.049 |
| 9.18 | 3.3899V | 9.18 | 9.152 | -0.028 |

**Verificación en agua real:**
- pH real (sensor manual): 7.06
- pH leído (sistema): 7.09
- **Error: 0.03 pH** ✅

---

## ⚙️ Configuración del Sistema

### Arquitectura de Red (Gateway Seguro)

```
Internet
   |
   | Ethernet (enp88s0)
   |
[Gateway - NUC Ubuntu 24.04]
   | IP Ethernet: (DHCP de ISP)
   | IP WiFi: 10.42.0.1/24
   | Firewall: iptables FORWARD DROP
   |
   | WiFi Hotspot (wlo1)
   | SSID: <tu-ssid-gateway>
   | Password: <tu-password-seguro>
   |
[ESP32]
   | MAC: 24:0a:c4:60:c8:e0
   | IP: 10.42.0.123 (DHCP)
   | SIN acceso a internet
```

### ESP32 (sdkconfig.defaults)

**IMPORTANTE:** Dual WiFi credentials (micro_ros + biofloc):
```ini
# Credenciales para componente micro_ros_espidf_component
CONFIG_ESP_WIFI_SSID="<tu-ssid-gateway>"
CONFIG_ESP_WIFI_PASSWORD="<tu-password-seguro>"
CONFIG_ESP_MAXIMUM_RETRY=15

# Credenciales para aplicación biofloc (main)
CONFIG_BIOFLOC_WIFI_SSID="<tu-ssid-gateway>"
CONFIG_BIOFLOC_WIFI_PASSWORD="<tu-password-seguro>"

# Agent (Gateway IP en red interna)
CONFIG_MICRO_ROS_AGENT_IP="10.42.0.1"
CONFIG_MICRO_ROS_AGENT_PORT=8888

# ROS
CONFIG_BIOFLOC_ROS_NAMESPACE="biofloc"
CONFIG_BIOFLOC_LOCATION="tanque_01"
CONFIG_BIOFLOC_SENSOR_SAMPLE_INTERVAL_MS=4000

# Calibración pH
CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR=1500
CONFIG_BIOFLOC_PH_SLOPE_MILLIPH_PER_VOLT=2559823     # 2.559823 × 1000000
CONFIG_BIOFLOC_PH_OFFSET_MILLIPH=469193              # 0.469193 × 1000000

# Calibración Temperatura
CONFIG_BIOFLOC_TEMP_VOLTAGE_DIVIDER_FACTOR=1500
CONFIG_BIOFLOC_TEMP_SLOPE=1000000                    # 1.0 × 1000000
CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES=1382        # +1.382°C

# Timezone (sin NTP - timestamps del servidor)
CONFIG_BIOFLOC_TIMEZONE="CLT3"
```

### MongoDB (.env)
```bash
MONGODB_URI=mongodb+srv://usuario:PASSWORD@cluster.mongodb.net/?retryWrites=true&w=majority&appName=TuApp
MONGODB_DB=TuDatabase
MONGODB_COLLECTION=telemetria
```

### Formato de Datos MongoDB

**Estructura de documento (con timestamps del servidor):**
```json
{
  "timestamp": "2026-02-10T14:32:15.847Z",        // Timestamp real del servidor (UTC)
  "timestamp_esp32": "sample_1523",               // Contador del ESP32 (sin NTP)
  "ph": 7.08,
  "temperature": 23.45,
  "device_id": "biofloc_esp32_c8e0",
  "location": "tanque_01",
  "_ros_topic": "/biofloc/sensor_data"
}
```

**Notas:**
- `timestamp`: Generado por `sensor_db_bridge.py` en el gateway (con acceso a internet/NTP)
- `timestamp_esp32`: Contador incremental del ESP32 (para correlación, NO es tiempo real)
- ESP32 opera SIN acceso a NTP (no tiene internet)
- Gateway agrega timestamp real antes de guardar en MongoDB

---

## 🔧 Herramientas de Mantenimiento

### Gestor Unificado (Recomendado)
```bash
python3 biofloc_manager.py
```
**Menú principal (12 opciones):**
1. ▶️ Iniciar micro-ROS Agent
2. ▶️ Iniciar sensor_db_bridge.py
3. 📊 Iniciar monitor_sensores.py
4. ✅ Verificar estado del sistema (8s timeout)
5. 🔌 Verificar conectividad ESP32 (DHCP, ARP, ping, ROS)
6. 🧪 Calibración completa pH (3 puntos interactiva)
7. 🌡️ Calibración completa Temperatura (3 puntos)
8. ⚡ Ajuste rápido pH (manual, 1 valor)
9. ⚡ Ajuste rápido Temperatura (manual, 1 valor)
10. 📶 Configurar WiFi (actualiza dual credentials)
11. ⚙️ Regenerar sdkconfig (desde defaults)
12. 🛠️ Compilar y Flashear (pipeline completo)

**Características:**
- Interfaz completamente en español
- Timeouts inteligentes (8s rápido, 20s opcional)
- Verificación completa de conectividad ESP32
- Actualización automática de sdkconfig.defaults
- Manejo robusto de procesos (sin pipes grep)

### Scripts Individuales (Alternativa)

### 1. Monitor de Voltaje en Tiempo Real
```bash
python3 scripts/monitor_voltage.py
```
**Uso:** Verificar voltajes con multímetro durante troubleshooting

### 2. Calibración de pH (3 Puntos)
```bash
python3 scripts/calibrate_ph.py
```
**Uso:** Recalibrar sensor con soluciones buffer 4.01, 6.86, 9.18
**Duración:** 15-30 minutos (3-7 min por buffer)

### 3. Calibración de Temperatura
```bash
python3 scripts/calibrate_temperature.py
```
**Uso:** Calibración interactiva con 3 puntos de temperatura
**Duración:** ~15 minutos

### 4. Monitor en Tiempo Real
```bash
python3 scripts/monitor_sensores.py
```
**Uso:** Visualización de datos con estadísticas (presionar Ctrl+C para reporte final)

---

## 📈 Performance del Sistema

### Métricas de Operación
| Métrica | Valor |
|---------|-------|
| Tasa de muestreo | 1 Hz (~cada 4s) |
| Latencia WiFi | <50ms (LAN) |
| Success rate MongoDB | 100% |
| Registros guardados | ~250/hora |
| Uptime típico | 24/7 |

### Consumo de Recursos
| Recurso | Uso | Libre |
|---------|-----|-------|
| Flash (app) | 867 KB | 1157 KB (57%) |
| RAM (runtime) | ~98 KB | ~230 KB (70%) |
| CPU (idle) | ~5% | ~95% |

---

## 🚨 Troubleshooting Rápido

### pH fuera de rango (>14 o <0)
1. Medir V_GPIO con multímetro
2. Comparar con lectura del software (`monitor_voltage.py`)
3. Si difieren >5%, recalcular divisor: `fix_voltage_divider.py`
4. Actualizar `CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR` en sdkconfig
5. Recompilar y flashear

### Error de pH >0.3
1. Verificar divisor de voltaje primero (arriba)
2. Preparar buffers pH 4.01, 6.86, 9.18 a temp ambiente
3. Ejecutar `calibrate_ph_3points.py`
4. Esperar 3-5 min de estabilización por buffer
5. Verificar R² > 0.999 en resultados
6. Aplicar parámetros en `main.c`: `sensors_calibrate_ph_manual(slope, offset)`
7. Recompilar y flashear

### Sensor no estabiliza
- Enjuagar con agua desmineralizada
- Esperar 3-5 minutos antes de leer
- Verificar temperatura del agua (±2°C)
- Asegurar inmersión completa del sensor
- Revisar si hay burbujas en el electrodo

### Timestamps incorrectos
1. Verificar timezone en sdkconfig: `CONFIG_BIOFLOC_TIMEZONE="CLT3"`
2. Verificar sincronización NTP: revisar logs del firmware
3. Verificar hora del sistema: `date`
4. Si no sincroniza: revisar firewall (puerto 123 UDP)

### MongoDB no guarda datos
1. Verificar Agent ejecutando: `ros2 topic list | grep biofloc`
2. Verificar bridge ejecutando: `ps aux | grep sensor_db_bridge`
3. Revisar `.env` con credenciales correctas
4. Verificar IP whitelisting en MongoDB Atlas
5. Probar conexión: `ping <tu-cluster>.mongodb.net`

---

## 📋 Checklist de Puesta en Marcha

### Inicial (primera vez)
- [ ] Instalar ESP-IDF v5.3.4+
- [ ] Instalar ROS 2 Jazzy
- [ ] Clonar repositorio con submódulo micro-ROS
- [ ] Configurar WiFi y Agent IP en menuconfig
- [ ] Compilar y flashear firmware base
- [ ] Verificar conexión con Agent

### Calibración
- [ ] Medir V_GPIO con multímetro en agua conocida
- [ ] Calcular y aplicar voltage divider factor
- [ ] Preparar soluciones buffer pH 4.01, 6.86, 9.18
- [ ] Ejecutar calibración de 3 puntos
- [ ] Verificar R² > 0.999
- [ ] Aplicar parámetros en firmware
- [ ] Recompilar y flashear
- [ ] Verificar error <0.05 pH en agua conocida

### Producción
- [ ] Configurar timezone correcto
- [ ] Verificar sincronización NTP
- [ ] Configurar MongoDB .env
- [ ] Probar guardado de datos
- [ ] Verificar timestamps correctos
- [ ] Monitorear por 1 hora (success rate >95%)
- [ ] Documentar instalación específica

---

## 🔄 Mantenimiento Recomendado

### Diario
- Verificar que los datos llegan a MongoDB
- Revisar lectura pH con sensor manual (spot check)

### Semanal
- Revisar estadísticas del bridge (success rate)
- Enjuagar sensor con agua desmineralizada
- Limpiar electrodos si hay depósitos

### Mensual
- Recalibrar con 1 punto (buffer pH 6.86)
- Verificar voltage divider factor con multímetro
- Backup de configuración (sdkconfig, .env)

### Trimestral
- Recalibración completa de 3 puntos
- Revisar conexiones eléctricas
- Actualizar firmware si hay mejoras

---

## 📞 Contacto y Soporte

**Proyecto:** Biofloc Firmware ROS  
**Versión:** 2.2.0  
**Repositorio:** [Biofloc-Firmware-ROS](https://github.com/Marton1123/Biofloc-Firmware-ROS)  
**Documentación:** README.md, docs/CALIBRATION.md  
**Licencia:** MIT

---

**Última actualización:** 2026-01-22  
**Próxima revisión:** Trimestral (Abril 2026)
