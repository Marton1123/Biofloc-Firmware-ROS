# Resumen Técnico — Sistema de Telemetría pH v2.2.0

> **Estándares:** ISO/IEC 25010 (Calidad de Software) | IEEE 829 (Documentación de Pruebas)

| Metadatos | Valor |
|-----------|-------|
| **Versión** | 2.2.0 |
| **Fecha** | 2026-01-22 |
| **Estado** | ✅ Operacional |
| **Precisión pH** | ±0.03 pH (99.4%) |

---

## 📊 Parámetros de Calibración Actuales

### Divisor de Voltaje
```
R1 = 20 kΩ
R2 = 10 kΩ
Factor calibrado = 1.474
CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR = 1474  (× 1000)
```

**Verificación:**
- V_GPIO medido: 1.71V (multímetro)
- pH agua: 7.06 (sensor manual)
- Cálculo: 1.474 = (7.06 / 2.8) / 1.71 ✓

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

### ESP32 (sdkconfig)
```ini
CONFIG_BIOFLOC_WIFI_SSID="tu_wifi"
CONFIG_BIOFLOC_WIFI_PASSWORD="tu_password"
CONFIG_BIOFLOC_AGENT_IP="192.168.1.100"
CONFIG_BIOFLOC_AGENT_PORT=8888
CONFIG_BIOFLOC_ROS_NAMESPACE="biofloc"
CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR=1474
CONFIG_BIOFLOC_TIMEZONE="CLT3"
CONFIG_BIOFLOC_NTP_SERVER="pool.ntp.org"
CONFIG_BIOFLOC_LOCATION="tanque_01"
CONFIG_BIOFLOC_SENSOR_SAMPLE_INTERVAL_MS=4000
```

### MongoDB (.env)
```bash
MONGODB_URI=mongodb+srv://sistemaslab:PASSWORD@sistemaslab.hk30i2k.mongodb.net/?retryWrites=true&w=majority&appName=SistemasLab
MONGODB_DB=SistemasLab
MONGODB_COLLECTION=telemetria
```

### Formato de Datos MongoDB
```json
{
  "timestamp": "2026-01-21T17:15:42-0300",
  "ph": 7.08,
  "temperature": 2.26,
  "device_id": "biofloc_esp32_c8e0",
  "location": "tanque_01",
  "_ros_topic": "/biofloc/sensor_data"
}
```

---

## 🔧 Herramientas de Mantenimiento

### 1. Monitor de Voltaje en Tiempo Real
```bash
python3 scripts/monitor_voltage.py
```
**Uso:** Verificar voltajes con multímetro durante troubleshooting

### 2. Diagnóstico de Divisor de Voltaje
```bash
python3 scripts/fix_voltage_divider.py
```
**Uso:** Calcular factor correcto del divisor de voltaje

### 3. Calibración de 3 Puntos
```bash
python3 scripts/calibrate_ph_3points.py
```
**Uso:** Recalibrar sensor con soluciones buffer
**Duración:** 15-30 minutos (3-7 min por buffer)

### 4. Diagnóstico General
```bash
python3 scripts/diagnose_ph.py
```
**Uso:** Troubleshooting general del sensor

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
5. Probar conexión: `ping sistemaslab.hk30i2k.mongodb.net`

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
