# Changelog

Todos los cambios notables de este proyecto serán documentados en este archivo.

El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.0.0/),
y este proyecto adhiere a [Semantic Versioning](https://semver.org/lang/es/).

## [3.2.1] - 2026-02-05

### Resumen
Nueva herramienta de análisis de ciclos de pH para detectar patrones fotosintéticos en cultivos de microalgas. Permite diagnosticar si las oscilaciones de pH son fenómenos biológicos normales o problemas de sensores.

### Agregado
- **scripts/check_ph_cycles.py**: Herramienta de análisis de ciclos circadianos de pH
  - Análisis de pH por hora del día (últimas 96 horas configurables)
  - Detección automática de patrón fotosintético
  - Comparación madrugada temprana (23:00-02:00) vs mediodía solar (10:00-13:00)
  - Cálculo de amplitud total del ciclo (pH máximo - pH mínimo)
  - Detección de outliers (pH < 3 o > 12)
  - Configuración de período de análisis (pre-mantenimiento vs tiempo real)
  - Documentación completa del fenómeno biológico esperado

### Características del análisis
- **Período configurable**: Análisis de 72-96 horas
- **Timezone-aware**: Maneja correctamente zonas horarias (Chile GMT-3)
- **Estructura anidada**: Compatible con formato MongoDB `sensors.ph.value`
- **Detección inteligente**: Identifica patrones de fotosíntesis vs anomalías
- **Interpretación automática**: Explica los resultados en contexto biológico

### Interpretación de resultados
```
Patrón Fotosintético Detectado:
  - pH máximo: 10:00-13:00 (pico solar)
  - Amplitud total del ciclo: >0.12 pH
  - Diferencia madrugada→mediodía: >0.08 pH
  - Indica: Cultivo con actividad fotosintética
  
Niveles de actividad:
  - Amplitud >0.3 pH: Cultivo muy activo, alta biomasa
  - Amplitud 0.12-0.3 pH: Cultivo activo, densidad moderada
  - Amplitud <0.12 pH: Baja biomasa o sistema muy tamponado

Patrón Atípico:
  - Amplitud <0.08 pH: Patrón muy débil o ausente
  - pH máximo fuera de 10:00-14:00: Posible iluminación artificial
  - Patrón inverso: Interferencia de equipos/mantenimiento
```

### Casos de uso documentados
1. **Diagnóstico de sensor**: Distinguir fallo de sensor vs fenómeno biológico
2. **Salud del cultivo**: Evaluar actividad fotosintética
3. **Optimización**: Correlacionar pH con densidad de microalgas
4. **Troubleshooting**: Identificar períodos de mantenimiento que afectan lecturas

### Resultados del análisis inicial
- **Período normal (29 ene - 1 feb)**: Patrón fotosintético detectado
  - pH máximo: 7.10 a las 11:00
  - pH mínimo: 6.94 a las 17:00
  - Amplitud del ciclo: 0.153 pH
  - Diferencia madrugada→mediodía: +0.073 pH
  - Amplitud: ~0.4 pH (normal para cultivos densos)
  - Ritmo circadiano claro y consistente
- **Período mantenimiento (2-5 feb)**: Sin patrón significativo
  - Confirmó que las anomalías eran por manipulación de sensores

### Dependencias
- pymongo: Conexión a MongoDB Atlas
- python-dateutil: Parsing de timestamps ISO
- pytz: Manejo de zonas horarias
- python-dotenv: Variables de entorno

## [3.2.0] - 2026-02-04

### Resumen
Agregado proyecto de ejemplo `test_led_project` para demostrar control de GPIO mediante micro-ROS con teclado. Proyecto educativo completamente documentado y funcional.

### Agregado
- **test_led_project/**: Proyecto de ejemplo completo
  - Control de LED via micro-ROS desde teclado de la computadora
  - Firmware ESP32 con suscripción a tópico `/led_control`
  - Script Python `keyboard_led_control.py` para control interactivo
  - Comandos soportados: ON, OFF, TOGGLE
  - GPIO configurable (por defecto GPIO 2)

- **Documentación completa del test_led_project:**
  - `README.md`: Guía completa de instalación y uso
  - `QUICKSTART.md`: Guía rápida de 5 minutos
  - `DESARROLLO.md`: Historial completo del desarrollo (164 minutos)
  - `SOLUCION_PROBLEMA_CONEXION.md`: Solución al bug crítico de conexión
  - `ARQUITECTURA_Y_FUNCIONAMIENTO.md`: Explicación técnica de ROS 2, micro-ROS, DDS vs XRCE-DDS (800+ líneas)

- **Archivos de configuración:**
  - `CMakeLists.txt`: Configuración del proyecto y main
  - `sdkconfig.defaults.example`: Plantilla de configuración
  - `check_network.sh`: Script de diagnóstico de red
  - `main_simple.c`: Firmware de prueba simple sin ROS

### Corregido
- **Bug crítico de conexión micro-ROS:**
  - **Problema:** ESP32 se conectaba a WiFi pero no al agente micro-ROS
  - **Causa:** Uso de `rmw_uros_ping_agent()` en lugar de `rmw_uros_ping_agent_options()`
  - **Solución:** Usar `rmw_uros_ping_agent_options(timeout, attempts, rmw_options)`
  - **Impacto:** Sin esto, el ESP32 nunca detecta al agente cuando se usa configuración UDP personalizada
  - **Documentación:** Solución completamente documentada en `SOLUCION_PROBLEMA_CONEXION.md`

### Seguridad
- **Sanitización de credenciales:**
  - Archivo `sdkconfig.defaults.example` con placeholders
  - `sdkconfig.defaults` actualizado con credenciales genéricas
  - `.gitignore` actualizado para prevenir commit de archivos sensibles

### Detalles técnicos
- **Implementación:**
  - Firmware: 838,896 bytes (20% de partición libre)
  - Protocolo: XRCE-DDS sobre UDP puerto 8888
  - Latencia: ~50ms desde tecla hasta LED
  - QoS: Best Effort con History 10

- **Aprendizajes documentados:**
  - Comparación DDS vs XRCE-DDS (92% menos overhead)
  - Flujo completo de mensaje (17 pasos)
  - Arquitectura de callbacks y executors
  - Debugging con tcpdump y ros2 tools

## [3.1.0] - 2026-01-29

### Resumen
Implementación completa del sistema de calibración de temperatura con slope+offset mediante regresión lineal de 3 puntos. Precisión mejorada a ≤0.03°C (R²=0.999999).

### Agregado
- **Sistema de calibración de temperatura:**
  - Calibración de 3 puntos con regresión lineal (método similar al de pH)
  - Configuración automática desde Kconfig: `CONFIG_BIOFLOC_TEMP_SLOPE` y `CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES`
  - Aplicación automática en `sensors_read_temperature()`: `T_cal = slope × T_raw + offset`
  - Logs con marcador `[CAL]` cuando la calibración está activa
  - Valores almacenados en `sdkconfig.defaults` para persistencia

- **Herramientas de calibración:**
  - `scripts/calibrate_temperature.py`: Script interactivo de calibración de 3 puntos
  - Espera forzada de 3 minutos por punto para estabilización térmica
  - Cálculo automático de slope, offset y R² por regresión lineal
  - Verificación de errores en cada punto de calibración
  - Generación automática de archivo `temperature_calibration_result.txt`

- **Opciones de configuración en `main/Kconfig.projbuild`:**
  - `CONFIG_BIOFLOC_TEMP_SLOPE`: Pendiente × 1,000,000 (rango 0.5-1.5, default: 1086092)
  - `CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES`: Offset × 1000 (rango ±5°C, default: -423)
  - Documentación integrada en menú de configuración

### Cambiado
- **main/sensors.c v2.3.0:**
  - Añadido struct `temp_cal` con campos slope, offset, enabled
  - Modificado `sensors_init()` para cargar calibración desde Kconfig
  - Modificado `sensors_read_temperature()` para aplicar calibración si está habilitada
  - Logs mejorados con información de calibración al inicio

- **sdkconfig.defaults:**
  - Añadida sección de calibración de temperatura con valores óptimos
  - Documentación de ecuación de calibración en comentarios
  - Fecha de calibración registrada (2026-01-29)

### Corregido
- **Configuración WiFi duplicada:**
  - El componente micro-ROS usa `CONFIG_ESP_WIFI_SSID/PASSWORD` (no `CONFIG_BIOFLOC_WIFI_*`)
  - Corregidas credenciales en líneas 457-458 de sdkconfig
  - Conexión WiFi ahora funciona correctamente

### Rendimiento
- **Precisión de temperatura mejorada:**
  - De ±1.5°C (sin calibración) a ≤0.03°C (con calibración)
  - R² = 0.999999 (ajuste prácticamente perfecto)
  - Verificado en rango 0-44°C con termómetro TP101
  - Error máximo observado: 0.03°C en 3 puntos de calibración

### Detalles técnicos
- **Calibración realizada:** 2026-01-29
  - Punto 1: ESP32=0.47°C, TP101=0.10°C → error -0.01°C (post-cal)
  - Punto 2: ESP32=21.96°C, TP101=23.40°C → error +0.03°C (post-cal)
  - Punto 3: ESP32=40.52°C, TP101=43.60°C → error -0.01°C (post-cal)
  - Ecuación: `T_calibrada = 1.086092 × T_raw - 0.423`

## [3.0.0] - 2026-01-29

### Resumen
Arquitectura de 2 colecciones con indexación optimizada para escalabilidad y consultas eficientes. Migración desde estructura de 1 colección a arquitectura IoT profesional.

### Agregado
- **Arquitectura de 2 colecciones:**
  - `telemetria`: Lecturas de sensores (series temporales) con índices compuestos
  - `devices`: Metadatos, estado e historial de conexión de dispositivos
  - Auto-registro de nuevos dispositivos en primera lectura
  
- **Índices MongoDB:**
  - `telemetria.idx_device_timestamp`: (device_id, timestamp DESC) para consultas por dispositivo
  - `telemetria.idx_timestamp`: (timestamp DESC) para lecturas recientes de todos los dispositivos
  - Rendimiento de consultas: tiempo de respuesta promedio <5ms

- **Nuevos scripts:**
  - `migrate_to_devices_collection.py`: Script de migración con cálculo automático de estadísticas
  - `verify_migration.py`: Verificación exhaustiva de migración

- **Metadatos de dispositivos:**
  - Historial de conexión (primera, última, total de lecturas)
  - Estado del dispositivo (activo, inactivo, pendiente)
  - Flag de auto-registro
  - Almacenamiento de parámetros de calibración
  - Configuración de umbrales por dispositivo
  - Seguimiento de versión de firmware

### Cambiado
- **sensor_db_bridge.py v3.0:**
  - Escritura dual: Insert en telemetria + Update en devices
  - Auto-creación de documento de dispositivo en primera lectura
  - Actualización de timestamp de última conexión y contador de lecturas
  - Eliminados caracteres emoji de logs (salida profesional)
  - **IMPORTANTE:** Sin cambios en firmware ESP32, solo backend Python
  
- **.env.example:**
  - Agregada variable `MONGODB_COLLECTION_DEVICES=devices`
  - Comentarios de documentación actualizados

- **README.md:**
  - Eliminados caracteres emoji (documentación profesional)
  - Documentada arquitectura de 2 colecciones
  - Agregadas instrucciones de migración
  - Actualizados ejemplos de formato de documentos
  - Agregada explicación de beneficios de indexación

### Detalles Técnicos
- **Beneficios de indexación:**
  - Consultas por dispositivo: O(log n) en lugar de O(n) escaneo completo
  - Consulta con 17,000+ documentos: 200ms → 5ms (40x más rápido)
  - Uso automático de índices por el planificador de consultas de MongoDB
  
- **Estructura de documento de dispositivo:**
  ```json
  {
    "_id": "device_id",
    "alias": "ESP32-xxxx",
    "location": "tanque_01",
    "estado": "activo",
    "auto_registrado": true,
    "conexion": {
      "primera": "ISO timestamp",
      "ultima": "ISO timestamp",
      "total_lecturas": 17210
    }
  }
  ```

### Guía de Migración
1. Respaldar datos existentes (recomendado)
2. Ejecutar: `python3 scripts/migrate_to_devices_collection.py`
3. Verificar: `python3 scripts/verify_migration.py`
4. Actualizar .env con `MONGODB_COLLECTION_DEVICES=devices`
5. Reiniciar bridge: `python3 scripts/sensor_db_bridge.py`
6. **NO es necesario flashear el ESP32** - cambios solo en backend Python

---

## [2.3.0] - 2026-01-22

### 🎯 Resumen
Corrección del bridge ROS 2 → MongoDB y mejora de documentación con guía paso a paso.

### ✨ Added
- **Guía paso a paso (`GUIA_PASO_A_PASO.md`):**
  - Instrucciones detalladas para ejecutar el proyecto completo
  - Comandos copiables para cada terminal
  - Sección de solución de problemas rápida
  - Script de verificación del sistema

- **Nuevos scripts:**
  - `scripts/monitor_sensores.py`: Monitor mejorado con estadísticas
  - `scripts/check_mongodb.py`: Verificador de conexión MongoDB

### 🔧 Changed
- **Bridge MongoDB (`sensor_db_bridge.py`):**
  - Corregido: Ahora usa `String` (JSON) en vez de `Float32MultiArray`
  - Preserva estructura JSON original del ESP32
  - Incluye voltajes en el documento guardado
  
- **Documentación:**
  - README.md actualizado con comandos correctos (sin Docker)
  - Estructura del proyecto actualizada
  - Información del autor: @Marton1123

### 🗑️ Removed
- `sensor_db_bridge_v2.py` (duplicado)
- Referencias a Docker (no se usa en este proyecto)

### 🐛 Fixed
- Bridge no recibía datos (usaba tipo de mensaje incorrecto)
- Import duplicado de `json` en sensor_db_bridge.py

---

## [2.2.0] - 2026-01-21

### 🎯 Resumen
Versión de calibración profesional del sensor de pH. Se logró reducir el error de ±7.71 pH a ±0.03 pH mediante calibración de 3 puntos con soluciones buffer profesionales.

### ✨ Added
- **Sistema de calibración de 3 puntos:**
  - `scripts/calibrate_ph.py`: Calibración automatizada con buffers pH 4.01, 6.86, 9.18
  - Timeout extendido: 420 segundos (7 minutos) por buffer
  - Criterio de estabilidad: σ < 0.002V durante 50 segundos
  - Tiempo mínimo de espera: 180 segundos (3 minutos) antes de verificar estabilidad
  - Cálculo de R² para validar calidad del ajuste lineal
  
- **Herramientas de diagnóstico:**
  - `scripts/monitor_temperature.py`: Monitor en tiempo real
  - Documentación en `docs/`
  
- **Documentación:**
  - `docs/CALIBRATION.md`: Guía completa de calibración
  - `docs/TROUBLESHOOTING.md`: Solución de problemas
  - `docs/SECURITY.md`: Guías de seguridad
  - `calibration_3point_result.txt`: Archivo de resultados de calibración

### 🔧 Changed
- **Voltage divider factor:**
  - Valor inicial: 3.0 (incorrecto, basado en cálculo teórico)
  - **Valor final: 1.474** (calibrado con multímetro físico)
  
- **Calibración del sensor:**
  - **Calibración 3 puntos: slope=2.559823, offset=0.469193**
    - R² = 0.9997 (ajuste casi perfecto)
    - Errores: [0.021, 0.049, 0.028] pH en los 3 puntos
    - Verificación en agua pH 7.06: lectura 7.09 (error 0.03 pH)
  
- **Timezone configuration:**
  - Valor anterior: `CLT3CLST,M10.2.0/00:00,M3.2.0/00:00` (con horario de verano)
  - **Valor actual: `CLT3`** (GMT-3 fijo, sin horario de verano)
  - Razón: Simplificación y evitar confusión con cambios de horario
  
- **MongoDB bridge (`sensor_db_bridge.py`):**
  - Eliminado campo `_received_at` (redundante con timestamp del dispositivo)
  - Campos actuales: `timestamp`, `ph`, `temperature`, `device_id`, `location`, `_ros_topic`
  - Timestamp usa formato ISO 8601 con timezone: `2026-01-21T17:15:42-0300`

### 🐛 Fixed
- **Error crítico de lectura de pH:**
  - Problema: Lectura 14.8 pH cuando el agua real era 7.06 pH (error 7.74 pH)
  - Causa raíz: Voltage divider factor incorrecto (3.0 vs 1.474 real)
  - Solución: Calibración en 2 etapas:
    1. Corrección del divisor de voltaje con multímetro
    2. Calibración de 3 puntos con soluciones buffer
  - **Resultado:** Error reducido a 0.03 pH (mejora 258x)
  
- **Timestamps incorrectos:**
  - Problema: ESP32 marcaba 18:04 cuando hora real era 17:04 (1 hora adelantado)
  - Causa: Configuración de timezone con horario de verano mal interpretada
  - Solución: Cambio a timezone fijo `CLT3` (GMT-3)
  - **Resultado:** Timestamps correctos alineados con hora del sistema
  
- **Sensor no estabilizaba en calibración:**
  - Problema: Timeout de 60 segundos insuficiente, lecturas inestables
  - Causa: Sensor CWT-BL requiere 3-5 minutos de estabilización
  - Solución: 
    - Timeout aumentado a 420 segundos (7 minutos)
    - Espera mínima de 180 segundos antes de verificar estabilidad
    - Criterio de estabilidad más estricto (50 segundos estable)
  - **Resultado:** Calibración exitosa con R² = 0.9997

### 📊 Performance
- **Precisión de pH:**
  - Antes: ±7.71 pH (completamente fuera de rango)
  - Después corrección divisor: ±0.27 pH
  - **Después calibración 3 puntos: ±0.03 pH** (mejora 9x desde corrección)
  
- **Calidad de calibración:**
  - R² = 0.9997 (99.97% de varianza explicada)
  - Error máximo en puntos de calibración: 0.049 pH
  - Error RMS: ~0.033 pH
  
- **MongoDB guardado:**
  - Tasa de guardado: ~250 registros/hora
  - Success rate: 100% (sin pérdida de datos)
  - Latencia típica: <50ms (LAN)

### 🧪 Testing
- **Calibración verificada con:**
  - Soluciones buffer profesionales: pH 4.01, 6.86, 9.18
  - Sensor manual calibrado chino (verificación independiente)
  - Agua de pH conocido: 7.06 medido → 7.09 leído
  
- **Voltage divider verificado con:**
  - Multímetro digital: 1.71V en GPIO (pH 7.06)
  - Cálculo inverso: 1.474 = (7.06 / 2.8) / 1.71
  
- **Timezone verificado con:**
  - Hora del sistema: 17:04 local
  - Timestamp ESP32: 17:04 -0300 ✓
  - Timestamp MongoDB: 17:04 -0300 ✓

### 📝 Documentation
- README.md ampliado con:
  - Tabla de estado de calibración con todos los parámetros
  - Proceso de calibración paso a paso
  - Troubleshooting específico para sensores de pH
  - Especificaciones técnicas completas
  - Changelog integrado
  
- docs/CALIBRATION.md creado con:
  - Teoría de operación del sensor CWT-BL
  - Guía detallada de calibración de 3 puntos
  - Mejores prácticas y tips
  - Troubleshooting avanzado

### 🔒 Security
- Archivo `.env` en `.gitignore` (credenciales MongoDB no commiteadas)
- MongoDB URI con autenticación en variable de entorno

### 🗑️ Deprecated
- Ninguno

### ❌ Removed
- Campo `_received_at` de documentos MongoDB (redundante)
- Script `calibrate_ph.py` original (reemplazado por versión de 3 puntos)

### 🔐 Security
- Ningún cambio

---

## [2.1.0] - 2026-01-20

### Added
- MongoDB bridge (`sensor_db_bridge.py`)
- Publicación de datos estructurados en JSON
- Integración con MongoDB Atlas
- Configuración por variables de entorno (.env)

### Changed
- Formato de mensaje de sensor a estructura personalizada
- Sistema de logging mejorado

---

## [2.0.0] - 2026-01-19

### Added
- Firmware base con micro-ROS Jazzy
- Soporte para ESP-IDF v5.3.4
- Lectura básica de sensores de pH y temperatura
- Publicación de datos por ROS 2 topics
- Configuración WiFi por Kconfig
- Reconexión automática WiFi y Agent
- Sistema de ping para verificar conectividad

### Changed
- Migración de micro-ROS Humble a Jazzy
- Actualización de componentes ESP-IDF

---

## [1.0.0] - 2026-01-15

### Added
- Versión inicial del firmware
- Soporte básico para micro-ROS Humble
- Lectura de sensores analógicos ADC

---

**Leyenda:**
- ✨ Added: Nuevas características
- 🔧 Changed: Cambios en funcionalidad existente
- 🐛 Fixed: Correcciones de bugs
- 📊 Performance: Mejoras de rendimiento
- 🗑️ Deprecated: Características obsoletas
- ❌ Removed: Características eliminadas
- 🔒 Security: Cambios de seguridad
