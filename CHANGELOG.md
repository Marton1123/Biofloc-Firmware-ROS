# Changelog

Todos los cambios notables de este proyecto serán documentados en este archivo.

El formato está basado en [Keep a Changelog](https://keepachangelog.com/es-ES/1.0.0/),
y este proyecto adhiere a [Semantic Versioning](https://semver.org/lang/es/).

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
