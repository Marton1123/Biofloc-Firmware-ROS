# Sistema de Calibración Remota v3.1.0

**Fecha:** 2026-02-12  
**Versión del Firmware:** 3.1.0  
**Estado:** ✅ Implementado y funcional

## Descripción General

Sistema profesional de calibración remota para sensores ESP32 vía ROS 2 topics. Permite calibrar sensores sin necesidad de conexión USB, manteniendo el ESP32 alimentado por fuente externa y los sensores energizados durante todo el proceso.

### Características Principales

- ✅ **Calibración sin USB**: Opera remotamente vía ROS 2 topics
- ✅ **Genérico y escalable**: Arquitectura lista para sensores futuros
- ✅ **N-point calibration**: Soporta de 2 a 5 puntos de calibración
- ✅ **Regresión lineal**: Cálculo automático de pendiente, offset y R²
- ✅ **Persistencia NVS**: Calibración se guarda en memoria no volátil
- ✅ **Feedback en tiempo real**: Respuestas inmediatas del ESP32
- ✅ **Validación automática**: Verificación de puntos y rangos

## Arquitectura

### Componentes del Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                  Raspberry Pi 3 Gateway                      │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                 biofloc_manager.py                    │   │
│  │  - Menú interactivo                                   │   │
│  │  - Lectura de voltajes en tiempo real                │   │
│  │  - Construcción de comandos JSON                     │   │
│  │  - Publicación a /biofloc/calibration_cmd            │   │
│  └──────────────────────────────────────────────────────┘   │
│                           │                                   │
│                           ▼                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │            micro-ROS Agent (UDP port 8888)            │   │
│  └──────────────────────────────────────────────────────┘   │
└───────────────────────────┬───────────────────────────────────┘
                            │ WiFi (10.42.0.1/24)
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                        ESP32                                 │
│                                                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              main.c - ROS Subscriber                  │   │
│  │  Topic: /biofloc/calibration_cmd                     │   │
│  │  - Parser JSON                                        │   │
│  │  - Validador de comandos                             │   │
│  │  - Dispatcher de acciones                            │   │
│  └──────────────────────────────────────────────────────┘   │
│                           │                                   │
│                           ▼                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │        sensors.c - Calibration Engine                 │   │
│  │  - sensors_calibrate_generic()                        │   │
│  │  - Regresión lineal (N-point)                        │   │
│  │  - Cálculo de R² (goodness of fit)                   │   │
│  │  - Persistencia NVS                                   │   │
│  └──────────────────────────────────────────────────────┘   │
│                           │                                   │
│                           ▼                                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Response Publisher                       │   │
│  │  Topic: /biofloc/calibration_status                  │   │
│  │  - Estado (success/error)                            │   │
│  │  - Parámetros calculados                             │   │
│  │  - Mensajes descriptivos                             │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### Topics ROS 2

| Topic                           | Type                | Dirección | Descripción                               |
|---------------------------------|---------------------|-----------|-------------------------------------------|
| `/biofloc/calibration_cmd`      | std_msgs/String     | RPi → ESP | Comandos de calibración (JSON)           |
| `/biofloc/calibration_status`   | std_msgs/String     | ESP → RPi | Respuestas y confirmaciones (JSON)        |
| `/biofloc/sensor_data`          | std_msgs/String     | ESP → RPi | Datos de sensores (incluye voltajes)      |

## Protocolo JSON

### Comando de Calibración

```json
{
  "sensor": "ph",
  "action": "calibrate",
  "points": [
    {"voltage": 1.423, "value": 4.01},
    {"voltage": 2.449, "value": 6.86},
    {"voltage": 3.282, "value": 9.18}
  ]
}
```

**Campos:**
- `sensor`: Tipo de sensor (`ph`, `temperature`, `dissolved_oxygen`, `conductivity`, `turbidity`)
- `action`: Acción a realizar (`calibrate`, `reset`, `get`)
- `points`: Array de 2-5 puntos de calibración (solo para action=`calibrate`)
  - `voltage`: Voltaje leído del sensor (V)
  - `value`: Valor de referencia conocido

### Respuesta de Calibración

```json
{
  "status": "success",
  "sensor": "ph",
  "slope": 2.559823,
  "offset": 0.469193,
  "r_squared": 0.9997,
  "message": "Calibration successful: R²=0.9997, slope=2.559823, offset=0.469193"
}
```

**Campos:**
- `status`: Estado (`success` o `error`)
- `sensor`: Tipo de sensor calibrado
- `slope`: Pendiente de la recta de calibración
- `offset`: Offset de la recta de calibración
- `r_squared`: Coeficiente de determinación (bondad de ajuste, 0-1)
- `message`: Mensaje descriptivo

### Otros Comandos

**Reset a valores de fábrica:**
```json
{
  "sensor": "ph",
  "action": "reset"
}
```

**Consultar calibración actual:**
```json
{
  "sensor": "ph",
  "action": "get"
}
```

## Sensores Soportados

### Implementados

| Sensor      | ID            | Unidad | Puntos recomendados                    | Estado |
|-------------|---------------|--------|----------------------------------------|--------|
| pH          | `ph`          | pH     | 3 puntos (4.01, 6.86, 9.18)            | ✅ Activo |
| Temperatura | `temperature` | °C     | 3 puntos (0°C, 25°C, 50°C)             | ✅ Activo |

### Preparados para Futuro

| Sensor             | ID                  | Unidad  | Estado              |
|--------------------|---------------------|---------|---------------------|
| Oxígeno Disuelto   | `dissolved_oxygen`  | mg/L    | 🔧 Hardware pendiente |
| Conductividad      | `conductivity`      | μS/cm   | 🔧 Hardware pendiente |
| Turbidez           | `turbidity`         | NTU     | 🔧 Hardware pendiente |

## Uso del Sistema

### Desde biofloc_manager.py

```bash
cd /home/Biofloc-Firmware-ROS
python3 biofloc_manager.py
```

**Menú → Opción [6]: ⭐ Calibración Remota (Recomendado)**

#### Flujo de Calibración pH (3 puntos)

1. **Seleccionar sensor:** pH
2. **Seleccionar tipo:** Calibración completa (3 puntos)
3. **Punto 1 - pH 4.01:**
   - Enjuagar sensor con agua destilada
   - Sumergir en buffer pH 4.01
   - Esperar 30-60 segundos
   - Presionar Enter
   - Sistema lee voltaje automáticamente desde topic `/biofloc/sensor_data`
   - Ingresar valor de referencia: `4.01`

4. **Punto 2 - pH 6.86:**
   - Enjuagar sensor
   - Sumergir en buffer pH 6.86
   - Esperar estabilización
   - Presionar Enter
   - Ingresar valor: `6.86`

5. **Punto 3 - pH 9.18:**
   - Enjuagar sensor
   - Sumergir en buffer pH 9.18
   - Esperar estabilización
   - Presionar Enter
   - Ingresar valor: `9.18`

6. **Confirmación:**
   - Sistema muestra resumen de puntos
   - Confirmar para aplicar
   - ESP32 calcula regresión lineal
   - Guarda en NVS
   - Muestra R² y parámetros

**Ejemplo de salida:**
```
═══ Resumen de Calibración ═══
Sensor: pH
Puntos: 3
  Punto 1: 1.423V → 4.01 pH
  Punto 2: 2.449V → 6.86 pH
  Punto 3: 3.282V → 9.18 pH

¿Aplicar esta calibración? (S/n): S

✓ Calibración de pH completada exitosamente
  Slope: 2.559823
  Offset: 0.469193
  R²: 0.9997
```

### Comando Manual (ROS CLI)

**Publicar comando:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash

ros2 topic pub --once /biofloc/calibration_cmd std_msgs/msg/String \
  "data: '{\"sensor\":\"ph\",\"action\":\"calibrate\",\"points\":[{\"voltage\":1.423,\"value\":4.01},{\"voltage\":2.449,\"value\":6.86},{\"voltage\":3.282,\"value\":9.18}]}'"
```

**Escuchar respuesta:**
```bash
ros2 topic echo --once /biofloc/calibration_status
```

## Matemática de la Calibración

### Regresión Lineal (Mínimos Cuadrados)

Dada una serie de puntos `(V₁, y₁), (V₂, y₂), ..., (Vₙ, yₙ)`:

**Fórmulas:**

```
pendiente (m) = [n·∑(V·y) - ∑V·∑y] / [n·∑(V²) - (∑V)²]

offset (b) = [∑y·∑(V²) - ∑V·∑(V·y)] / [n·∑(V²) - (∑V)²]

y = m·V + b
```

**Donde:**
- `V`: Voltaje del sensor (0-5V)
- `y`: Valor físico (pH, °C, etc.)
- `n`: Número de puntos

### Coeficiente de Determinación (R²)

Mide la bondad del ajuste (0 = muy malo, 1 = perfecto):

```
SS_tot = ∑(yᵢ - ȳ)²    (Varianza total)
SS_res = ∑(yᵢ - ŷᵢ)²   (Varianza residual)

R² = 1 - (SS_res / SS_tot)
```

**Interpretación:**
- R² ≥ 0.99: Excelente
- R² ≥ 0.95: Bueno
- R² < 0.90: Revisar puntos

## Persistencia en NVS

### Namespace
`biofloc_cal`

### Keys
- `cal_0`: Calibración de pH
- `cal_1`: Calibración de temperatura
- `cal_2`: Calibración de oxígeno disuelto (futuro)
- `cal_3`: Calibración de conductividad (futuro)
- `cal_4`: Calibración de turbidez (futuro)

### Estructura Almacenada

```c
typedef struct {
    sensor_type_t type;                             
    calibration_point_t points[5];  // Hasta 5 puntos
    uint8_t num_points;             
    float slope;                    
    float offset;                   
    bool enabled;                   
    float r_squared;                
    char timestamp[32];             // ISO8601
} sensor_calibration_t;
```

## Códigos de Error

| Código                         | Descripción                                    |
|--------------------------------|------------------------------------------------|
| `CAL_STATUS_SUCCESS`           | Calibración exitosa                            |
| `CAL_STATUS_INVALID_SENSOR`    | Tipo de sensor desconocido                     |
| `CAL_STATUS_INVALID_POINTS`    | Puntos con valores inválidos (voltaje ≤ 0)    |
| `CAL_STATUS_INSUFFICIENT_POINTS` | Necesita 2-5 puntos                          |
| `CAL_STATUS_NVS_ERROR`         | Error guardando en NVS                         |
| `CAL_STATUS_NOT_INITIALIZED`   | Subsistema de sensores no inicializado        |

## Troubleshooting

### Problema: "No se pudo leer voltaje del sensor"

**Causas posibles:**
1. ESP32 no está publicando datos
2. micro-ROS Agent no está activo
3. Topic `/biofloc/sensor_data` sin mensajes

**Solución:**
```bash
# Verificar que el Agent esté corriendo
ps aux | grep micro_ros_agent

# Verificar topics activos
ros2 topic list

# Monitorear datos del sensor
ros2 topic echo /biofloc/sensor_data
```

### Problema: "Calibración aplicada pero R² bajo"

**Causas posibles:**
1. Sensor no estabilizado
2. Soluciones buffer contaminadas
3. Sensor defectuoso

**Solución:**
- Esperar más tiempo entre mediciones (60-120 segundos)
- Usar soluciones buffer frescas
- Verificar con instrumento de referencia
- Repetir calibración con más cuidado

### Problema: "NVS save failed"

**Causas posibles:**
1. NVS partition llena
2. NVS corrupto

**Solución:**
```bash
# Borrar y re-flashear NVS
idf.py erase-flash
idf.py flash
```

## Ventajas sobre Calibración USB

| Aspecto                  | USB (Anterior)                | Remota (v3.1.0)              |
|--------------------------|-------------------------------|------------------------------|
| **Conexión física**      | Requiere USB                  | ✅ Sin cables, vía WiFi       |
| **Energía de sensores**  | ⚠️ Puede interrumpirse         | ✅ Siempre energizados        |
| **Riesgo de conflicto**  | ⚠️ USB + fuente externa        | ✅ Solo fuente externa        |
| **Escalabilidad**        | Manual por sensor             | ✅ Cualquier sensor futuro    |
| **Automatización**       | Script local                  | ✅ API ROS 2 (remoto/scripts) |
| **Feedback**             | Serial log                    | ✅ Topics ROS con JSON        |
| **Persistencia**         | Kconfig (recompilación)       | ✅ NVS (inmediata)            |

## Agregar un Nuevo Sensor

### Paso 1: Definir en sensors.h

```c
typedef enum {
    SENSOR_TYPE_PH = 0,
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_DISSOLVED_OXYGEN,    // ← Nuevo
    SENSOR_TYPE_CONDUCTIVITY,
    SENSOR_TYPE_TURBIDITY,
    SENSOR_TYPE_MAX
} sensor_type_t;
```

### Paso 2: Agregar nombre en sensors.c

```c
static const char* SENSOR_TYPE_NAMES[] = {
    [SENSOR_TYPE_PH] = "pH",
    [SENSOR_TYPE_TEMPERATURE] = "Temperature",
    [SENSOR_TYPE_DISSOLVED_OXYGEN] = "Dissolved Oxygen",  // ← Nuevo
    [SENSOR_TYPE_CONDUCTIVITY] = "Conductivity",
    [SENSOR_TYPE_TURBIDITY] = "Turbidity"
};
```

### Paso 3: Actualizar calibration_callback en main.c

```c
} else if (strcmp(sensor_str, "dissolved_oxygen") == 0) {
    sensor_type = SENSOR_TYPE_DISSOLVED_OXYGEN;  // ← Nuevo case
```

### Paso 4: Actualizar biofloc_manager.py

```python
sensor_map = {
    '1': ('ph', 'pH', [4.01, 6.86, 9.18]),
    '2': ('temperature', 'Temperatura', [0.0, 25.0, 50.0]),
    '3': ('dissolved_oxygen', 'Oxígeno Disuelto', [0.0, 5.0, 10.0]),  # ← Nuevo
    # ...
}
```

### Paso 5: Implementar lectura en sensors.c

```c
esp_err_t sensors_read_dissolved_oxygen(sensor_reading_t *reading) {
    // Implementar lectura específica del sensor
}
```

**Total de modificaciones:** ~5 líneas de código por sensor nuevo ✅

## Seguridad

### Validaciones Implementadas

1. ✅ Validación de tipo de sensor
2. ✅ Validación de número de puntos (2-5)
3. ✅ Validación de voltajes (> 0V)
4. ✅ Validación de formato JSON
5. ✅ Verificación de estado del sistema

### Protección de Datos

- Calibraciones en NVS (separado de configuración WiFi)
- Sin exposición de credenciales en topics
- Namespace aislado (`biofloc_cal`)

## Roadmap

### v3.2 (Futuro)
- [ ] Calibración polinomial (curvas no lineales)
- [ ] Validación cruzada automática
- [ ] Historial de calibraciones
- [ ] Interfaz web (dashboard)

### v3.3 (Futuro)
- [ ] Calibración multi-punto asistida por IA
- [ ] Detección automática de drift
- [ ] Alarmas de calibración vencida

## Referencias

- [ESP-IDF NVS API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)
- [Linear Regression - Least Squares Method](https://mathworld.wolfram.com/LeastSquaresFitting.html)
- [Coefficient of Determination (R²)](https://en.wikipedia.org/wiki/Coefficient_of_determination)

## Autor

**Proyecto:** Biofloc Firmware ROS  
**Versión:** 3.1.0  
**Fecha:** Febrero 2026  
**Licencia:** MIT  
