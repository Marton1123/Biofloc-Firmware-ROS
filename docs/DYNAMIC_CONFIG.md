# Sistema de Configuración Dinámica de Sensores

## 📋 Descripción General

El firmware v4.0.0 incluye un **sistema profesional de configuración dinámica** que permite cambiar en tiempo real:
- **Intervalo de muestreo** (sample_interval_ms): Cada cuánto leer sensores
- **Intervalo de publicación** (publish_interval_ms): Cada cuánto publicar a ROS2
- **Modo de agregación**: instant, average, median, min_max, last
- **Número de muestras**: Cuántas muestras acumular antes de publicar

## 🎯 Casos de Uso

### Modo Normal (4 segundos)
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "action": "set_config",
  "config": {
    "sample_interval_ms": 4000,
    "publish_interval_ms": 4000,
    "mode": "instant",
    "samples_per_publish": 1,
    "enabled": true
  }
}
```
**Uso**: Monitoreo en tiempo real, calibración, desarrollo

### Modo Ahorro de Datos (30 minutos con mediana)
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "action": "set_config",
  "config": {
    "sample_interval_ms": 4000,
    "publish_interval_ms": 1800000,
    "mode": "median",
    "samples_per_publish": 450,
    "enabled": true
  }
}
```
**Uso**: Piscicultura estable, reducir tráfico de red, optimizar base de datos
- Lee cada 4s internamente (450 muestras en 30 min)
- Calcula mediana de esas 450 muestras
- Publica 1 valor cada 30 minutos
- **Reducción de datos: 99.8%** (1 mensaje vs 450)

### Modo Monitoreo Continuo (5 minutos con promedio)
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "action": "set_config",
  "config": {
    "sample_interval_ms": 4000,
    "publish_interval_ms": 300000,
    "mode": "average",
    "samples_per_publish": 75,
    "enabled": true
  }
}
```
**Uso**: Balance entre resolución temporal y eficiencia
- Promedio de 75 muestras (5 minutos)
- **Reducción de datos: 98.7%** (1 mensaje vs 75)

### Modo Producción (1 minuto última muestra)
```json
{
  "device_id": "biofloc_esp32_c8e0",
  "action": "set_config",
  "config": {
    "sample_interval_ms": 60000,
    "publish_interval_ms": 60000,
    "mode": "last",
    "samples_per_publish": 1,
    "enabled": true
  }
}
```
**Uso**: Producción con alertas rápidas
- Lee cada 60s
- Publica inmediatamente
- Menor carga de CPU que 4s

## 🔧 Arquitectura del Sistema

### Módulos Profesionales

#### 1. **Data Aggregator** (`middleware/data_aggregator.c`)
Responsable de:
- Acumular N muestras en buffer circular (max 64 muestras)
- Calcular estadísticas:
  - **Mediana**: Ordena valores y retorna valor central (robusto a outliers)
  - **Promedio**: Media aritmética de todas las muestras
  - **Última muestra**: Retorna la más reciente
  - **Min/Max**: (Futuro) Retorna rango de valores
- Gestión de ventanas de tiempo
- Thread-safe con FreeRTOS mutex

**API Principales**:
```c
esp_err_t data_aggregator_add_sample(const sensors_data_t *data);
bool data_aggregator_should_publish(void);
esp_err_t data_aggregator_get_result(sensors_data_t *result);
```

#### 2. **Config Manager** (`middleware/config_manager.c`)
Responsable de:
- Recibir comandos vía ROS2 topic `/biofloc/config_cmd`
- Validar configuración propuesta:
  - Intervalos: 1s - 60s (sample), 1s - 1h (publish)
  - Coherencia: publish_interval ≥ sample_interval
  - Samples: 1 - 450 máximo
- Aplicar configuración en app_state (thread-safe)
- Responder ACK/NACK en `/biofloc/config_status`
- Persistir en NVS (futuro)

**API Principales**:
```c
esp_err_t config_manager_validate_config(const sensor_config_t *config);
esp_err_t config_manager_apply_defaults(void);
void config_manager_command_callback(const void *msgin);
```

#### 3. **App State** (`core/app_state.c`)
Contiene configuración actual:
```c
typedef struct {
    uint32_t sample_interval_ms;        // Intervalo de lectura
    uint32_t publish_interval_ms;       // Intervalo de publicación
    data_aggregation_mode_t mode;       // instant/average/median/min_max/last
    uint16_t samples_per_publish;       // N muestras a agregar
    bool enabled;                       // Publicación habilitada
} sensor_config_t;
```
- Acceso thread-safe con mutex
- Integrado en app_state_t global

## 📊 Comparación de Modos

| Modo | Intervalo Publicación | Muestras | Reducción Datos | Uso |
|------|---------------------|----------|-----------------|-----|
| **Instant** | 4s | 1 | 0% | Tiempo real, calibración |
| **Average 5min** | 5min | 75 | 98.7% | Monitoreo continuo |
| **Median 30min** | 30min | 450 | 99.8% | Ahorro máximo, estable |
| **Last 1min** | 1min | 1 | 93.3% | Producción, alertas |

## 🛠️ Implementación

### Flujo de Datos

```
sensor_task (cada sample_interval_ms):
    ├─> Leer sensores (pH, temp)
    ├─> data_aggregator_add_sample()
    └─> ¿data_aggregator_should_publish()?
         ├─ SÍ: data_aggregator_get_result()
         │      └─> uros_manager_publish_sensor_data()
         └─ NO: Continuar acumulando
```

### Estados del Agregador

1. **Acumulación**: Recibe muestras hasta alcanzar N o timeout
2. **Cálculo**: Aplica función según modo (mediana, promedio, etc.)
3. **Publicación**: Envía resultado agregado a ROS2
4. **Limpieza**: Vacía buffer y reinicia ciclo

### Validaciones

#### Config Manager valida:
```c
// Intervalos válidos
MIN_SAMPLE_INTERVAL_MS = 1000 (1s)
MAX_SAMPLE_INTERVAL_MS = 60000 (1min)
MIN_PUBLISH_INTERVAL_MS = 1000 (1s)
MAX_PUBLISH_INTERVAL_MS = 3600000 (1h)

// Coherencia
publish_interval_ms >= sample_interval_ms

// Muestras
1 <= samples_per_publish <= 450
```

## 🔐 Seguridad y Robustez

### Thread-Safety
- **Data Aggregator**: FreeRTOS mutex con timeout 100ms
- **App State**: Mutex global para sensor_config
- **Sin deadlocks**: Timeouts en todas las operaciones

### Manejo de Buffer
- Buffer circular: Si lleno, elimina muestra más antigua
- Máximo 64 muestras (suficiente para 30min @ 4s con margen)
- Limpieza automática después de publicar

### Validación de Entrada
- JSON parsing robusto con cJSON
- Validación de rangos antes de aplicar
- Rollback automático si validación falla

## 📈 Ventajas del Sistema

### 1. **Eficiencia de Red**
- **Modo 30min mediana**: 99.8% menos mensajes
- Reducción de ancho de banda proporcional
- Menor carga en Gateway y base de datos

### 2. **Robustez Estadística**
- **Mediana**: Resistente a outliers (picos de ruido)
- **Promedio**: Suaviza fluctuaciones naturales
- Calidad de datos superior vs lecturas individuales

### 3. **Flexibilidad Operacional**
- Cambio dinámico sin recompilar firmware
- Adaptación a fases del cultivo:
  - Inicio: Monitoreo intensivo (4s)
  - Estable: Ahorro de datos (30min)
  - Crítico: Alertas rápidas (1min)

### 4. **Escalabilidad**
- Soporta hasta 450 muestras (30min @ 4s)
- Memoria estática (sin malloc en runtime)
- CPU eficiente: Cálculos solo al publicar

## 🚀 Uso con ROS2 (Futuro)

### Python Script de Configuración
```python
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String
import json

def configure_device(mode='normal'):
    node = rclpy.create_node('biofloc_configurator')
    pub = node.create_publisher(String, '/biofloc/config_cmd', 10)
    
    configs = {
        'normal': {
            'sample_interval_ms': 4000,
            'publish_interval_ms': 4000,
            'mode': 'instant',
            'samples_per_publish': 1
        },
        'ahorro': {
            'sample_interval_ms': 4000,
            'publish_interval_ms': 1800000,  # 30 min
            'mode': 'median',
            'samples_per_publish': 450
        },
        'monitoreo': {
            'sample_interval_ms': 4000,
            'publish_interval_ms': 300000,  # 5 min
            'mode': 'average',
            'samples_per_publish': 75
        }
    }
    
    msg = String()
    msg.data = json.dumps({
        'device_id': 'biofloc_esp32_c8e0',
        'action': 'set_config',
        'config': configs[mode]
    })
    
    pub.publish(msg)
    print(f"✓ Configuración '{mode}' enviada")

if __name__ == '__main__':
    rclpy.init()
    configure_device('ahorro')
    rclpy.shutdown()
```

### Comandos CLI
```bash
# Modo ahorro de datos (30 min mediana)
ros2 topic pub /biofloc/config_cmd std_msgs/String \
  "{data: '{\"device_id\":\"biofloc_esp32_c8e0\",\"action\":\"set_config\",\"config\":{\"sample_interval_ms\":4000,\"publish_interval_ms\":1800000,\"mode\":\"median\",\"samples_per_publish\":450,\"enabled\":true}}'}"

# Escuchar respuesta
ros2 topic echo /biofloc/config_status
```

## 📝 Estado de Implementación

### ✅ COMPLETADO
- [x] Enum `data_aggregation_mode_t` en types.h
- [x] Struct `sensor_config_t` en types.h
- [x] Integración en `app_state_t`
- [x] Data Aggregator completo:
  - [x] Buffer circular thread-safe
  - [x] Cálculo de mediana (qsort)
  - [x] Cálculo de promedio
  - [x] Gestión de ventanas de tiempo
- [x] Config Manager completo:
  - [x] Validación de configuración
  - [x] Aplicación thread-safe
  - [x] Defaults profesionales
- [x] Compilación exitosa
- [x] Documentación completa

### 🚧 PENDIENTE (Integración)
- [ ] Subscriber `/biofloc/config_cmd` en uros_manager
- [ ] Callback config_manager_command_callback()
- [ ] Publisher `/biofloc/config_status` para ACK/NACK
- [ ] Integración en sensor_task:
  - [ ] Usar app_state->sensor_config.sample_interval_ms
  - [ ] Llamar data_aggregator_add_sample()
  - [ ] Publicar solo cuando data_aggregator_should_publish()
- [ ] Persistencia en NVS (opcional)
- [ ] Python script de configuración
- [ ] Pruebas con hardware

## 🎓 Principios Aplicados

### SOLID
- **Single Responsibility**: 
  - Data Aggregator: Solo agrega datos
  - Config Manager: Solo gestiona configuración
- **Open/Closed**: Fácil agregar nuevos modos (ej: MODE_WEIGHTED_AVG)
- **Dependency Inversion**: Ambos dependen de abstracciones (app_state)

### Clean Code
- Nombres descriptivos: `data_aggregator_should_publish()`
- Funciones pequeñas: `calculate_median()`, `validate_config()`
- Comentarios significativos: Casos de uso documentados
- Sin magic numbers: Constantes con nombres (MAX_SAMPLES_BUFFER)

### Seguridad
- Validación estricta de entrada
- Protección contra buffer overflow
- Thread-safety con mutexes
- Timeouts para prevenir deadlocks

## 📚 Referencias

- **ESP-IDF**: [FreeRTOS Semaphores](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos_idf.html)
- **Estadística Robusta**: Mediana vs Media para datos con outliers
- **ROS2 Parameters**: [Dynamic Reconfigure](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- **Clean Code**: Robert C. Martin - Principios de diseño profesional

---

**Versión**: 4.0.0  
**Autor**: Biofloc Team  
**Fecha**: Febrero 2026  
**Estado**: Módulos core completos, integración pendiente
