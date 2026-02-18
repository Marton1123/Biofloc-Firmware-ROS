/**
 * @file ARCHITECTURE.md
 * @brief Biofloc Firmware v4.0.0 - Arquitectura Modular Profesional
 * @date 2026-02-18
 */

# Arquitectura Biofloc Firmware v4.0.0

## Visión General

Firmware ESP32 refactorizado con arquitectura en capas, siguiendo principios **SOLID**, **Clean Code** y **separación de responsabilidades**.

## Estructura de Directorios

```
main/
├── core/                    # Núcleo - Estado y configuración
│   ├── config.h            # Constantes centralizadas (cero magic numbers)
│   ├── types.h             # Tipos compartidos (Single Source of Truth)
│   ├── app_state.h/.c      # Estado global thread-safe con mutex
│
├── drivers/                 # Capa de drivers hardware
│   ├── wifi_manager.h/.c   # Gestión WiFi (conexión, reconexión, estado)
│   ├── watchdog_manager.h/.c # Gestión watchdog (alimentación, timeouts)
│
├── middleware/              # Lógica de negocio
│   ├── uros/               # micro-ROS (publishers, subscribers, executor)
│   │   └── uros_manager.h/.c
│   └── calibration/        # Calibración (parsing JSON, ejecución)
│       └── calibration_controller.h/.c
│
├── hal/                     # Hardware Abstraction Layer
│   ├── sensors/            # Interfaz genérica de sensores
│   │   └── sensor_interface.h/.c
│   └── storage/            # Abstracción NVS
│       └── nvs_storage.h/.c
│
├── app/                     # Tareas de aplicación
│   ├── sensor_task.h/.c    # Task de lectura de sensores
│   └── uros_task.h/.c      # Task de comunicación micro-ROS
│
├── main.c                   # Orquestador principal (<200 líneas)
├── sensors.c/.h             # LEGACY - A refactorizar
└── CMakeLists.txt
```

## Capas y Responsabilidades

### 1. Core (`core/`)

**Responsabilidad**: Configuración y estado global compartido.

- **config.h**: 
  - Todas las constantes del sistema (timeouts, buffers, pines GPIO)
  - Configuración WiFi/micro-ROS
  - Validación en tiempo de compilación (`#error` si inválido)
  
- **types.h**: 
  - Tipos compartidos entre módulos
  - Enums de estado (WiFi, micro-ROS, calibración)
  - Structs de datos (sensores, calibración, dispositivo)
  
- **app_state.h/.c**: 
  - Estado global thread-safe (mutex FreeRTOS)
  - API para modificar estado desde cualquier tarea
  - Control de modo calibración (pause/resume sensor_data)

### 2. Drivers (`drivers/`)

**Responsabilidad**: Interfaz con hardware/servicios de bajo nivel.

- **wifi_manager**: 
  - Inicialización WiFi STA mode
  - Conexión con retry automático
  - Reconexión en caso de pérdida
  - API: `wifi_manager_init()`, `wifi_manager_get_info()`, `wifi_manager_reconnect_forever()`

- **watchdog_manager** (pendiente):
  - Configuración hardware watchdog
  - Alimentación periódica desde tareas
  - Prevención de zombie states

### 3. Middleware (`middleware/`)

**Responsabilidad**: Lógica de negocio y orquestación.

- **uros_manager**: 
  - Setup micro-ROS (support, node, allocator)
  - Creación publishers (`/biofloc/sensor_data`, `/biofloc/calibration_status`)
  - Creación subscribers (`/biofloc/calibration_cmd`)
  - Executor spin
  - Ping Agent y reconexión
  - API: `uros_manager_init()`, `uros_manager_publish_sensor_data()`, `uros_manager_spin_once()`

- **calibration_controller** (pendiente):
  - Parsing JSON de comandos calibración
  - Ejecución acciones (calibrate, query, reset)
  - Generación respuestas JSON
  - Validación entrada (buffer overflow protection)

### 4. HAL (`hal/`)

**Responsabilidad**: Abstracción de hardware específico.

- **sensor_interface** (pendiente):
  - Interfaz genérica para sensores
  - Implementaciones: pH, temperatura, DO, conductividad
  - Calibración por sensor
  - Lectura thread-safe

- **nvs_storage** (pendiente):
  - Abstracción NVS para calibración
  - Save/load calibration params
  - Gestión de errores NVS

### 5. App (`app/`)

**Responsabilidad**: Tareas FreeRTOS de aplicación.

- **sensor_task** (pendiente):
  - Loop lectura sensores cada SAMPLE_INTERVAL
  - Serialización JSON
  - Publicación vía uros_manager
  - Pause/resume según app_state

- **uros_task** (pendiente):
  - Loop executor spin
  - Ping periódico Agent
  - Reconexión automática
  - Watchdog feed

### 6. Main (`main.c`)

**Responsabilidad**: Orquestación de alto nivel.

**Target**: <200 líneas - solo setup e inicialización de módulos.

```c
void app_main(void) {
    // 1. Init core
    app_state_init();
    
    // 2. Init drivers
    wifi_manager_init();
    watchdog_manager_init();
    
    // 3. Init middleware
    uros_manager_init();
    calibration_controller_init();
    
    // 4. Start app tasks
    sensor_task_start();
    uros_task_start();
    
    // Done - FreeRTOS scheduler takes over
}
```

## Principios de Diseño

### SOLID

- **S**ingle Responsibility: Cada módulo tiene una responsabilidad única
- **O**pen/Closed: Extensible sin modificar código existente
- **L**iskov Substitution: HAL permite cambiar implementaciones
- **I**nterface Segregation: APIs mínimas y específicas
- **D**ependency Inversion: Depender de abstracciones (HAL), no implementaciones

### Clean Code

- ❌ Magic numbers → ✅ Constantes en `config.h`
- ❌ Funciones >50 líneas → ✅ Funciones pequeñas y descriptivas
- ❌ Estado global desprotegido → ✅ Estado con mutex
- ❌ Nombres crípticos → ✅ Nombres autodocumentados
- ❌ Código duplicado → ✅ DRY (Don't Repeat Yourself)

### Seguridad

- **Buffer overflow protection**: Todos los buffers con tamaño fijo y validación
- **Watchdog always-on**: Sistema se resetea si alguna tarea se bloquea >20s
- **Input validation**: JSON parsing con límites estrictos
- **Thread-safety**: Acceso a estado compartido con mutex

## Estado de Migración

### ✅ Completado (v4.0.0)

- [x] `core/config.h` - Constantes centralizadas
- [x] `core/types.h` - Tipos compartidos
- [x] `core/app_state.c/.h` - Estado thread-safe
- [x] `drivers/wifi_manager.c/.h` - Gestión WiFi
- [x] Compilación exitosa
- [x] Flasheo y validación en hardware

### 🚧 En Progreso

- [ ] `middleware/uros/uros_manager.c` - Implementación completa
- [ ] Integración en main.c

### ⏳ Pendiente

- [ ] `middleware/calibration/calibration_controller.c/.h`
- [ ] `drivers/watchdog_manager.c/.h`
- [ ] `hal/sensors/sensor_interface.c/.h`
- [ ] `hal/storage/nvs_storage.c/.h`
- [ ] `app/sensor_task.c/.h`
- [ ] `app/uros_task.c/.h`
- [ ] Refactorizar `sensors.c` → HAL
- [ ] Reducir `main.c` a <200 líneas

## Ventajas de la Nueva Arquitectura

### Mantenibilidad
- **Código organizado**: Fácil encontrar y modificar funcionalidad
- **Modular**: Cambios en un módulo no afectan otros
- **Autodocumentado**: Estructura clara refleja flujo del sistema

### Debuggabilidad
- **Aislamiento**: Errores contenidos en módulo específico
- **Logging estructurado**: Tags por módulo (TAG_WIFI, TAG_UROS, TAG_SENSOR)
- **Estado visible**: app_state_get() permite inspeccionar estado en cualquier momento

### Extensibilidad
- **Nuevos sensores**: Implementar sensor_interface sin tocar main
- **Nuevos protocolos**: Agregar managers sin modificar core
- **Nuevas features**: Plug-and-play en middleware

### Testabilidad
- **Mocks fáciles**: APIs limpias permiten crear stubs
- **Unit testing**: Módulos independientes testeables
- **Integration testing**: HAL permite simular hardware

## Migración Incremental

**Estrategia**: No breaking changes - compilar después de cada fase.

1. **FASE 1** (✅): Core + WiFi Manager
2. **FASE 2** (🚧): micro-ROS Manager + Calibration Controller
3. **FASE 3**: HAL Sensors + NVS Storage
4. **FASE 4**: App Tasks
5. **FASE 5**: Simplificar main.c

## Comparación v3.6.5 vs v4.0.0

| Aspecto | v3.6.5 | v4.0.0 |
|---------|---------|---------|
| **Líneas main.c** | 993 | <200 (target) |
| **Magic numbers** | ~50 | 0 |
| **Archivos** | 3 | ~15 |
| **Módulos** | 0 | 8 |
| **Thread-safety** | ❌ | ✅ mutex |
| **Testeable** | ❌ | ✅ |
| **Extensible** | ❌ | ✅ HAL |

## Referencias

- ESP-IDF v5.3.4: https://docs.espressif.com/projects/esp-idf/
- micro-ROS Jazzy: https://micro.ros.org/
- Clean Code (Robert C. Martin)
- SOLID Principles (Robert C. Martin)
