# Arquitectura Digital Twin - Respaldo de Calibración en MongoDB

## 📋 Resumen

Este documento describe la arquitectura **Digital Twin** implementada para el sistema Biofloc, donde MongoDB Atlas sirve como "fuente única de verdad" para los datos de calibración, proporcionando respaldo profesional más allá del NVS (Non-Volatile Storage) del ESP32.

## 🏗️ Arquitectura

```
┌─────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI 4                           │
│                      (Gateway + Manager)                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  CLI Manager (biofloc_manager.py)                         │   │
│  │                                                            │   │
│  │  1. Leer voltajes de sensores (rclpy nativo)              │   │
│  │  2. Enviar comando calibración → /biofloc/calibration_cmd │   │
│  │  3. Esperar respuesta → /biofloc/calibration_status       │   │
│  │  4. Guardar en MongoDB (Digital Twin) ✨                   │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  micro-ROS Agent                                           │   │
│  │  - Puente DDS ↔ micro-ROS                                 │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
                               ↓
                    WiFi Hotspot (10.42.0.0/24)
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│                          ESP32-WROOM-32D                         │
│                      (MAC: 24:0a:c4:60:c8:e0)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Firmware v3.2.0                                           │   │
│  │                                                            │   │
│  │  - Suscrito a /biofloc/calibration_cmd                    │   │
│  │  - Parseo JSON con cJSON (validado)                       │   │
│  │  - Regresión lineal                                        │   │
│  │  - Guardar en NVS (almacenamiento primario)               │   │
│  │  - Publicar respuesta → /biofloc/calibration_status       │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
                               ↓
                         Internet (4G)
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│                        MongoDB Atlas                             │
│                     (Base de Datos en la Nube)                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  Database: SistemasLab                                           │
│                                                                   │
│  Collection: devices                                             │
│  {                                                               │
│    "_id": "biofloc_esp32_c8e0",                                  │
│    "alias": "Esp-32 MicroAlgas Martin",                          │
│    "calibracion": {                                              │
│      "ph": {                                                     │
│        "fecha": "2026-02-17T12:45:00Z",                          │
│        "slope": 6.2558,                                          │
│        "offset": -8.6328,                                        │
│        "r_squared": 0.9997,                                      │
│        "points": [                                               │
│          {"voltage": 2.053, "value": 4.46},                      │
│          {"voltage": 2.49, "value": 7.2},                        │
│          {"voltage": 2.914, "value": 9.85}                       │
│        ],                                                        │
│        "num_points": 3,                                          │
│        "status": "success",                                      │
│        "message": "Calibration successful"                       │
│      },                                                          │
│      "temperature": { ... }                                      │
│    },                                                            │
│    "ultima_calibracion": "2026-02-17T12:45:00Z"                  │
│  }                                                               │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## 🔄 Flujo de Calibración

### 1. **Leer Voltaje** (rclpy nativo - sin subprocess)
```python
voltage = read_sensor_voltage(device_id, sensor_type)
# Usa VoltageReader(Node) - elimina problemas de timeout con bash
```

### 2. **Enviar Comando de Calibración** (topic ROS 2)
```bash
ros2 topic pub --once /biofloc/calibration_cmd std_msgs/msg/String \
  "data: '{\"sensor\":\"ph\",\"action\":\"calibrate\",\"points\":[...]}'"
```

### 3. **Procesamiento ESP32** (C++ con validaciones)
```c
// main/main.c - calibration_callback()
- Validar tamaño del mensaje (< 512 bytes)
- Parsear JSON con cJSON (chequeos NULL)
- Validar límites del array (2-5 puntos)
- Llamar sensors_calibrate_generic()
- Guardar en NVS
- Publicar respuesta
```

### 4. **Recibir Respuesta** (topic ROS 2)
```bash
ros2 topic echo --once /biofloc/calibration_status
```

### 5. **Guardar en MongoDB** ✨ (Digital Twin)
```python
save_calibration_to_mongodb(calibration_request, calibration_response)
# Actualiza la colección devices con los datos de calibración
```

## 📊 Esquema MongoDB

### Colección: `devices`

```json
{
  "_id": "biofloc_esp32_c8e0",
  "alias": "Esp-32 MicroAlgas Martin",
  "location": "tanque_01",
  "firmware_version": "v3.2.0",
  "sensores_habilitados": ["ph", "temperatura"],
  
  "calibracion": {
    "ph": {
      "fecha": "2026-02-17T12:45:00.000Z",
      "slope": 6.2558,
      "offset": -8.6328,
      "r_squared": 0.9997,
      "points": [
        {"voltage": 2.053, "value": 4.46},
        {"voltage": 2.49, "value": 7.2},
        {"voltage": 2.914, "value": 9.85}
      ],
      "num_points": 3,
      "status": "success",
      "message": "Calibration successful"
    },
    
    "temperature": {
      "fecha": "2026-02-15T10:30:00.000Z",
      "slope": 1.0,
      "offset": 0.0,
      "r_squared": 1.0,
      "points": [
        {"voltage": 1.5, "value": 25.0}
      ],
      "num_points": 1,
      "status": "success",
      "message": "Single-point calibration"
    }
  },
  
  "ultima_calibracion": "2026-02-17T12:45:00.000Z",
  
  "umbrales": {
    "ph": {
      "min": 6.5,
      "max": 8.5,
      "critico_min": 6.0,
      "critico_max": 9.0
    },
    "temperature": {
      "min": 20.0,
      "max": 30.0,
      "critico_min": 15.0,
      "critico_max": 35.0
    }
  },
  
  "conexion": {
    "ultima": "2026-02-17T12:45:23.694665",
    "total_lecturas": 260333,
    "ip_address": "10.42.0.123"
  }
}
```

## 🛡️ Beneficios del Digital Twin

### 1. **Respaldo Persistente**
- El NVS puede corromperse (pérdida de energía, desgaste de flash, actualizaciones de firmware)
- MongoDB proporciona respaldo en la nube con replicación automática
- Historial de calibración preservado incluso si el ESP32 es reemplazado

### 2. **Mecanismo de Recuperación**
```python
# Característica futura: Restaurar calibración desde la nube
def restore_calibration_from_mongodb(device_id, sensor_type):
    # Leer calibración desde MongoDB
    # Republicar al ESP32
    # Verificar actualización del NVS
```

### 3. **Análisis y Monitoreo**
- Rastrear frecuencia de calibración
- Detectar deriva del sensor (comparar historial de calibración)
- Alertar si R² cae por debajo del umbral

### 4. **Gestión Multi-Dispositivo**
- Calibración centralizada para todos los dispositivos
- Comparar parámetros de calibración entre sensores
- Detectar anomalías (slope/offset atípicos)

## 🔧 Implementación

### Cambios ESP32 (main/main.c)

**Bugs corregidos que causaban PANIC**:

1. **Agregados chequeos NULL** para punteros cJSON
```c
cJSON *point = cJSON_GetArrayItem(points_json, i);
if (!point) {
    ESP_LOGE(TAG_UROS, "NULL point at index %d", i);
    parse_error = true;
    break;
}
```

2. **Validar cJSON_IsNumber** antes de acceder al valor
```c
if (!voltage || !cJSON_IsNumber(voltage) || !value || !cJSON_IsNumber(value)) {
    ESP_LOGE(TAG_UROS, "Invalid point: missing or non-numeric voltage/value");
    parse_error = true;
    break;
}
```

3. **Inicialización a cero de calibration_response_t**
```c
calibration_response_t cal_response;
memset(&cal_response, 0, sizeof(cal_response));
```

4. **Agregado logging detallado**
```c
ESP_LOGI(TAG_UROS, "  Point %d: %.3fV → %.2f pH", i+1, voltage, value);
ESP_LOGI(TAG_UROS, "✓ Calibration SUCCESS: R²=%.4f", r_squared);
```

### Cambios Python (biofloc_manager.py)

**Nueva función: save_calibration_to_mongodb()**

```python
def save_calibration_to_mongodb(calibration_request, calibration_response):
    """
    Guardar datos de calibración en la colección devices de MongoDB (Digital Twin)
    
    Args:
        calibration_request: Comando de calibración original (dict)
        calibration_response: Respuesta del ESP32 con parámetros calculados (dict)
    
    Returns:
        bool: True si se guardó exitosamente, False en caso contrario
    """
    from pymongo import MongoClient
    
    # Obtener credenciales de MongoDB desde .env
    mongodb_uri = os.getenv('MONGODB_URI')
    mongodb_database = os.getenv('MONGODB_DATABASE', 'SistemasLab')
    mongodb_devices_collection = os.getenv('MONGODB_COLLECTION_DEVICES', 'devices')
    
    # Conectar a MongoDB
    client = MongoClient(mongodb_uri, serverSelectionTimeoutMS=5000)
    db = client[mongodb_database]
    devices_col = db[mongodb_devices_collection]
    
    # Preparar documento de calibración
    sensor_type = calibration_request.get('sensor')
    calibration_doc = {
        'fecha': datetime.utcnow().isoformat() + 'Z',
        'slope': calibration_response.get('slope'),
        'offset': calibration_response.get('offset'),
        'r_squared': calibration_response.get('r_squared'),
        'points': calibration_request.get('points', []),
        'num_points': len(calibration_request.get('points', [])),
        'status': calibration_response.get('status'),
        'message': calibration_response.get('message')
    }
    
    # Actualizar documento del dispositivo
    devices_col.update_one(
        {'_id': device_id},
        {
            '$set': {
                f'calibracion.{sensor_type}': calibration_doc,
                'ultima_calibracion': datetime.utcnow().isoformat() + 'Z'
            }
        },
        upsert=False
    )
```

**Integrado en publish_calibration_command()**:

```python
if response_json.get('status') == 'success':
    print_success(f"✓ {response_json.get('message')}")
    print_info(f"  Slope: {response_json['slope']:.6f}")
    print_info(f"  Offset: {response_json['offset']:.6f}")
    print_info(f"  R²: {response_json['r_squared']:.4f}")
    
    # ===== DIGITAL TWIN: Guardar en MongoDB =====
    save_calibration_to_mongodb(cal_data, response_json)
    
    return True
```

## 📝 Variables de Entorno

Requeridas en `.env` o `scripts/.env`:

```bash
# Conexión MongoDB Atlas
MONGODB_URI=mongodb+srv://username:password@cluster.mongodb.net/
MONGODB_DATABASE=SistemasLab
MONGODB_COLLECTION=telemetria
MONGODB_COLLECTION_DEVICES=devices
MONGODB_COLLECTION_SYSTEM_HEALTH=system_health

# Configuración del dispositivo
ESP32_MAC=24:0a:c4:60:c8:e0
GATEWAY_IP=10.42.0.1
GATEWAY_NETWORK=10.42.0.0/24
```

## 🧪 Pruebas

### 1. Probar Flujo de Calibración
```bash
cd /home/Biofloc-Firmware-ROS
./biofloc_manager.py

# Menú:
# 7) Calibración remota de pH (3 puntos)
#    → Leer voltajes con soluciones buffer (4.46, 7.0, 9.85)
#    → Enviar comando de calibración
#    → Esperar respuesta del ESP32
#    → Verificar guardado en MongoDB
```

### 2. Verificar Documento MongoDB
```python
from pymongo import MongoClient
import os
from dotenv import load_dotenv

load_dotenv('scripts/.env')
client = MongoClient(os.getenv('MONGODB_URI'))
db = client['SistemasLab']
devices = db['devices']

device = devices.find_one({'_id': 'biofloc_esp32_c8e0'})
print(device['calibracion']['ph'])
```

Salida esperada:
```json
{
  "fecha": "2026-02-17T12:45:00.000Z",
  "slope": 6.2558,
  "offset": -8.6328,
  "r_squared": 0.9997,
  "points": [
    {"voltage": 2.053, "value": 4.46},
    {"voltage": 2.49, "value": 7.2},
    {"voltage": 2.914, "value": 9.85}
  ],
  "num_points": 3,
  "status": "success",
  "message": "Calibration successful"
}
```

### 3. Monitorear Logs del ESP32
```bash
python3 biofloc_manager.py
# Menú: 4) Monitorear ESP32 (logs en vivo)

# Salida esperada:
[TAG_UROS] Received calibration command (154 bytes): {"sensor":"ph","action":"calibrate","points":[...]}
[TAG_UROS]   Point 1: 2.053V → 4.46 pH
[TAG_UROS]   Point 2: 2.490V → 7.20 pH
[TAG_UROS]   Point 3: 2.914V → 9.85 pH
[TAG_UROS] Starting calibration for ph with 3 points
[TAG_UROS] ✓ Calibration SUCCESS: R²=0.9997, slope=6.2558, offset=-8.6328
[TAG_UROS] Calibration response sent
```

## 🚨 Solución de Problemas

### Problema: "MONGODB_URI no configurado"
**Solución**: Crear archivo `.env` con credenciales de MongoDB

```bash
cp scripts/.env.example scripts/.env
nano scripts/.env  # Agregar MONGODB_URI
```

### Problema: ESP32 todavía crashea con PANIC
**Causas posibles**:
1. **Stack overflow** - Verificar `uxTaskGetStackHighWaterMark(NULL)`
2. **NVS lleno** - Ejecutar `idf.py erase-flash` para limpiar NVS
3. **Watchdog timeout** - Aumentar timeout en sdkconfig

**Debug**:
```bash
# Monitorear logs del ESP32 con timestamps
python3 biofloc_manager.py
# Menú: 4) Monitorear ESP32

# Buscar:
# - Errores de "Stack overflow"
# - Errores de "NVS full"
# - Watchdog timeout (TWDT)
```

### Problema: Timeout de conexión MongoDB
**Solución**: Verificar conectividad de red y credenciales

```python
from pymongo import MongoClient
import os
from dotenv import load_dotenv

load_dotenv('scripts/.env')
client = MongoClient(os.getenv('MONGODB_URI'), serverSelectionTimeoutMS=5000)

try:
    client.admin.command('ping')
    print("✓ Conectado a MongoDB Atlas")
except Exception as e:
    print(f"✗ Falló la conexión: {e}")
```

## 📚 Referencias

- [Documentación MongoDB Atlas](https://www.mongodb.com/docs/atlas/)
- [Documentación pymongo](https://pymongo.readthedocs.io/)
- [Documentación micro-ROS](https://micro.ros.org/)
- [ESP-IDF NVS Storage](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html)

## 🔮 Mejoras Futuras

### 1. Historial de Calibración
Almacenar array de calibraciones con timestamps:
```json
"calibracion_history": {
  "ph": [
    {
      "fecha": "2026-02-17T12:45:00Z",
      "slope": 6.2558,
      "offset": -8.6328,
      "r_squared": 0.9997
    },
    {
      "fecha": "2026-02-10T10:30:00Z",
      "slope": 6.2012,
      "offset": -8.5123,
      "r_squared": 0.9995
    }
  ]
}
```

### 2. Recuperación Automática
Restaurar calibración desde MongoDB si el NVS está corrupto:
```python
def auto_restore_calibration(device_id):
    # Verificar si ESP32 tiene calibración válida (R² > 0.95)
    # Si no, restaurar desde MongoDB
    # Republicar comando de calibración
```

### 3. Detección de Deriva
Alertar si los parámetros de calibración cambian significativamente:
```python
def detect_sensor_drift(device_id, sensor_type):
    # Comparar últimas N calibraciones
    # Alertar si slope cambia > 10%
    # Alertar si R² cae por debajo del umbral
```

### 4. Comparación Multi-Punto
Comparar curvas de calibración entre dispositivos:
```python
def compare_calibrations(sensor_type):
    # Obtener todas las calibraciones de dispositivos para el sensor
    # Graficar curvas de calibración
    # Detectar outliers
```

---

**Autor**: [@Marton1123](https://github.com/Marton1123)  
**Versión**: 1.0.0  
**Última Actualización**: 17 de febrero de 2026  
**Estado**: Listo para Producción ✅
