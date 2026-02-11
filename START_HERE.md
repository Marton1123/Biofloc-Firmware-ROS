# 🚀 START HERE - Biofloc Firmware ROS v3.0.0

**¿Nuevo en este proyecto? Empieza aquí.**

**Autor:** [@Marton1123](https://github.com/Marton1123)  
**Repositorio:** [Biofloc-Firmware-ROS](https://github.com/Marton1123/Biofloc-Firmware-ROS)

---

## ⚡ TL;DR (30 segundos)

```bash
# Ejecutar el gestor unificado
cd /home/Biofloc-Firmware-ROS
python3 biofloc_manager.py
```

**Eso es todo.** El gestor tiene 12 opciones para hacer TODO.

---

## 🎯 ¿Qué es este proyecto?

Sistema de telemetría IoT para monitoreo de pH y temperatura en acuicultura:
- **ESP32** lee sensores y publica datos vía micro-ROS
- **Gateway** (NUC Linux) recibe datos y los guarda en MongoDB Atlas
- **Arquitectura segura**: ESP32 sin acceso a internet (aislado por firewall)

---

## 📚 Documentación por Objetivo

### "Quiero ejecutar el sistema AHORA"
👉 **[docs/guides/GUIA_PASO_A_PASO.md](docs/guides/GUIA_PASO_A_PASO.md)**
- Tiempo: 5-10 minutos
- Incluye: Configuración, ejecución, verificación

### "Quiero entender cómo funciona"
👉 **[README.md](README.md)**
- Tiempo: 15 minutos
- Incluye: Arquitectura, características, requisitos

### "Tengo un proyecto antiguo y quiero migrarlo"
👉 **[docs/guides/MIGRATION_GUIDE_SECURE_GATEWAY.md](docs/guides/MIGRATION_GUIDE_SECURE_GATEWAY.md)**
- Tiempo: 2-3 horas (migración completa)
- Incluye: Configuración de gateway, cambios en código, checklist

### "¿Qué cambió en v3.0.0?"
👉 **[docs/releases/RELEASE_NOTES_v3.0.0.md](docs/releases/RELEASE_NOTES_v3.0.0.md)**
- Tiempo: 10 minutos
- Incluye: Resumen ejecutivo, breaking changes, migración

### "Necesito detalles técnicos"
👉 **[docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md)**
- Tiempo: 20 minutos
- Incluye: Configuración, calibración, hardware

### "Algo no funciona"
👉 **[docs/guides/TROUBLESHOOTING.md](docs/guides/TROUBLESHOOTING.md)**
- O usa: `python3 biofloc_manager.py` → Opción [4] o [5]

---

## 🗺️ Mapa de Documentación Visual

```
START HERE.md (ESTÁS AQUÍ)
    │
    ├─► ¿Primera vez?
    │   └─► README.md (15 min)
    │       └─► docs/guides/GUIA_PASO_A_PASO.md (30 min)
    │           └─► biofloc_manager.py (usar)
    │
    ├─► ¿Migrando desde v2.x?
    │   └─► docs/releases/RELEASE_NOTES_v3.0.0.md (10 min)
    │       └─► docs/guides/MIGRATION_GUIDE_SECURE_GATEWAY.md (2h)
    │           └─► docs/guides/GUIA_PASO_A_PASO.md (verificar)
    │
    ├─► ¿Otro proyecto ESP32?
    │   └─► docs/guides/MIGRATION_GUIDE_SECURE_GATEWAY.md (guía genérica)
    │       └─► Usar prompt para IA incluido
    │
    ├─► ¿Desarrollador/técnico?
    │   └─► docs/technical/TECHNICAL_SUMMARY.md (detalles)
    │       ├─► main/sensors.h (API)
    │       └─► sdkconfig.defaults (configuración)
    │
    └─► ¿Problema/error?
        └─► biofloc_manager.py → [4] o [5]
            └─► docs/guides/TROUBLESHOOTING.md
```

---

## 📋 Archivos Principales del Proyecto

### Código del Sistema
```
main/
├── main.c                    # Punto de entrada, WiFi, micro-ROS
├── sensors.c                 # Lectura y calibración de sensores
└── sensors.h                 # API de sensores

components/
└── micro_ros_espidf_component/  # Cliente micro-ROS

scripts/
├── biofloc_manager.py        # 🌟 GESTOR UNIFICADO (USA ESTO)
├── sensor_db_bridge.py       # Puente ROS → MongoDB
├── monitor_sensores.py       # Monitor en tiempo real
├── calibrate_ph.py           # Calibración pH
└── calibrate_temperature.py  # Calibración temperatura
```

### Configuración
```
sdkconfig.defaults            # 🌟 Configuración firmware ESP32
.env                          # 🌟 Credenciales y variables (MongoDB, ROS, Gateway)
.env.example                  # Template de configuración

config/
└── README.md                 # Guía de configuración
```

### Documentación
```
README.md                     # Visión general
START_HERE.md                 # 🌟 PUNTO DE ENTRADA (aquí)
QUICKSTART.md                 # Instalación rápida
DOCUMENTATION_INDEX.md        # 🌟 ÍNDICE COMPLETO

docs/
├── guides/
│   ├── GUIA_PASO_A_PASO.md          # 🌟 USO DIARIO
│   ├── MIGRATION_GUIDE_SECURE_GATEWAY.md  # Migración
│   ├── CALIBRATION.md                # Calibración detallada
│   └── TROUBLESHOOTING.md            # Solución de problemas
├── technical/
│   ├── TECHNICAL_SUMMARY.md          # Detalles técnicos
│   ├── PROJECT_STATUS.md             # Estado del proyecto
│   ├── CONFIG_ORGANIZATION_PROPOSAL.md  # Propuesta .env
│   ├── CALIBRATION_SUMMARY.md        # Resumen calibraciones
│   └── PH_CYCLE_ANALYSIS.md          # Análisis de datos
├── releases/
│   ├── RELEASE_NOTES_v3.0.0.md       # Notas v3.0.0
│   └── CHANGELOG.md                  # Historial cambios
└── SECURITY.md                       # Seguridad
```

---

## 🎯 Casos de Uso Comunes

### 1. Iniciar el sistema por primera vez
```bash
python3 biofloc_manager.py
# [1] Iniciar micro-ROS Agent
# [2] Iniciar sensor_db_bridge.py
# [4] Verificar estado del sistema
```

### 2. Verificar que ESP32 está conectado
```bash
python3 biofloc_manager.py
# [5] Verificar conectividad ESP32
```

### 3. Calibrar sensor de pH
```bash
python3 biofloc_manager.py
# [6] Calibración completa pH (3 puntos, 30 min)
# O [8] Ajuste rápido pH (1 valor, 30 seg)
```

### 4. Cambiar WiFi
```bash
python3 biofloc_manager.py
# [10] Configurar WiFi (actualiza DUAL credentials)
```

### 5. Recompilar y flashear firmware
```bash
python3 biofloc_manager.py
# [12] Compilar y Flashear
```

---

## 🔑 Conceptos Clave de v3.0.0

### 1. Arquitectura de Gateway Seguro
- ESP32 **NO tiene** acceso a internet
- Gateway (NUC) tiene doble red: WiFi (ESP32) + Ethernet (Internet)
- Firewall iptables bloquea ESP32 → Internet

### 2. Dual WiFi Credentials
- `CONFIG_ESP_WIFI_*` - Para componente micro_ros
- `CONFIG_BIOFLOC_WIFI_*` - Para aplicación principal
- **Ambos deben ser idénticos**

### 3. Timestamps
- ESP32: Contador (`sample_0001`, `sample_0002`, ...)
- Servidor: Timestamp real (UTC) agregado antes de MongoDB

### 4. Hardware
- R1 = 10kΩ (pull-up)
- R2 = 20kΩ (pull-down)
- Factor = 1.5 (verificado desde PCB)

### 5. Gestor Unificado
- **biofloc_manager.py** es la herramienta principal
- 12 opciones cubren TODO el sistema
- Interfaz 100% en español

---

## 🚨 Si Algo Sale Mal

### ESP32 no se conecta
```bash
python3 biofloc_manager.py
# [5] Verificar conectividad ESP32
# Revisa: Hotspot activo, dual credentials, firewall
```

### Datos no llegan a MongoDB
```bash
python3 biofloc_manager.py
# [4] Verificar estado del sistema
# Debe mostrar: Agent ✅, Bridge ✅, ESP32 publicando ✅
```

### Lecturas incorrectas
```bash
python3 biofloc_manager.py
# [6] Calibración completa pH
# [7] Calibración completa Temperatura
```

### Firewall no funciona
```bash
sudo iptables -L FORWARD -v -n
# Debe mostrar: Chain FORWARD (policy DROP)
```

---

## 💡 Tips Rápidos

1. **Siempre usa el gestor** (`biofloc_manager.py`) primero
2. **sdkconfig.defaults** es la fuente de verdad para configuración
3. **Dual WiFi credentials** deben ser idénticas (usa opción [10] del gestor)
4. **Verificación rápida**: Opción [4] del gestor (8 segundos)
5. **Diagnóstico completo**: Opción [5] del gestor (DHCP/ARP/ping/ROS)

---

## 🎓 Aprendizaje Progresivo

### Nivel 1: Usuario Básico (30 min)
1. Lee [README.md](README.md) - Visión general
2. Lee [GUIA_PASO_A_PASO.md](GUIA_PASO_A_PASO.md) - Uso diario
3. Usa `biofloc_manager.py` - Ejecuta el sistema

### Nivel 2: Usuario Avanzado (2 horas)
1. Lee [TECHNICAL_SUMMARY.md](TECHNICAL_SUMMARY.md) - Detalles técnicos
2. Lee [MIGRATION_GUIDE_SECURE_GATEWAY.md](MIGRATION_GUIDE_SECURE_GATEWAY.md) - Arquitectura
3. Explora `main/sensors.c` - Código del firmware

### Nivel 3: Desarrollador (1 día)
1. Lee [PROJECT_STATUS.md](PROJECT_STATUS.md) - Estado completo
2. Lee [CHANGELOG.md](CHANGELOG.md) - Historial
3. Lee todos los archivos en `docs/`
4. Modifica y experimenta con el código

---

## 🔗 Enlaces Externos Útiles

- **ESP-IDF Programming Guide**: https://docs.espressif.com/projects/esp-idf/en/v5.3.4/
- **ROS 2 Jazzy Documentation**: https://docs.ros.org/en/jazzy/
- **micro-ROS**: https://micro.ros.org/
- **MongoDB Atlas**: https://www.mongodb.com/cloud/atlas

---

## 📞 Ayuda Adicional

**¿Aún perdido?**

1. Revisa [DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md) - Índice completo
2. Busca en [CHANGELOG.md](CHANGELOG.md) - Historial de cambios
3. Revisa [PROJECT_STATUS.md](PROJECT_STATUS.md) - Estado actual

---

**¡Bienvenido al proyecto Biofloc Firmware ROS v3.0.0! 🚀**

Empieza con `python3 biofloc_manager.py` y todo lo demás es fácil.
