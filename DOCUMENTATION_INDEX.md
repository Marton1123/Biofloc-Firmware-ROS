# 📚 Índice de Documentación — Biofloc Firmware ROS v3.0.0

> **Cumplimiento de Estándares:** ISO/IEC 26514 (Documentación de Software) | IEEE 1063 (Documentación de Usuario)

| Metadatos | Valor |
|-----------|-------|
| **Versión Firmware** | 3.0.0 (Secure Gateway) |
| **Versión Gestor** | 1.0.0 (biofloc_manager.py) |
| **Última Actualización** | 2026-02-10 |
| **Autor** | [@Marton1123](https://github.com/Marton1123) |

---

## 📋 Tabla de Contenidos

1. [Referencia Rápida](#-referencia-rápida)
2. [Documentación por Rol](#-documentación-por-rol)
3. [Índice por Tema](#-índice-por-tema)
4. [Rutas de Aprendizaje](#-rutas-de-aprendizaje)
5. [Matriz de Troubleshooting](#-matriz-de-troubleshooting)
6. [Estadísticas de Documentación](#-estadísticas-de-documentación)

---

## ⚡ Referencia Rápida

| Necesidad | Documento | Sección |
|-----------|-----------|---------|
| **🚀 Ejecutar el proyecto** | [docs/guides/GUIA_PASO_A_PASO.md](docs/guides/GUIA_PASO_A_PASO.md) | Completo |
| **🛠️ Gestor unificado** | [biofloc_manager.py](biofloc_manager.py) | Ejecutar `python3 biofloc_manager.py` |
| **🔒 Migrar a gateway seguro** | [docs/guides/MIGRATION_GUIDE_SECURE_GATEWAY.md](docs/guides/MIGRATION_GUIDE_SECURE_GATEWAY.md) | Completo |
| **Instalación inicial** | [QUICKSTART.md](QUICKSTART.md) | Instalación |
| **Sistema no responde** | [docs/guides/TROUBLESHOOTING.md](docs/guides/TROUBLESHOOTING.md) | Procedimiento de Recuperación |
| **Calibración de pH** | [docs/guides/CALIBRATION.md](docs/guides/CALIBRATION.md) | Calibración 3 Puntos |
| **Parámetros actuales** | [docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md) | Sección Calibración |
| **Hardware verificado** | [docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md) | Divisor de Voltaje (R1=10k, R2=20k) |
| **Referencia de API** | [main/sensors.h](main/sensors.h) | Documentación de Funciones |
| **Estado del proyecto** | [docs/technical/PROJECT_STATUS.md](docs/technical/PROJECT_STATUS.md) | Métricas |
| **Guías de seguridad** | [docs/SECURITY.md](docs/SECURITY.md) | Completo |
| **Historial cambios** | [docs/releases/CHANGELOG.md](docs/releases/CHANGELOG.md) | v3.0.0 |
| **Notas de versión** | [docs/releases/RELEASE_NOTES_v3.0.0.md](docs/releases/RELEASE_NOTES_v3.0.0.md) | v3.0.0 |
| **Organización config** | [docs/technical/CONFIG_ORGANIZATION_PROPOSAL.md](docs/technical/CONFIG_ORGANIZATION_PROPOSAL.md) | Estructura .env |

---

## 👥 Documentación por Rol

### 🆕 Usuarios Nuevos — Primeros Pasos
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| **1** | [**docs/guides/GUIA_PASO_A_PASO.md**](docs/guides/GUIA_PASO_A_PASO.md) | **Comandos para ejecutar paso a paso** | **15 min** |
| 2 | [QUICKSTART.md](QUICKSTART.md) | Instalación y primera ejecución | 30 min |
| 3 | [README.md](README.md) | Visión general del sistema | 15 min |
| 4 | [docs/guides/TROUBLESHOOTING.md](docs/guides/TROUBLESHOOTING.md) | Problemas comunes | Referencia |

### 🔬 Técnicos de Calibración
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [docs/guides/GUIA_PASO_A_PASO.md § Calibración](docs/guides/GUIA_PASO_A_PASO.md#4-calibración-del-sensor-de-ph) | Comandos de calibración | 10 min |
| 2 | [docs/guides/CALIBRATION.md](docs/guides/CALIBRATION.md) | Teoría y procedimiento de calibración | 1 hr |
| 3 | [docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md) | Parámetros actuales | 10 min |
| 4 | [calibration_3point_result.txt](calibration_3point_result.txt) | Datos de calibración activa | 5 min |

### � Analistas de Datos / Biólogos
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [docs/technical/PH_CYCLE_ANALYSIS.md](docs/technical/PH_CYCLE_ANALYSIS.md) | Análisis de ciclos circadianos de pH | 45 min |
| 2 | [scripts/check_ph_cycles.py](scripts/check_ph_cycles.py) | Herramienta de análisis automático | 15 min |
| 3 | [scripts/monitor_sensores.py](scripts/monitor_sensores.py) | Monitoreo en tiempo real | 10 min |
| 4 | [config/README.md](config/README.md) | Configuración de BD | 5 min |

### �💻 Desarrolladores de Firmware
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [README.md](README.md) | Visión general de arquitectura | 20 min |
| 2 | [main/sensors.h](main/sensors.h) | Documentación de API | 30 min |
| 3 | [docs/releases/CHANGELOG.md](docs/releases/CHANGELOG.md) | Historial de versiones | 15 min |
| 4 | [docs/SECURITY.md](docs/SECURITY.md) | Guías de seguridad | 15 min |

### 🔧 DevOps / Administradores de Sistemas
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md) | Referencia de configuración | 20 min |
| 2 | [docs/guides/TROUBLESHOOTING.md](docs/guides/TROUBLESHOOTING.md) | Procedimientos de recuperación | Referencia |
| 3 | [docs/technical/PROJECT_STATUS.md](docs/technical/PROJECT_STATUS.md) | Métricas de salud del sistema | 10 min |

### 📊 Gerentes de Proyecto
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [docs/technical/PROJECT_STATUS.md](docs/technical/PROJECT_STATUS.md) | Estado y roadmap | 15 min |
| 2 | [docs/releases/CHANGELOG.md](docs/releases/CHANGELOG.md) | Historial de releases | 10 min |

---

## 📑 Índice por Tema

### Ejecución del Sistema
- [**docs/guides/GUIA_PASO_A_PASO.md**](docs/guides/GUIA_PASO_A_PASO.md) — Comandos paso a paso
- [docs/guides/GUIA_PASO_A_PASO.md § Uso Diario](docs/guides/GUIA_PASO_A_PASO.md#3-uso-diario) — Comandos frecuentes
- [docs/guides/GUIA_PASO_A_PASO.md § Solución de Problemas](docs/guides/GUIA_PASO_A_PASO.md#5-solución-de-problemas-rápida)

### Hardware y Sensores
- [README.md § Requisitos de Hardware](README.md#-requisitos)
- [README.md § Especificaciones Técnicas](README.md#-especificaciones-técnicas)
- [docs/guides/CALIBRATION.md § Sensor CWT-BL](docs/guides/CALIBRATION.md)
- [docs/technical/TECHNICAL_SUMMARY.md § Divisor de Voltaje](docs/technical/TECHNICAL_SUMMARY.md)

### Calibración de pH
- [docs/guides/CALIBRATION.md](docs/guides/CALIBRATION.md) — Guía completa
- [QUICKSTART.md § Calibración](QUICKSTART.md) — Procedimiento rápido
- [calibration_3point_result.txt](calibration_3point_result.txt) — Valores actuales
- [main/sensors.h § API de Calibración](main/sensors.h) — Interfaz de código

### Análisis de Datos y Monitoreo
- [docs/technical/PH_CYCLE_ANALYSIS.md](docs/technical/PH_CYCLE_ANALYSIS.md) — Análisis de ciclos circadianos de pH
- [scripts/check_ph_cycles.py](scripts/check_ph_cycles.py) — Herramienta de análisis automático
- [scripts/monitor_sensores.py](scripts/monitor_sensores.py) — Monitoreo en tiempo real

### Instalación y Configuración
- [QUICKSTART.md § Instalación](QUICKSTART.md) — Paso a paso
- [README.md § Quick Start](README.md#-quick-start) — Resumen
- [docs/technical/TECHNICAL_SUMMARY.md § Configuración](docs/technical/TECHNICAL_SUMMARY.md) — Todos los parámetros
- [main/Kconfig.projbuild](main/Kconfig.projbuild) — Opciones de menuconfig
- [config/README.md](config/README.md) — Guía de configuración .env

### Seguridad
- [docs/SECURITY.md](docs/SECURITY.md) — Guías completas de seguridad
- Gestión de credenciales
- Seguridad de red
- Prácticas de código seguro

### micro-ROS y ROS 2
- [README.md § micro-ROS Agent](README.md)
- [docs/guides/TROUBLESHOOTING.md § Conexión del Agent](docs/guides/TROUBLESHOOTING.md)
- [docs/technical/TECHNICAL_SUMMARY.md § Configuración de Red](docs/technical/TECHNICAL_SUMMARY.md)

### MongoDB y Telemetría
- [README.md § MongoDB Bridge](README.md#-mongodb-bridge-almacenamiento-de-datos)
- [docs/technical/TECHNICAL_SUMMARY.md § Formato de Datos](docs/technical/TECHNICAL_SUMMARY.md)
- [scripts/sensor_db_bridge.py](scripts/sensor_db_bridge.py) — Código fuente del bridge

---

## 🎓 Rutas de Aprendizaje

### Ruta 1: Operador del Sistema (2 horas)
```
QUICKSTART.md → README.md (Visión General) → docs/TROUBLESHOOTING.md → TECHNICAL_SUMMARY.md (Mantenimiento)
```

### Ruta 2: Especialista en Calibración (4 horas + práctica)
```
docs/CALIBRATION.md → QUICKSTART.md (Calibración) → TECHNICAL_SUMMARY.md (Herramientas) → Práctica con buffers
```

### Ruta 3: Desarrollador de Firmware (6 horas)
```
README.md → main/sensors.h → main/sensors.c → CHANGELOG.md → docs/SECURITY.md
```

### Ruta 4: Ingeniero DevOps (3 horas)
```
QUICKSTART.md → TECHNICAL_SUMMARY.md → docs/TROUBLESHOOTING.md → PROJECT_STATUS.md
```

---

## 🔍 Matriz de Troubleshooting

| Síntoma | Causa Probable | Documento | Sección |
|---------|----------------|-----------|---------|
| Sistema no responde | Agent no está corriendo | [docs/guides/TROUBLESHOOTING.md](docs/guides/TROUBLESHOOTING.md) | Recuperación |
| pH fuera de rango (>14, <0) | Divisor de voltaje mal configurado | [docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md) | Troubleshooting |
| Error de pH >0.3 | Necesita recalibración | [docs/guides/CALIBRATION.md](docs/guides/CALIBRATION.md) | Calibración 3 Puntos |
| Conexión WiFi fallida | Credenciales incorrectas | [QUICKSTART.md](QUICKSTART.md) | Troubleshooting |
| Agent inalcanzable | IP/Puerto no coinciden | [docs/guides/TROUBLESHOOTING.md](docs/guides/TROUBLESHOOTING.md) | Red |
| MongoDB no guarda | Credenciales/whitelist | [docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md) | MongoDB |
| Timestamps incorrectos | Configuración de timezone | [docs/technical/TECHNICAL_SUMMARY.md](docs/technical/TECHNICAL_SUMMARY.md) | Configuración |
| Errores de compilación | Build sucio | [README.md](README.md) | Troubleshooting |

---

## 📊 Estadísticas de Documentación

| Documento | Líneas | Tamaño | Audiencia | Última Actualización |
|-----------|--------|--------|-----------|---------------------|
| README.md | 400+ | 13 KB | Todos | 2026-01-22 |
| QUICKSTART.md | 350+ | 8 KB | Usuarios Nuevos | 2026-01-22 |
| CHANGELOG.md | 300+ | 7 KB | Desarrolladores | 2026-01-22 |
| PROJECT_STATUS.md | 280+ | 9 KB | Gerencia | 2026-01-22 |
| TECHNICAL_SUMMARY.md | 250+ | 7 KB | Técnicos | 2026-01-22 |
| docs/CALIBRATION.md | 350+ | 8 KB | Calibradores | 2026-01-22 |
| docs/TROUBLESHOOTING.md | 300+ | 7 KB | Todos | 2026-01-22 |
| docs/SECURITY.md | 200+ | 5 KB | Desarrolladores | 2026-01-22 |
| **Total** | **2400+** | **64 KB** | — | — |

---

## 📝 Estándares de Documentación

Esta documentación sigue:

- **IEEE 1063-2001** — Estándar para Documentación de Usuario de Software
- **ISO/IEC 26514:2008** — Ingeniería de sistemas y software - Documentación de usuario
- **Semantic Versioning 2.0.0** — Numeración de versiones
- **Keep a Changelog 1.0.0** — Formato de changelog
- **CommonMark** — Especificación de Markdown

### Convenciones Utilizadas

| Convención | Significado |
|------------|-------------|
| `código` | Comandos, código, nombres de archivo |
| **negrita** | Términos importantes, elementos de UI |
| *cursiva* | Énfasis, variables |
| > cita | Notas, advertencias |
| ⚠️ | Advertencia |
| ✅ | Éxito/Verificado |
| ❌ | Error/Falla |

---

## 🔄 Historial de Versiones

| Versión | Fecha | Cambios |
|---------|-------|---------|
| 1.1.0 | 2026-01-22 | Agregado TROUBLESHOOTING.md, SECURITY.md, estándares profesionales |
| 1.0.0 | 2026-01-21 | Índice de documentación inicial |

---

**Mantenido por:** [@Marton1123](https://github.com/Marton1123)  
**Repositorio:** [Biofloc-Firmware-ROS](https://github.com/Marton1123/Biofloc-Firmware-ROS)
