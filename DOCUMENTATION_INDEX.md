# 📚 Índice de Documentación — Biofloc Firmware ROS v2.3.0

> **Cumplimiento de Estándares:** ISO/IEC 26514 (Documentación de Software) | IEEE 1063 (Documentación de Usuario)

| Metadatos | Valor |
|-----------|-------|
| **Versión** | 2.3.0 |
| **Última Actualización** | 2026-01-22 |
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
| **🚀 Ejecutar el proyecto** | [GUIA_PASO_A_PASO.md](GUIA_PASO_A_PASO.md) | Completo |
| **Instalación inicial** | [QUICKSTART.md](QUICKSTART.md) | Instalación |
| **Sistema no responde** | [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Procedimiento de Recuperación |
| **Calibración de pH** | [docs/CALIBRATION.md](docs/CALIBRATION.md) | Calibración 3 Puntos |
| **Parámetros actuales** | [calibration_3point_result.txt](calibration_3point_result.txt) | Completo |
| **Referencia de API** | [main/sensors.h](main/sensors.h) | Documentación de Funciones |
| **Estado del proyecto** | [PROJECT_STATUS.md](PROJECT_STATUS.md) | Métricas |
| **Guías de seguridad** | [docs/SECURITY.md](docs/SECURITY.md) | Completo |

---

## 👥 Documentación por Rol

### 🆕 Usuarios Nuevos — Primeros Pasos
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| **1** | [**GUIA_PASO_A_PASO.md**](GUIA_PASO_A_PASO.md) | **Comandos para ejecutar paso a paso** | **15 min** |
| 2 | [QUICKSTART.md](QUICKSTART.md) | Instalación y primera ejecución | 30 min |
| 3 | [README.md](README.md) | Visión general del sistema | 15 min |
| 4 | [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Problemas comunes | Referencia |

### 🔬 Técnicos de Calibración
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [GUIA_PASO_A_PASO.md § Calibración](GUIA_PASO_A_PASO.md#4-calibración-del-sensor-de-ph) | Comandos de calibración | 10 min |
| 2 | [docs/CALIBRATION.md](docs/CALIBRATION.md) | Teoría y procedimiento de calibración | 1 hr |
| 3 | [TECHNICAL_SUMMARY.md](TECHNICAL_SUMMARY.md) | Parámetros actuales | 10 min |
| 4 | [calibration_3point_result.txt](calibration_3point_result.txt) | Datos de calibración activa | 5 min |

### 💻 Desarrolladores de Firmware
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [README.md](README.md) | Visión general de arquitectura | 20 min |
| 2 | [main/sensors.h](main/sensors.h) | Documentación de API | 30 min |
| 3 | [CHANGELOG.md](CHANGELOG.md) | Historial de versiones | 15 min |
| 4 | [docs/SECURITY.md](docs/SECURITY.md) | Guías de seguridad | 15 min |

### 🔧 DevOps / Administradores de Sistemas
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [TECHNICAL_SUMMARY.md](TECHNICAL_SUMMARY.md) | Referencia de configuración | 20 min |
| 2 | [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Procedimientos de recuperación | Referencia |
| 3 | [PROJECT_STATUS.md](PROJECT_STATUS.md) | Métricas de salud del sistema | 10 min |

### 📊 Gerentes de Proyecto
| Prioridad | Documento | Propósito | Tiempo |
|-----------|-----------|-----------|--------|
| 1 | [PROJECT_STATUS.md](PROJECT_STATUS.md) | Estado y roadmap | 15 min |
| 2 | [CHANGELOG.md](CHANGELOG.md) | Historial de releases | 10 min |

---

## 📑 Índice por Tema

### Ejecución del Sistema
- [**GUIA_PASO_A_PASO.md**](GUIA_PASO_A_PASO.md) — Comandos paso a paso
- [GUIA_PASO_A_PASO.md § Uso Diario](GUIA_PASO_A_PASO.md#3-uso-diario) — Comandos frecuentes
- [GUIA_PASO_A_PASO.md § Solución de Problemas](GUIA_PASO_A_PASO.md#5-solución-de-problemas-rápida)

### Hardware y Sensores
- [README.md § Requisitos de Hardware](README.md#-requisitos)
- [README.md § Especificaciones Técnicas](README.md#-especificaciones-técnicas)
- [docs/CALIBRATION.md § Sensor CWT-BL](docs/CALIBRATION.md)
- [TECHNICAL_SUMMARY.md § Divisor de Voltaje](TECHNICAL_SUMMARY.md)

### Calibración de pH
- [docs/CALIBRATION.md](docs/CALIBRATION.md) — Guía completa
- [QUICKSTART.md § Calibración](QUICKSTART.md) — Procedimiento rápido
- [calibration_3point_result.txt](calibration_3point_result.txt) — Valores actuales
- [main/sensors.h § API de Calibración](main/sensors.h) — Interfaz de código

### Instalación y Configuración
- [QUICKSTART.md § Instalación](QUICKSTART.md) — Paso a paso
- [README.md § Quick Start](README.md#-quick-start) — Resumen
- [TECHNICAL_SUMMARY.md § Configuración](TECHNICAL_SUMMARY.md) — Todos los parámetros
- [main/Kconfig.projbuild](main/Kconfig.projbuild) — Opciones de menuconfig

### Seguridad
- [docs/SECURITY.md](docs/SECURITY.md) — Guías completas de seguridad
- Gestión de credenciales
- Seguridad de red
- Prácticas de código seguro

### micro-ROS y ROS 2
- [README.md § micro-ROS Agent](README.md)
- [docs/TROUBLESHOOTING.md § Conexión del Agent](docs/TROUBLESHOOTING.md)
- [TECHNICAL_SUMMARY.md § Configuración de Red](TECHNICAL_SUMMARY.md)

### MongoDB y Telemetría
- [README.md § MongoDB Bridge](README.md#-mongodb-bridge-almacenamiento-de-datos)
- [TECHNICAL_SUMMARY.md § Formato de Datos](TECHNICAL_SUMMARY.md)
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
| Sistema no responde | Agent no está corriendo | [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Recuperación |
| pH fuera de rango (>14, <0) | Divisor de voltaje mal configurado | [TECHNICAL_SUMMARY.md](TECHNICAL_SUMMARY.md) | Troubleshooting |
| Error de pH >0.3 | Necesita recalibración | [CALIBRATION.md](docs/CALIBRATION.md) | Calibración 3 Puntos |
| Conexión WiFi fallida | Credenciales incorrectas | [QUICKSTART.md](QUICKSTART.md) | Troubleshooting |
| Agent inalcanzable | IP/Puerto no coinciden | [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Red |
| MongoDB no guarda | Credenciales/whitelist | [TECHNICAL_SUMMARY.md](TECHNICAL_SUMMARY.md) | MongoDB |
| Timestamps incorrectos | Configuración de timezone | [TECHNICAL_SUMMARY.md](TECHNICAL_SUMMARY.md) | Configuración |
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

**Mantenido por:** Biofloc Engineering Team  
**Repositorio:** [Biofloc-Firmware-ROS](https://github.com/biofloc/Biofloc-Firmware-ROS)
