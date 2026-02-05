# 🔬 Análisis de Ciclos de pH en Cultivos de Microalgas

## 📋 Índice
1. [Introducción](#introducción)
2. [Fenómeno Biológico](#fenómeno-biológico)
3. [Uso de la Herramienta](#uso-de-la-herramienta)
4. [Interpretación de Resultados](#interpretación-de-resultados)
5. [Casos de Uso](#casos-de-uso)
6. [Troubleshooting](#troubleshooting)

---

## 🌟 Introducción

### ¿Por qué analizar ciclos de pH?

En sistemas de acuicultura con microalgas, el **pH fluctúa naturalmente** siguiendo un patrón circadiano (24 horas). Estas oscilaciones son **normales y deseables** cuando se deben a actividad fotosintética.

Sin embargo, al ver estas fluctuaciones en gráficas históricas, pueden generar dudas:
- ❓ ¿Es un problema del sensor?
- ❓ ¿Es un error en el código?
- ❓ ¿Es un fenómeno biológico normal?

**Esta herramienta te ayuda a distinguir entre estos escenarios.**

---

## 🌿 Fenómeno Biológico

### Fotosíntesis y Respiración

Las microalgas realizan dos procesos metabólicos principales:

#### 1. Fotosíntesis (DÍA, con luz)
```
6CO₂ + 6H₂O + luz → C₆H₁₂O₆ + 6O₂
```
- Las algas **consumen CO₂** del agua
- El CO₂ forma ácido carbónico (H₂CO₃) en el agua
- Menos CO₂ = Menos H₂CO₃ = **pH SUBE** ↑

#### 2. Respiración (NOCHE, sin luz)
```
C₆H₁₂O₆ + 6O₂ → 6CO₂ + 6H₂O + energía
```
- Las algas **producen CO₂**
- Más CO₂ = Más H₂CO₃ = **pH BAJA** ↓

### Patrón Típico

```
pH
│
7.4 ┤              ╭───╮              ╭───╮
    │             ╱     ╲            ╱     ╲
7.2 ┤            ╱       ╲          ╱       ╲
    │           ╱         ╲        ╱         ╲
7.0 ┤──────────╯           ╰──────╯           ╰─────
    │
6.8 ┤
    └─┬────┬────┬────┬────┬────┬────┬────┬────┬───
      00   06   12   18   00   06   12   18   00
              HORA DEL DÍA
      
      🌙 Noche    ☀️ Día    🌙 Noche
```

**Características del patrón:**
- **pH máximo**: 10:00-14:00 (máxima intensidad lumínica)
- **pH mínimo**: 04:00-08:00 (final de la noche)
- **Amplitud típica**: 0.2-0.5 pH en cultivos densos
- **Período**: 24 horas (ritmo circadiano)

---

## 🛠️ Uso de la Herramienta

### Instalación de Dependencias

```bash
pip3 install pymongo python-dateutil pytz python-dotenv
```

### Configurar Variables de Entorno

Archivo `.env` en `/home/Biofloc-Firmware-ROS/scripts/`:
```bash
MONGODB_URI=mongodb+srv://usuario:password@cluster.mongodb.net/
MONGODB_DATABASE=SistemasLab
```

### Ejecutar Análisis

```bash
cd /home/Biofloc-Firmware-ROS/scripts
python3 check_ph_cycles.py
```

### Configuración del Script

Editar variables en `check_ph_cycles.py`:

```python
# Dispositivo a analizar
DEVICE_ID = "biofloc_esp32_c8e0"

# Zona horaria
TIMEZONE = 'America/Santiago'

# Horas a analizar
ANALYSIS_HOURS = 96  # 4 días

# Período específico (para evitar datos de mantenimiento)
ANALYSIS_END = datetime(2026, 2, 2, 0, 0, 0)  # Antes de mantenimiento

# Para análisis en tiempo real:
# ANALYSIS_END = None
```

---

## 📊 Interpretación de Resultados

### Salida del Script

```
📊 Análisis de Ciclos de pH - biofloc_esp32_c8e0
📅 Período: 2026-01-29 00:00 - 2026-02-02 00:00
============================================================
📦 Total de registros en DB: 307445
✅ Registros en período analizado: 83595

  Hora |  Promedio pH |    Min |    Max |  Rango
------------------------------------------------------------
00:00 |         6.99 |   6.92 |   7.04 |   0.12
...
11:00 |         7.10 |   7.00 |   7.37 |   0.37  ← PICO
...
17:00 |         6.94 |   3.91 |   7.20 |   3.29  ← MÍNIMO
...
============================================================
🌙 pH Madrugada temprana (23:00-02:00):   7.000
☀️  pH Mediodía solar (10:00-13:00):      7.073
Δ   Diferencia madrugada→mediodía:        +0.073

📊 Amplitud total del ciclo (min→máx):    0.153
    pH mínimo a las 17:00 = 6.94
    pH máximo a las 11:00 = 7.10
```

### Interpretación Automática

#### ✅ Patrón Fotosintético Detectado
```
✅ PATRÓN FOTOSINTÉTICO DETECTADO
   Amplitud del ciclo: 0.153 pH
   Esto indica actividad fotosintética de microalgas 🌿

   Interpretación:
   - Fotosíntesis: Algas consumen CO2 durante el día → pH sube
   - Respiración nocturna: Algas producen CO2 → pH baja
   - pH máximo a las 11:00 coincide con período de luz

   ⚠️  Amplitud moderada (0.153)
   - Cultivo activo pero densidad moderada
   - O sistema con alta capacidad tampón
```

**Significado:**
- Amplitud total del ciclo **> 0.12 pH** o diferencia madrugada→mediodía **> 0.08 pH**
- Cultivo con **actividad fotosintética detectada**
- Fotosíntesis **funcionando**
- **NO es problema** de sensor o código

#### ⚠️ Sin Patrón Significativo
```
⚠️  NO SE DETECTA PATRÓN DÍA/NOCHE SIGNIFICATIVO
   Posibles causas:
   - Baja densidad de microalgas
   - Iluminación insuficiente
```

**Significado:**
- Diferencia día/noche **< 0.05 pH**
- Posibles causas:
  - Cultivo joven o diludo
  - Iluminación inadecuada
  - Sistema muy tamponado (alta alcalinidad)
  - Aireación muy intensa

#### ❓ Patrón Inverso
```
❓ PATRÓN INVERSO O ATÍPICO
   El pH baja durante el día (-0.10)
   Revisar:
   - ¿Hay equipos que se activan durante el día?
```

**Significado:**
- pH **baja** en el día (contraintuitivo)
- Causas comunes:
  - Equipos automatizados (aireación, alimentación)
  - Período de mantenimiento/calibración
  - Interferencia humana

---

## 💼 Casos de Uso

### 1. Diagnóstico: ¿Sensor o Biología?

**Problema:** Gráfica muestra oscilaciones de pH como "olas"

**Pregunta:** ¿Es fallo del sensor o fenómeno normal?

**Solución:**
1. Ejecutar `check_ph_cycles.py`
2. Si detecta patrón fotosintético → **Es normal, cultivo saludable**
3. Si no detecta patrón → Revisar sensor o condiciones del cultivo

### 2. Evaluación de Salud del Cultivo

**Objetivo:** Verificar si las microalgas están activas

**Método:**
- Analizar amplitud total del ciclo (pH máx - pH mín)
- **> 0.3 pH**: Cultivo muy activo, alta biomasa
- **0.12-0.3 pH**: Cultivo activo con densidad moderada
- **< 0.12 pH**: Cultivo poco activo o baja densidad

Alternativamente:
- Diferencia madrugada temprana→mediodía solar
- **> 0.15 pH**: Patrón fotosintético fuerte
- **0.08-0.15 pH**: Patrón moderado
- **< 0.08 pH**: Patrón débil o ausente

### 3. Optimización de Iluminación

**Objetivo:** Verificar si la luz es suficiente

**Método:**
- Observar hora del pico de pH
- Debería coincidir con máxima iluminación (11:00-14:00)
- Si el pico es temprano/tardío → Ajustar horario de luces

### 4. Detección de Interferencias

**Objetivo:** Identificar períodos de mantenimiento/calibración

**Método:**
- Comparar período normal vs período sospechoso
- Outliers (pH < 3 o > 12) indican manipulación del sensor
- Ausencia de patrón indica datos no representativos

---

## 🔧 Troubleshooting

### Error: "Pocos datos para analizar"

**Causa:** No hay suficientes registros en el período

**Solución:**
- Aumentar `ANALYSIS_HOURS`
- Verificar que el `DEVICE_ID` es correcto
- Revisar conexión a MongoDB

### Error: "can't compare offset-naive and offset-aware"

**Causa:** Problemas con zonas horarias

**Solución:**
- Verificar que `TIMEZONE` está correctamente configurado
- El script maneja esto automáticamente ahora

### Outliers Detectados

**Ejemplo:**
```
⚠️  Outliers detectados: 3
    2026-01-29 17:23:15: pH = 3.91
    2026-01-30 15:45:22: pH = 1.00
```

**Significado:**
- Valores fuera del rango normal (3-12 pH)
- Típicamente ocurren durante:
  - Calibración de sensores
  - Manipulación manual
  - Desconexión temporal
- **No se incluyen en estadísticas**

### Patrón No Coincide con Expectativa

**Ejemplo:** Pico de pH a las 08:00 en lugar de 12:00

**Posibles causas:**
1. **Equipos automáticos**
   - Aireación que se activa a esa hora
   - Expulsa CO₂ → pH sube
   
2. **Alimentación**
   - Alimento puede alterar pH temporalmente
   
3. **Iluminación artificial**
   - Luces que no siguen ciclo solar natural

**Solución:** Documentar rutinas y correlacionar con datos

---

## 📈 Ejemplo Real: Caso MicroAlgas Martin

### Análisis Realizado

**Período normal (29 ene - 1 feb 2026):**
```
🌙 pH Madrugada temprana (23:00-02:00):   7.000
☀️  pH Mediodía solar (10:00-13:00):      7.073
Δ   Diferencia madrugada→mediodía:        +0.073

📊 Amplitud total del ciclo (min→máx):    0.153
    pH mínimo a las 17:00 = 6.94
    pH máximo a las 11:00 = 7.10

✅ PATRÓN FOTOSINTÉTICO DETECTADO
```

**Hallazgos:**
- pH máximo: **7.10** a las **11:00** (pico solar) ✅
- pH mínimo: **6.94** a las **17:00** (final de tarde) ✅
- Amplitud: **0.153 pH** (cultivo activo, densidad moderada) ✅
- Diferencia madrugada→mediodía: **+0.073 pH** ✅
- Ritmo circadiano **detectado y coherente** ✅

**Conclusión:**
- Las "olas" observadas en la gráfica son **normales** ✅
- Indican un cultivo de microalgas **activo** con fotosíntesis funcionando 🌿
- Amplitud moderada sugiere densidad media o sistema tamponado
- **NO es problema** de sensor ni de código ✅

---

**Período de mantenimiento (2-5 feb 2026):**
```
📈 pH Promedio Día (8am-8pm):    6.87
📉 pH Promedio Noche (8pm-8am):  6.89
Δ  Diferencia:                    -0.02

⚠️  NO SE DETECTA PATRÓN DÍA/NOCHE SIGNIFICATIVO
```

**Hallazgos:**
- Múltiples outliers detectados ⚠️
- Patrón inconsistente
- Diferencia día/noche mínima

**Conclusión:**
- Período no representativo debido a **manipulación de sensores**
- Confirma la importancia de analizar **períodos de operación normal**

---

## 🎯 Resumen de Mejores Prácticas

1. **Analizar períodos normales**: Evitar datos de mantenimiento/calibración
2. **Verificar biomasa**: Cultivos densos muestran patrones más claros
3. **Documentar rutinas**: Correlacionar picos con actividades operativas
4. **Monitorear tendencias**: Cambios en amplitud pueden indicar cambios en densidad
5. **Complementar con otros parámetros**: Oxígeno disuelto, temperatura, turbidez

---

**Autor:** [@Marton1123](https://github.com/Marton1123)  
**Fecha:** 2026-02-05  
**Versión:** 1.0  
**Herramienta:** `scripts/check_ph_cycles.py`
