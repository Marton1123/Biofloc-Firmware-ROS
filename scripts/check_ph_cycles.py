#!/usr/bin/env python3
"""
Análisis de Ciclos de pH - Detección de Patrones Fotosintéticos
================================================================

Este script analiza los ciclos de pH del cultivo de microalgas para:
- Detectar patrones de fotosíntesis (pH sube durante el día por consumo de CO2)
- Identificar ritmos circadianos del cultivo
- Validar salud del cultivo (oscilaciones normales vs anómalas)
- Diagnosticar problemas de sensores vs fenómenos biológicos

Fenómeno Biológico Esperado:
----------------------------
DÍA (con luz): 
  - Fotosíntesis activa
  - Microalgas consumen CO2
  - CO2 ↓ → pH ↑ (alcalinización)
  
NOCHE (sin luz):
  - Solo respiración
  - Microalgas producen CO2
  - CO2 ↑ → pH ↓ (acidificación)

Patrón Normal en Microalgas:
- pH máximo: 10:00-14:00 (máxima intensidad lumínica)
- pH mínimo: 04:00-08:00 (final de la noche)
- Amplitud típica: 0.2-0.5 pH en cultivos densos

Uso:
----
    python3 check_ph_cycles.py

Salida:
-------
- Tabla de pH promedio por hora del día
- Comparación día vs noche
- Detección automática de patrón fotosintético
- Identificación de outliers

Autor: @Marton1123 (https://github.com/Marton1123)
Fecha: 2026-02-05
Versión: 1.0
"""
import pymongo
from datetime import datetime, timedelta
import os
from dotenv import load_dotenv
from dateutil import parser
import pytz

# ============================================================================
# CONFIGURACIÓN
# ============================================================================

# Dispositivo a analizar
DEVICE_ID = "biofloc_esp32_c8e0"

# Zona horaria
TIMEZONE = 'America/Santiago'

# Horas a analizar (96 horas = 4 días)
ANALYSIS_HOURS = 96

# Período de análisis (antes del mantenimiento para datos limpios)
ANALYSIS_END = datetime(2026, 2, 2, 0, 0, 0)  # Antes del mantenimiento del 02-02
# Para análisis en tiempo real, comentar línea anterior y descomentar:
# ANALYSIS_END = None  # Usar datetime.now()

# ============================================================================
# CONEXIÓN A MONGODB ATLAS
# ============================================================================

# Cargar variables de entorno
load_dotenv()

# Conectar a MongoDB Atlas
client = pymongo.MongoClient(os.getenv("MONGODB_URI"))
db = client[os.getenv("MONGODB_DATABASE")]

# ============================================================================
# ANÁLISIS DE DATOS
# ============================================================================

# Configurar timezone y período de análisis
local_tz = pytz.timezone(TIMEZONE)

if ANALYSIS_END is None:
    end_time = datetime.now(local_tz)
else:
    end_time = ANALYSIS_END.replace(tzinfo=local_tz) if ANALYSIS_END.tzinfo is None else ANALYSIS_END

start_time = end_time - timedelta(hours=ANALYSIS_HOURS)

print(f"{'='*60}")
print(f"📊 Análisis de Ciclos de pH - {DEVICE_ID}")
print(f"📅 Período: {start_time.strftime('%Y-%m-%d %H:%M')} - {end_time.strftime('%Y-%m-%d %H:%M')}")
print(f"{'='*60}")

# Query para obtener datos del sensor
query = {
    "device_id": DEVICE_ID,
    "sensors.ph.valid": True  # Solo datos válidos
}

# Obtener todos los datos (ordenados por timestamp)
all_data = list(db.telemetria.find(query).sort("timestamp", 1))
print(f"📦 Total de registros en DB: {len(all_data)}")

if len(all_data) < 10:
    print("⚠️  Pocos datos para analizar")
    exit(0)

# ============================================================================
# PROCESAMIENTO Y AGRUPACIÓN POR HORA
# ============================================================================

hourly_ph = {}
data = []
outliers = []

for reading in all_data:
    # Parsear timestamp
    if isinstance(reading['timestamp'], str):
        ts = parser.parse(reading['timestamp'])
    else:
        ts = reading['timestamp']
    
    # Filtrar por rango de tiempo
    if ts < start_time or ts > end_time:
        continue
    
    data.append(reading)
    hour = ts.hour
    
    # Obtener valor de pH desde estructura anidada
    ph_value = reading['sensors']['ph']['value']
    
    # Detectar outliers (valores fuera de rango normal 3-12 pH)
    if ph_value < 3.0 or ph_value > 12.0:
        outliers.append((ts, ph_value))
        continue  # No incluir en estadísticas
    
    if hour not in hourly_ph:
        hourly_ph[hour] = []
    hourly_ph[hour].append(ph_value)

print(f"✅ Registros en período analizado: {len(data)}")
if outliers:
    print(f"\n⚠️  Outliers detectados: {len(outliers)}")
    for ts, ph_val in outliers[:5]:  # Mostrar primeros 5
        print(f"    {ts}: pH = {ph_val:.2f}")

# ============================================================================
# MOSTRAR ESTADÍSTICAS POR HORA
# ============================================================================

print(f"\n{'Hora':>6} | {'Promedio pH':>12} | {'Min':>6} | {'Max':>6} | {'Rango':>6}")
print(f"{'-'*60}")

for hour in sorted(hourly_ph.keys()):
    values = hourly_ph[hour]
    avg = sum(values) / len(values)
    min_ph = min(values)
    max_ph = max(values)
    range_ph = max_ph - min_ph
    print(f"{hour:02d}:00 | {avg:12.2f} | {min_ph:6.2f} | {max_ph:6.2f} | {range_ph:6.2f}")

# ============================================================================
# DETECCIÓN DE PATRÓN FOTOSINTÉTICO
# ============================================================================

# Comparar los períodos críticos del ciclo circadiano:
# - Madrugada temprana (23:00-02:00): Inicio del día, pH bajo tras noche
# - Mediodía solar (10:00-13:00): Pico de fotosíntesis

early_morning_hours = [23, 0, 1, 2]           # Madrugada temprana (pH mínimo esperado)
solar_noon_hours = [10, 11, 12, 13]           # Mediodía solar (pH máximo esperado)

early_morning_ph = [sum(hourly_ph[h])/len(hourly_ph[h]) for h in early_morning_hours if h in hourly_ph]
solar_noon_ph = [sum(hourly_ph[h])/len(hourly_ph[h]) for h in solar_noon_hours if h in hourly_ph]

if early_morning_ph and solar_noon_ph:
    avg_morning = sum(early_morning_ph) / len(early_morning_ph)
    avg_noon = sum(solar_noon_ph) / len(solar_noon_ph)
    diff = avg_noon - avg_morning
    
    # Encontrar pH mínimo y máximo del día para contexto
    all_hourly_avgs = {h: sum(hourly_ph[h])/len(hourly_ph[h]) for h in hourly_ph}
    min_hour = min(all_hourly_avgs, key=all_hourly_avgs.get)
    max_hour = max(all_hourly_avgs, key=all_hourly_avgs.get)
    amplitude = all_hourly_avgs[max_hour] - all_hourly_avgs[min_hour]
    
    print(f"\n{'='*60}")
    print(f"🌙 pH Madrugada temprana (23:00-02:00):   {avg_morning:.3f}")
    print(f"☀️  pH Mediodía solar (10:00-13:00):      {avg_noon:.3f}")
    print(f"Δ   Diferencia madrugada→mediodía:        {diff:+.3f}")
    print(f"\n📊 Amplitud total del ciclo (min→máx):    {amplitude:.3f}")
    print(f"    pH mínimo a las {min_hour:02d}:00 = {all_hourly_avgs[min_hour]:.2f}")
    print(f"    pH máximo a las {max_hour:02d}:00 = {all_hourly_avgs[max_hour]:.2f}")
    
    # Interpretación basada en amplitud del ciclo circadiano
    if diff > 0.08 or amplitude > 0.12:
        print(f"\n✅ PATRÓN FOTOSINTÉTICO DETECTADO")
        print(f"   Amplitud del ciclo: {amplitude:.3f} pH")
        print(f"   Esto indica actividad fotosintética de microalgas 🌿")
        print(f"\n   Interpretación:")
        print(f"   - Fotosíntesis: Algas consumen CO2 durante el día → pH sube")
        print(f"   - Respiración nocturna: Algas producen CO2 → pH baja")
        print(f"   - pH máximo a las {max_hour:02d}:00 coincide con período de luz")
        if amplitude < 0.2:
            print(f"\n   ⚠️  Amplitud moderada ({amplitude:.3f})")
            print(f"   - Cultivo activo pero densidad moderada")
            print(f"   - O sistema con alta capacidad tampón")
    elif abs(diff) < 0.03 and amplitude < 0.08:
        print(f"\n⚠️  NO SE DETECTA PATRÓN CIRCADIANO SIGNIFICATIVO")
        print(f"   Amplitud muy baja: {amplitude:.3f}")
        print(f"   Posibles causas:")
        print(f"   - Baja densidad de microalgas")
        print(f"   - Iluminación insuficiente")
        print(f"   - Sistema muy tamponado (alta alcalinidad)")
        print(f"   - Aireación muy intensa (expulsa CO2 constantemente)")
    else:
        print(f"\n❓ PATRÓN ATÍPICO")
        print(f"   Amplitud: {amplitude:.3f}, pH máx a las {max_hour:02d}:00")
        print(f"   Revisar:")
        print(f"   - ¿Horario de iluminación artificial?")
        print(f"   - ¿Equipos automatizados (aireación, alimentación)?")
        print(f"   - ¿Período de mantenimiento/calibración?")

print(f"\n{'='*60}\n")
