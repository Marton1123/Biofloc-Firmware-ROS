#!/usr/bin/env python3
"""
Quick ESP32 health check
"""

import subprocess
import json
import time

def run_cmd(cmd, timeout=10):
    """Run bash command"""
    result = subprocess.run(
        ['bash', '-c', cmd], 
        capture_output=True, 
        text=True, 
        timeout=timeout
    )
    return result.stdout, result.stderr, result.returncode

print("=" * 70)
print("  ESP32 Health Check")
print("=" * 70)
print()

# Try to read sensor data 3 times
for attempt in range(3):
    print(f"Intento {attempt + 1}/3: Leyendo /biofloc/sensor_data...")
    
    cmd = "source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && timeout 5 ros2 topic echo --once /biofloc/sensor_data"
    stdout, stderr, rc = run_cmd(cmd, timeout=10)
    
    if rc == 0 and "data:" in stdout:
        print("   ✅ ESP32 ESTÁ VIVO y publicando")
        
        try:
            data_str = stdout.split('data:', 1)[1].strip()
            if data_str.startswith("'") or data_str.startswith('"'):
                data_str = data_str[1:-1]
            data_str = data_str.replace('\\n', '').replace('\\r', '').replace('\\t', '').strip()
            data = json.loads(data_str)
            
            device_id = data.get('device_id', 'unknown')
            system = data.get('system', {})
            sensors = data.get('sensors', {})
            
            print(f"   Device ID: {device_id}")
            print(f"   Uptime: {system.get('uptime_sec', 0)}s")
            print(f"   Free Heap: {system.get('free_heap', 0)} bytes")
            print(f"   Reset Reason: {system.get('reset_reason', 'UNKNOWN')}")
            print(f"   pH: {sensors.get('ph', {}).get('value', 'N/A')}")
            print(f"   Temp: {sensors.get('temperature', {}).get('value', 'N/A')}°C")
        except Exception as e:
            print(f"   ⚠️  Error parseando datos: {e}")
        
        print()
        print("✅ ESP32 está operativo - puedes intentar calibración")
        exit(0)
    
    elif rc == 124:
        print(f"   ❌ TIMEOUT (5s) - No hay mensajes")
    else:
        print(f"   ❌ Error (rc={rc})")
    
    if attempt < 2:
        print("   Reintentando en 3s...")
        time.sleep(3)
        print()

print()
print("=" * 70)
print("❌ ESP32 NO RESPONDE")
print("=" * 70)
print()
print("Posibles causas:")
print("  1. ESP32 está en PANIC loop (reiniciando constantemente)")
print("  2. WiFi desconectado")
print("  3. micro-ROS Agent no está corriendo")
print("  4. ESP32 apagado")
print()
print("Acciones recomendadas:")
print("  1. Conecta ESP32 por USB:")
print("     idf.py -p /dev/ttyUSB0 monitor")
print("  2. Verifica si hay PANICs o errores")
print("  3. Si hay PANIC loop, reflashea firmware:")
print("     idf.py -p /dev/ttyUSB0 flash")
print()
