#!/usr/bin/env python3
"""
Test the full calibration flow step by step
"""

import subprocess
import json
import time

def run_cmd(cmd, timeout=10):
    """Run command and return output"""
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, shell=True)
    return result.stdout, result.stderr, result.returncode

print("="*70)
print("  Test de Flujo de Calibración")
print("="*70)
print()

# Step 1: Check if topics exist
print("1️⃣ Verificando topics ROS2...")
print()

cmd = "source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 topic list"
stdout, stderr, rc = run_cmd(cmd)

if "/biofloc/calibration_cmd" in stdout:
    print("   ✅ Topic /biofloc/calibration_cmd existe")
else:
    print("   ❌ Topic /biofloc/calibration_cmd NO existe")

if "/biofloc/calibration_status" in stdout:
    print("   ✅ Topic /biofloc/calibration_status existe")
else:
    print("   ❌ Topic /biofloc/calibration_status NO existe")

if "/biofloc/sensor_data" in stdout:
    print("   ✅ Topic /biofloc/sensor_data existe")
else:
    print("   ❌ Topic /biofloc/sensor_data NO existe")

print()

# Step 2: Check if ESP32 is publishing sensor data
print("2️⃣ Verificando si ESP32 está publicando datos...")
print()

cmd = "source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && timeout 5 ros2 topic echo --once /biofloc/sensor_data"
stdout, stderr, rc = run_cmd(cmd, timeout=10)

if rc == 0 and "data:" in stdout:
    print("   ✅ ESP32 está publicando en /biofloc/sensor_data")
    # Extract device_id
    try:
        data_str = stdout.split('data:', 1)[1].strip()
        if data_str.startswith("'") or data_str.startswith('"'):
            data_str = data_str[1:-1]
        data_str = data_str.replace('\\n', '').replace('\\r', '').replace('\\t', '').strip()
        data = json.loads(data_str)
        device_id = data.get('device_id', 'unknown')
        print(f"   Device ID: {device_id}")
    except:
        print("   ⚠️  No se pudo parsear device_id")
else:
    print("   ❌ ESP32 NO está publicando datos")
    print(f"   Return code: {rc}")

print()

# Step 3: Send simple test command
print("3️⃣ Enviando comando de prueba simple (action=get)...")
print()

test_cmd = json.dumps({"sensor": "ph", "action": "get"})
print(f"   JSON: {test_cmd}")

pub_cmd = f"source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && ros2 topic pub --once /biofloc/calibration_cmd std_msgs/msg/String \"data: '{test_cmd}'\""

stdout, stderr, rc = run_cmd(pub_cmd, timeout=10)

if rc == 0:
    print("   ✅ Comando publicado correctamente")
else:
    print(f"   ❌ Error publicando comando (rc={rc})")
    if stderr:
        print(f"   STDERR: {stderr[:200]}")

print()
print("4️⃣ Esperando respuesta en /biofloc/calibration_status (10s)...")
print()

ack_cmd = "source /opt/ros/jazzy/setup.bash && source ~/microros_ws/install/local_setup.bash && timeout 10 ros2 topic echo --once /biofloc/calibration_status"

stdout, stderr, rc = run_cmd(ack_cmd, timeout=15)

if rc == 0 and "data:" in stdout:
    print("   ✅ ESP32 respondió!")
    print(f"   Respuesta: {stdout[:300]}")
elif rc == 124:
    print("   ❌ TIMEOUT: ESP32 no respondió en 10s")
    print("   Posibles causas:")
    print("     - ESP32 no está suscrito a /biofloc/calibration_cmd")
    print("     - Firmware no tiene handler de calibración activo")
    print("     - JSON no se está parseando correctamente")
else:
    print(f"   ❌ Error (rc={rc})")

print()
print("="*70)
print("  Diagnóstico completo")
print("="*70)
