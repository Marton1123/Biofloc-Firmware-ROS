#!/usr/bin/env python3
"""
Send calibration command directly without re-measuring
Useful when you already have the 3 points and just need to send them
"""

import subprocess
import json
import time
import sys

def send_calibration(device_id, points, sensor_id='ph'):
    """Send calibration command directly to ESP32"""
    
    # Build JSON command
    cmd_json = json.dumps({
        "sensor": sensor_id,
        "action": "calibrate",
        "points": points
    })
    
    print(f"📤 Enviando calibración directa al ESP32...")
    print(f"   Dispositivo: {device_id}")
    print(f"   Sensor: {sensor_id}")
    print(f"   Puntos: {len(points)}")
    for i, p in enumerate(points, 1):
        print(f"     Punto {i}: {p['voltage']:.3f}V → {p['value']} {sensor_id}")
    print()
    
    # Step 1: Start ACK listener BEFORE sending
    print("⏳ Preparando listener de ACK...")
    ack_cmd = [
        'bash', '-c',
        'source /opt/ros/jazzy/setup.bash && '
        'source ~/microros_ws/install/local_setup.bash && '
        'timeout 120 ros2 topic echo --once --full-length '
        '/biofloc/calibration_status std_msgs/msg/String 2>&1'
    ]
    
    ack_process = subprocess.Popen(
        ack_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    
    time.sleep(2)
    
    # Step 2: Publish calibration command
    pub_cmd = [
        'bash', '-c',
        f'source /opt/ros/jazzy/setup.bash && '
        f'source ~/microros_ws/install/local_setup.bash && '
        f"ros2 topic pub --once /biofloc/calibration_cmd "
        f"std_msgs/msg/String 'data: {cmd_json}'"
    ]
    
    print(f"📡 Publicando comando al ESP32...")
    print(f"   JSON: {cmd_json}")
    print()
    pub_result = subprocess.run(pub_cmd, capture_output=True, text=True, timeout=15)
    
    if pub_result.returncode != 0:
        print(f"❌ Error publicando: {pub_result.stderr}")
        ack_process.kill()
        ack_process.wait()
        return False
    
    print(f"✓ Comando enviado, esperando ACK (120s)...")
    print()
    
    # Step 3: Wait for ACK
    try:
        ack_stdout, ack_stderr = ack_process.communicate(timeout=120)
    except subprocess.TimeoutExpired:
        ack_process.kill()
        ack_process.wait()
        print("❌ TIMEOUT: No se recibió ACK del ESP32 en 120s")
        return False
    
    if ack_process.returncode != 0 or 'data:' not in (ack_stdout or ''):
        print("❌ No se recibió respuesta válida del ESP32")
        print(f"   Return code: {ack_process.returncode}")
        print(f"   STDOUT: {ack_stdout[:200] if ack_stdout else 'None'}")
        print(f"   STDERR: {ack_stderr[:200] if ack_stderr else 'None'}")
        return False
    
    # Step 4: Parse ACK
    try:
        response_data = ack_stdout.split('data:', 1)[1].strip()
        if response_data.startswith("'") or response_data.startswith('"'):
            response_data = response_data[1:]
        if response_data.endswith("'") or response_data.endswith('"'):
            response_data = response_data[:-1]
        response_data = response_data.replace('\\n', '').replace('\\r', '').replace('\\t', '').strip()
        
        # DEBUG: Print raw data
        print(f"🔍 DEBUG - Raw data recibido ({len(response_data)} bytes):")
        print(f"   HEX: {response_data.encode().hex()}")
        print(f"   ASCII: {repr(response_data)}")
        print()
        
        response_json = json.loads(response_data)
    except (json.JSONDecodeError, IndexError, AttributeError) as e:
        print(f"⚠ ACK recibido pero JSON inválido: {e}")
        print(f"   Raw stdout: {repr(ack_stdout[:500])}")
        return False
    
    # Step 5: Check status
    if response_json.get('status') == 'success':
        print("✅ ¡ÉXITO! ESP32 confirmó calibración")
        if 'slope' in response_json:
            print(f"   Slope: {response_json['slope']:.6f}")
        if 'offset' in response_json:
            print(f"   Offset: {response_json['offset']:.6f}")
        if 'r_squared' in response_json:
            print(f"   R²: {response_json['r_squared']:.4f}")
        print()
        print("✅ Calibración guardada en NVS del ESP32")
        return True
    else:
        error_msg = response_json.get('message', 'Error desconocido')
        print(f"❌ ESP32 reportó error: {error_msg}")
        return False

if __name__ == '__main__':
    # Calibration points from user's last successful measurement
    points = [
        {"voltage": 1.742, "value": 4.52},
        {"voltage": 2.508, "value": 7.34},
        {"voltage": 3.439, "value": 9.86}
    ]
    
    device_id = "biofloc_esp32_c8e0"
    
    print("=" * 70)
    print("  Envío Directo de Calibración pH")
    print("=" * 70)
    print()
    
    success = send_calibration(device_id, points, sensor_id='ph')
    
    sys.exit(0 if success else 1)
