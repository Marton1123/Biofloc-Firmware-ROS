#!/usr/bin/env python3
"""
Send calibration command directly without re-measuring
Useful when you already have the 3 points and just need to send them
"""

import json
import time
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.publisher = self.create_publisher(String, '/biofloc/calibration_cmd', 10)
        self.subscription = self.create_subscription(
            String,
            '/biofloc/calibration_status',
            self.ack_callback,
            10
        )
        self.ack_received = None
        self.ack_event = threading.Event()
        
    def ack_callback(self, msg):
        """Called when ACK is received from ESP32"""
        self.ack_received = msg.data
        self.ack_event.set()
    
    def send_command(self, cmd_json):
        """Publish calibration command"""
        msg = String()
        msg.data = cmd_json
        self.publisher.publish(msg)
        self.get_logger().info(f'✓ Command published: {len(cmd_json)} bytes')
    
    def wait_for_ack(self, timeout=120.0):
        """Wait for ACK with timeout"""
        return self.ack_event.wait(timeout)

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
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        client = CalibrationClient()
        
        print("⏳ Esperando 2s para establecer conexiones...")
        time.sleep(2)
        
        print(f"📡 Publicando comando al ESP32...")
        print(f"   JSON: {cmd_json}")
        print()
        
        client.send_command(cmd_json)
        
        print(f"✓ Comando enviado, esperando ACK (120s)...")
        print()
        
        # Spin in background while waiting for ACK
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(client)
        
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        # Wait for ACK with timeout
        if not client.wait_for_ack(timeout=120.0):
            print("❌ TIMEOUT: No se recibió ACK del ESP32 en 120s")
            return False
        
        executor.shutdown()
        
        # Parse ACK
        try:
            # Debug raw data
            response_data = client.ack_received
            print(f"🔍 DEBUG - Raw data recibido ({len(response_data)} bytes):")
            print(f"   HEX: {response_data.encode().hex()}")
            print(f"   ASCII: {repr(response_data)}")
            print()
            
            response_json = json.loads(response_data)
        except json.JSONDecodeError as e:
            print(f"⚠ ACK recibido pero JSON inválido: {e}")
            print(f"   Raw data: {repr(client.ack_received[:500])}")
            return False
        
        # Check status
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
            
    finally:
        client.destroy_node()
        rclpy.shutdown()

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
