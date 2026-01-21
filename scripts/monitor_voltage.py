#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""
Monitor de Voltaje en Tiempo Real
Muestra el voltaje del sensor para comparar con multímetro
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys

class VoltageMonitor(Node):
    def __init__(self):
        super().__init__('voltage_monitor')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.count = 0
        print("\n" + "═" * 70)
        print("  MONITOR DE VOLTAJE EN TIEMPO REAL")
        print("═" * 70)
        print("\nInstrucciones:")
        print("  1. Conecta el multímetro entre el pin GPIO36 y GND")
        print("  2. Observa el voltaje en el multímetro")
        print("  3. Compara con el voltaje que aparece aquí")
        print("  4. Presiona Ctrl+C para salir")
        print("\n" + "─" * 70)
        print(f"{'Tiempo':>8} | {'V_GPIO (Software)':>18} | {'V_Sensor':>10} | {'pH':>7}")
        print("─" * 70)
        
    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            ph_sensor = data.get('sensors', {}).get('ph', {})
            
            # Extraer datos
            v_sensor = ph_sensor.get('voltage', 0)      # Voltaje del sensor (después de divider)
            v_gpio = v_sensor / 1.596                    # Voltaje en el GPIO (antes de divider)
            ph_value = ph_sensor.get('value', 0)
            
            self.count += 1
            timestamp = time.strftime("%H:%M:%S")
            
            # Mostrar en formato tabular
            print(f"{timestamp:>8} | {v_gpio:>10.4f}V (GPIO) | {v_sensor:>8.4f}V | {ph_value:>6.2f}")
            sys.stdout.flush()
            
        except (json.JSONDecodeError, KeyError) as e:
            pass


def main():
    rclpy.init()
    monitor = VoltageMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n" + "─" * 70)
        print("Monitor detenido")
        print("═" * 70 + "\n")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
