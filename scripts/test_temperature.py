#!/usr/bin/env python3
"""
Script de prueba rápida para el sensor de temperatura
Lee datos del tópico ROS 2 y muestra solo la información de temperatura
"""

import rclpy
from rclpy.node import Node
from datetime import datetime
import sys

# Importar el mensaje personalizado
try:
    from sensor_interfaces.msg import SensorData
except ImportError:
    print("ERROR: No se pudo importar sensor_interfaces.msg.SensorData")
    print("Asegúrate de que el workspace de ROS 2 esté compilado y source'd")
    sys.exit(1)


class TemperatureTestNode(Node):
    def __init__(self):
        super().__init__('temperature_test')
        
        self.subscription = self.create_subscription(
            SensorData,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.count = 0
        self.get_logger().info('=' * 80)
        self.get_logger().info('TEST DE SENSOR DE TEMPERATURA - GPIO34')
        self.get_logger().info('=' * 80)
        self.get_logger().info('Esperando datos del ESP32...')
        self.get_logger().info('Presiona Ctrl+C para salir')
        self.get_logger().info('-' * 80)
        
    def sensor_callback(self, msg):
        self.count += 1
        
        # Extraer datos de temperatura
        temp_value = msg.temperature.value
        temp_voltage = msg.temperature.voltage
        temp_valid = msg.temperature.valid
        
        # Extraer timestamp del ESP32
        esp_timestamp = msg.timestamp
        
        # Calcular voltaje en el pin GPIO (antes del divisor)
        divider_factor = 3.0  # CONFIG_BIOFLOC_TEMP_VOLTAGE_DIVIDER_FACTOR / 1000
        v_gpio = temp_voltage / divider_factor
        
        # Formatear output
        now = datetime.now().strftime('%H:%M:%S')
        
        status = "✓" if temp_valid else "✗"
        
        print(f"\n[{self.count:4d}] {now} | ESP32: {esp_timestamp}")
        print(f"  Temperatura: {temp_value:6.2f}°C {status}")
        print(f"  V_sensor:    {temp_voltage:6.3f}V")
        print(f"  V_GPIO:      {v_gpio:6.3f}V (medido en pin, antes del divisor)")
        print(f"  Válido:      {'SÍ' if temp_valid else 'NO'}")
        
        # Información adicional de diagnóstico
        if not temp_valid:
            print(f"  ⚠️  FUERA DE RANGO VÁLIDO")
        
        # Mostrar cálculo teórico según datasheet
        # Según CWT-BL: Temp = V_sensor * 20.0 - 20.0
        temp_teorica = temp_voltage * 20.0 - 20.0
        print(f"  Temp teórica (datasheet): {temp_teorica:6.2f}°C")
        
        if abs(temp_value - temp_teorica) > 0.1:
            print(f"  ℹ️  Diferencia con fórmula datasheet: {abs(temp_value - temp_teorica):.2f}°C")


def main(args=None):
    rclpy.init(args=args)
    
    node = TemperatureTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print(f"Total de lecturas: {node.count}")
        print("=" * 80)
        node.get_logger().info('Test finalizado')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
