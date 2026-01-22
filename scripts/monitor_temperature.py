#!/usr/bin/env python3
"""
Monitor de temperatura en tiempo real para calibración
Muestra voltajes y temperatura actual del sensor

Actualizado para usar el nuevo formato JSON en /biofloc/sensor_data
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import sys
import statistics
import json


class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temp_monitor')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.callback,
            10
        )
        
        self.samples = []
        self.max_samples = 50
        
        print("=" * 80)
        print("MONITOR DE TEMPERATURA PARA CALIBRACIÓN")
        print("=" * 80)
        print("\nRecopilando lecturas... (Ctrl+C para ver estadísticas)")
        print("-" * 80)
        
    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            temp_data = data.get('sensors', {}).get('temperature', {})
            
            temp = temp_data.get('value', 0)
            voltage = temp_data.get('voltage', 0)
            
            # V_GPIO = V_sensor / factor (factor actual = 1.419)
            v_gpio = voltage / 1.419
            
            self.samples.append({
                'temp': temp,
                'voltage': voltage,
                'v_gpio': v_gpio,
            })
            
            if len(self.samples) > self.max_samples:
                self.samples.pop(0)
            
            # Mostrar cada 10 muestras
            if len(self.samples) % 10 == 0:
                self.print_stats()
                
        except json.JSONDecodeError:
            pass
    
    def print_stats(self):
        if not self.samples:
            return
        
        # Promedios
        avg_temp = statistics.mean([s['temp'] for s in self.samples])
        avg_voltage = statistics.mean([s['voltage'] for s in self.samples])
        avg_v_gpio = statistics.mean([s['v_gpio'] for s in self.samples])
        
        # Desviaciones
        std_temp = statistics.stdev([s['temp'] for s in self.samples]) if len(self.samples) > 1 else 0
        std_voltage = statistics.stdev([s['voltage'] for s in self.samples]) if len(self.samples) > 1 else 0
        
        now = datetime.now().strftime('%H:%M:%S')
        
        print(f"\n[{now}] Promedio de últimas {len(self.samples)} lecturas:")
        print(f"  Temperatura:    {avg_temp:6.2f}°C  (σ={std_temp:.3f})")
        print(f"  V_sensor:       {avg_voltage:6.3f}V  (σ={std_voltage:.4f})")
        print(f"  V_GPIO (est):   {avg_v_gpio:6.3f}V")
        print("-" * 80)
    
    def final_report(self):
        if not self.samples:
            print("\nNo hay datos recopilados.")
            return
        
        print("\n" + "=" * 80)
        print("REPORTE FINAL DE TEMPERATURA")
        print("=" * 80)
        
        self.print_stats()
        
        avg_temp = statistics.mean([s['temp'] for s in self.samples])
        avg_voltage = statistics.mean([s['voltage'] for s in self.samples])
        
        print(f"\n📊 RESUMEN:")
        print(f"   Temperatura promedio: {avg_temp:.2f}°C")
        print(f"   Voltaje promedio:     {avg_voltage:.3f}V")
        print(f"   Factor divisor:       1.419")
        print("=" * 80)


def main():
    rclpy.init()
    node = TemperatureMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.final_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
