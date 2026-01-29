#!/usr/bin/env python3
"""
Temperature Sensor Calibration Script - 3-point offset calibration

Compares ESP32 sensor readings with a reference thermometer (TP101)
to calculate temperature offset correction.

Temperature equation is linear: T = V × 20 - 20
Only offset correction needed (no slope adjustment like pH).

Usage:
    1. Prepare 3 water samples: cold (5-10°C), ambient (20-25°C), warm (35-40°C)
    2. Have reference thermometer (TP101) ready
    3. Run: python3 scripts/calibrate_temperature.py
    4. Follow instructions for each temperature point
    5. Script calculates average offset and verifies linearity

Requirements:
    - ROS 2 environment sourced
    - micro-ROS Agent running
    - ESP32 publishing sensor data

Author: @Marton1123
Version: 3.1.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import time
import statistics
from datetime import datetime


class TemperatureCalibrator(Node):
    """Temperature calibration node."""
    
    def __init__(self):
        super().__init__('temp_calibrator')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.current_temp = None
        self.last_update = None
        
        self.get_logger().info("Temperature Calibrator iniciado")
        self.get_logger().info("Esperando datos del sensor...")
    
    def sensor_callback(self, msg):
        """Process sensor data."""
        try:
            data = json.loads(msg.data)
            temp_data = data.get('sensors', {}).get('temperature', {})
            self.current_temp = temp_data.get('value', None)
            self.last_update = time.time()
        except Exception as e:
            self.get_logger().error(f"Error parsing data: {e}")
    
    def get_stable_temperature(self, timeout=60):
        """Wait for stable temperature reading (after thermal stabilization)."""
        print("\n  Verificando estabilidad de lectura...")
        
        start_time = time.time()
        readings = []
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.current_temp is not None:
                readings.append(self.current_temp)
                
                # Keep last 15 readings
                if len(readings) > 15:
                    readings.pop(0)
                
                # Check stability (last 10 readings within 0.05°C)
                if len(readings) >= 10:
                    recent = readings[-10:]
                    std_dev = statistics.stdev(recent)
                    
                    if std_dev < 0.05:
                        avg_temp = statistics.mean(recent)
                        print(f"  ✓ Lectura estable: {avg_temp:.2f}°C (σ={std_dev:.3f}°C)")
                        return avg_temp
                
                # Show progress
                if len(readings) % 10 == 0:
                    print(f"  Lecturas: {len(readings)} (última: {self.current_temp:.2f}°C)")
        
        # Timeout
        if readings:
            avg_temp = statistics.mean(readings[-10:])
            print(f"  Timeout - usando última lectura: {avg_temp:.2f}°C")
            return avg_temp
        
        return None


def print_header():
    """Print calibration header."""
    print("\n" + "=" * 70)
    print("CALIBRACIÓN DE TEMPERATURA - 3 PUNTOS")
    print("=" * 70)
    print("\nTermómetro de referencia: TP101 (digital)")
    print("Método: Offset correction (ecuación lineal)")
    print("\nTemperaturas objetivo:")
    print("  1. Agua fría:     5-10°C  (agua con hielo)")
    print("  2. Agua ambiente: 20-25°C (agua a temperatura del laboratorio)")
    print("  3. Agua caliente: 45-55°C (agua muy caliente de grifo/termo)")
    print("\n" + "=" * 70)


def get_reference_temp(point_name):
    """Get reference temperature from user."""
    while True:
        try:
            temp_str = input(f"\n{point_name} - Lectura del TP101 [°C]: ")
            temp = float(temp_str)
            
            if -10 <= temp <= 65:
                return temp
            else:
                print("  ERROR: Temperatura fuera de rango (-10 a 65°C)")
        except ValueError:
            print("  ERROR: Ingrese un número válido")
        except KeyboardInterrupt:
            print("\n\nCalibración cancelada por usuario")
            sys.exit(0)


def calibrate_point(node, point_num, point_name, temp_range):
    """Calibrate one temperature point."""
    print("\n" + "-" * 70)
    print(f"PUNTO {point_num}/3: {point_name.upper()}")
    print("-" * 70)
    print(f"Preparar agua {point_name} ({temp_range})")
    print("Instrucciones:")
    print("  1. Coloca ambos sensores (ESP32 + TP101) en el agua")
    print("  2. Presiona ENTER para iniciar el temporizador")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\n\nCalibración cancelada por usuario")
        sys.exit(0)
    
    # Forced waiting time: 3 minutes for thermal stabilization
    print("\n  Esperando 3 minutos para estabilización térmica...")
    for remaining in range(180, 0, -30):
        mins = remaining // 60
        secs = remaining % 60
        print(f"    Tiempo restante: {mins}:{secs:02d} - Agita el agua suavemente")
        time.sleep(30)
    
    print("  ✓ Tiempo de estabilización completado")
    
    # Get ESP32 temperature (now should be stable)
    esp32_temp = node.get_stable_temperature(timeout=60)
    
    if esp32_temp is None:
        print("\n  ERROR: No se pudo obtener lectura del ESP32")
        print("  Verifica que el Agent y el ESP32 estén funcionando")
        return None
    
    # Get reference temperature
    reference_temp = get_reference_temp(f"  Punto {point_num}")
    
    # Calculate offset
    offset = reference_temp - esp32_temp
    
    print(f"\n  Resultados:")
    print(f"    ESP32:      {esp32_temp:.2f}°C")
    print(f"    TP101:      {reference_temp:.2f}°C")
    print(f"    Offset:     {offset:+.2f}°C")
    
    return {
        'point': point_num,
        'name': point_name,
        'esp32_temp': esp32_temp,
        'reference_temp': reference_temp,
        'offset': offset
    }


def analyze_results(results):
    """Analyze calibration results."""
    print("\n" + "=" * 70)
    print("ANÁLISIS DE RESULTADOS")
    print("=" * 70)
    
    # Calculate statistics
    offsets = [r['offset'] for r in results]
    avg_offset = statistics.mean(offsets)
    std_offset = statistics.stdev(offsets) if len(offsets) > 1 else 0
    min_offset = min(offsets)
    max_offset = max(offsets)
    
    print("\nOffset por punto:")
    for r in results:
        print(f"  {r['name']:15s}: {r['offset']:+.2f}°C")
    
    print(f"\nEstadísticas:")
    print(f"  Offset promedio:  {avg_offset:+.2f}°C")
    print(f"  Desviación estándar: {std_offset:.3f}°C")
    print(f"  Rango: [{min_offset:+.2f}, {max_offset:+.2f}]°C")
    
    # Verify linearity
    print("\nVerificación de linealidad:")
    if std_offset < 0.2:
        print(f"  ✓ EXCELENTE: σ < 0.2°C - Offset muy consistente")
        quality = "excellent"
    elif std_offset < 0.5:
        print(f"  ✓ BUENO: σ < 0.5°C - Offset aceptable")
        quality = "good"
    else:
        print(f"  ✗ ADVERTENCIA: σ >= 0.5°C - Alta variación")
        print(f"    Posibles causas:")
        print(f"      - Tiempo de estabilización insuficiente")
        print(f"      - Mezcla de agua no homogénea")
        print(f"      - Sensor con problema no lineal")
        quality = "warning"
    
    return {
        'avg_offset': avg_offset,
        'std_offset': std_offset,
        'quality': quality,
        'points': results
    }


def save_results(analysis):
    """Save calibration results to file."""
    filename = "temperature_calibration_result.txt"
    
    with open(filename, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write("RESULTADOS DE CALIBRACIÓN DE TEMPERATURA\n")
        f.write("=" * 70 + "\n\n")
        
        f.write(f"Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Termómetro de referencia: TP101\n")
        f.write(f"Método: 3-point offset calibration\n\n")
        
        f.write("Mediciones:\n")
        f.write("-" * 70 + "\n")
        for p in analysis['points']:
            f.write(f"{p['name']:15s}: ESP32={p['esp32_temp']:6.2f}°C, ")
            f.write(f"TP101={p['reference_temp']:6.2f}°C, ")
            f.write(f"Offset={p['offset']:+.2f}°C\n")
        
        f.write("\nResultados:\n")
        f.write("-" * 70 + "\n")
        f.write(f"Offset promedio:     {analysis['avg_offset']:+.2f}°C\n")
        f.write(f"Desviación estándar: {analysis['std_offset']:.3f}°C\n")
        f.write(f"Calidad:             {analysis['quality']}\n\n")
        
        f.write("Aplicar en firmware:\n")
        f.write("-" * 70 + "\n")
        f.write(f"CONFIG_BIOFLOC_TEMP_OFFSET_CELSIUS={int(analysis['avg_offset'] * 1000)}\n")
        f.write(f"  // Offset en miligrados: {analysis['avg_offset']:.2f}°C × 1000 = {int(analysis['avg_offset'] * 1000)}\n\n")
        
        f.write("Ecuación calibrada:\n")
        f.write(f"  T_calibrada = (V × 20 - 20) + {analysis['avg_offset']:.2f}\n\n")
    
    print(f"\n✓ Resultados guardados en: {filename}")


def main():
    """Main calibration routine."""
    print_header()
    
    rclpy.init()
    node = TemperatureCalibrator()
    
    try:
        # Wait for initial connection
        print("\nEsperando conexión con el sensor...")
        timeout = 10
        start = time.time()
        
        while node.current_temp is None and (time.time() - start) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if node.current_temp is None:
            print("\nERROR: No se detecta señal del sensor")
            print("Verifica:")
            print("  1. micro-ROS Agent está corriendo")
            print("  2. ESP32 está conectado y publicando")
            print("  3. Topic correcto: /biofloc/sensor_data")
            return 1
        
        print(f"✓ Sensor detectado (última lectura: {node.current_temp:.2f}°C)")
        
        # Calibrate 3 points
        results = []
        
        points = [
            (1, "agua fría", "5-10°C"),
            (2, "agua ambiente", "20-25°C"),
            (3, "agua caliente", "45-55°C")
        ]
        
        for point_num, point_name, temp_range in points:
            result = calibrate_point(node, point_num, point_name, temp_range)
            if result is None:
                print("\nERROR: Calibración fallida")
                return 1
            results.append(result)
        
        # Analyze and save
        analysis = analyze_results(results)
        save_results(analysis)
        
        print("\n" + "=" * 70)
        print("CALIBRACIÓN COMPLETADA")
        print("=" * 70)
        print("\nPróximos pasos:")
        print("  1. Revisar: temperature_calibration_result.txt")
        print("  2. Aplicar offset en firmware (main/Kconfig.projbuild)")
        print("  3. Recompilar: idf.py build")
        print("  4. Flashear: idf.py flash")
        print("\n")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n\nCalibración cancelada por usuario")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
