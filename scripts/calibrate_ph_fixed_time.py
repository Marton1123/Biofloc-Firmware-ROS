#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""
Calibración pH - Método con tiempo fijo y promedio final
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import numpy as np
from collections import deque

class PHCalibratorFixed(Node):
    def __init__(self):
        super().__init__('ph_calibrator_fixed')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.voltage_readings = []
        self.last_update = time.time()
        
    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            voltage = data.get('sensors', {}).get('ph', {}).get('voltage', None)
            
            if voltage is not None:
                self.voltage_readings.append({
                    'time': time.time(),
                    'voltage': voltage
                })
                self.last_update = time.time()
                
        except json.JSONDecodeError:
            pass
    
    def wait_and_average(self, wait_time=180, average_window=30):
        """Espera tiempo fijo y promedia últimas lecturas"""
        print(f"\n⏳ Método: Esperar {wait_time}s, luego promediar últimos {average_window}s")
        print("   Monitoreo cada 10 segundos...")
        print("\n   Tiempo | Voltaje | Tendencia")
        print("   " + "─" * 40)
        
        self.voltage_readings.clear()
        start_time = time.time()
        last_display = 0
        
        while (time.time() - start_time) < wait_time:
            rclpy.spin_once(self, timeout_sec=0.5)
            
            elapsed = int(time.time() - start_time)
            
            # Mostrar cada 10 segundos
            if elapsed - last_display >= 10 and self.voltage_readings:
                recent = [r['voltage'] for r in self.voltage_readings[-10:]]
                avg = sum(recent) / len(recent)
                
                # Calcular tendencia
                if len(self.voltage_readings) >= 20:
                    last_20 = [r['voltage'] for r in self.voltage_readings[-20:]]
                    first_10 = sum(last_20[:10]) / 10
                    last_10 = sum(last_20[10:]) / 10
                    trend = last_10 - first_10
                    
                    if abs(trend) < 0.01:
                        trend_str = "Estable ✓"
                    elif trend > 0:
                        trend_str = f"Subiendo (+{trend:.3f}V)"
                    else:
                        trend_str = f"Bajando ({trend:.3f}V)"
                else:
                    trend_str = "..."
                
                print(f"   {elapsed:4d}s | {avg:.4f}V | {trend_str}")
                last_display = elapsed
            
            if (time.time() - self.last_update) > 10:
                print("\n✗ ERROR: Sin datos del sensor")
                return None
        
        # Promediar últimas lecturas
        cutoff_time = time.time() - average_window
        final_readings = [r['voltage'] for r in self.voltage_readings if r['time'] >= cutoff_time]
        
        if len(final_readings) < 5:
            print(f"\n⚠ Pocas lecturas en ventana final ({len(final_readings)})")
            if self.voltage_readings:
                final_readings = [r['voltage'] for r in self.voltage_readings[-20:]]
        
        avg_voltage = sum(final_readings) / len(final_readings)
        std_voltage = (sum((v - avg_voltage) ** 2 for v in final_readings) / len(final_readings)) ** 0.5
        
        print(f"\n✓ Promedio final: {avg_voltage:.4f}V (σ={std_voltage:.5f}V, n={len(final_readings)})")
        
        return avg_voltage


def main():
    print("\n" + "═" * 70)
    print("  CALIBRACIÓN pH - MÉTODO CON TIEMPO FIJO")
    print("═" * 70)
    print("\n✓ Este método espera 3 minutos por buffer para estabilización completa")
    print("✓ Buffers: pH 4.01, 6.86, 9.18")
    
    rclpy.init()
    calibrator = PHCalibratorFixed()
    
    calibration_points = []
    
    try:
        buffers = [
            (4.01, "pH 4.01 (ácido)"),
            (6.86, "pH 6.86 (neutro)"),
            (9.18, "pH 9.18 (alcalino)")
        ]
        
        for i, (ph_buffer, description) in enumerate(buffers, 1):
            print(f"\n" + "▶" * 35)
            print(f"BUFFER {i}/3: {description}")
            print("▶" * 35)
            
            if i == 1:
                print("\n📋 PREPARACIÓN:")
                print("   1. Enjuaga el sensor con agua desmineralizada")
                print("   2. Seca suavemente")
            else:
                print("\n📋 PREPARACIÓN:")
                print("   1. RETIRA el sensor")
                print("   2. Enjuaga MUY BIEN con agua desmineralizada (mínimo 30 segundos)")
                print("   3. Seca suavemente")
            
            print(f"   4. Coloca el sensor en buffer {description}")
            print(f"   5. Agita suavemente")
            print(f"   6. ESPERA 3 minutos completos sin tocar")
            
            input(f"\nPresiona ENTER cuando esté en {description}...")
            
            print(f"\n📊 Esperando estabilización (3 minutos)...")
            voltage = calibrator.wait_and_average(wait_time=180, average_window=30)
            
            if voltage is None:
                print(f"✗ Error en {description}")
                return
            
            calibration_points.append((ph_buffer, voltage))
            print(f"\n✓ {description} → {voltage:.4f}V")
        
        # Calcular calibración
        print("\n" + "═" * 70)
        print("RESULTADOS")
        print("═" * 70)
        
        ph_values = np.array([p[0] for p in calibration_points])
        voltages = np.array([p[1] for p in calibration_points])
        
        coeffs = np.polyfit(voltages, ph_values, 1)
        slope = coeffs[0]
        offset = coeffs[1]
        
        print(f"\n📐 Parámetros:")
        print(f"   Slope:  {slope:.6f}")
        print(f"   Offset: {offset:.6f}")
        print(f"   Fórmula: pH = {slope:.6f} × V + {offset:.6f}")
        
        # R²
        ph_predicted = slope * voltages + offset
        ss_res = np.sum((ph_values - ph_predicted) ** 2)
        ss_tot = np.sum((ph_values - np.mean(ph_values)) ** 2)
        r_squared = 1 - (ss_res / ss_tot)
        print(f"   R² = {r_squared:.6f}")
        
        # Verificación
        print(f"\n✓ Verificación:")
        print(f"   {'pH':>6} | {'Voltaje':>10} | {'pH Calc':>9} | {'Error':>8}")
        print("   " + "─" * 40)
        
        max_error = 0
        for ph_buf, v in calibration_points:
            ph_calc = slope * v + offset
            error = abs(ph_calc - ph_buf)
            max_error = max(max_error, error)
            print(f"   {ph_buf:>6.2f} | {v:>9.4f}V | {ph_calc:>9.2f} | {error:>7.3f}")
        
        print(f"\n   Error máximo: {max_error:.3f} pH")
        
        if max_error < 0.1:
            print("   ✓ ¡Excelente precisión!")
        elif max_error < 0.2:
            print("   ✓ Buena precisión")
        else:
            print("   ⚠ Precisión aceptable")
        
        # Código
        print("\n" + "═" * 70)
        print("CÓDIGO PARA APLICAR")
        print("═" * 70)
        
        print(f"\n📝 En main/main.c (reemplazar línea de calibración):\n")
        print(f"   sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);")
        print(f"   ESP_LOGI(TAG_SENSOR, \"pH cal: R²={r_squared:.4f}, err={max_error:.3f}\");")
        
        # Guardar
        with open('/home/Biofloc-Firmware-ROS/calibration_final.txt', 'w') as f:
            f.write(f"pH Calibration - Fixed Time Method\n")
            f.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"Points:\n")
            for ph, v in calibration_points:
                f.write(f"  pH {ph:.2f} → {v:.4f}V\n")
            f.write(f"\nParameters:\n")
            f.write(f"  Slope: {slope:.6f}\n")
            f.write(f"  Offset: {offset:.6f}\n")
            f.write(f"  R²: {r_squared:.6f}\n")
            f.write(f"  Max Error: {max_error:.3f} pH\n\n")
            f.write(f"Code:\n")
            f.write(f"sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);\n")
        
        print(f"\n💾 Guardado en: calibration_final.txt")
        print("\n" + "═" * 70)
        
    except KeyboardInterrupt:
        print("\n\n✗ Cancelado")
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
