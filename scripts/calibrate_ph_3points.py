#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""
Calibración de pH - 3 Puntos con Verificación de Voltage Divider

Este script realiza:
1. Verificación del voltage divider con multímetro
2. Calibración de 3 puntos (pH 4.01, 6.86, 9.18)
3. Cálculo de parámetros óptimos
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import numpy as np
from collections import deque

class PHCalibrator3Point(Node):
    def __init__(self):
        super().__init__('ph_calibrator_3point')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.voltage_readings = deque(maxlen=30)
        self.last_update = time.time()
        
    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            voltage = data.get('sensors', {}).get('ph', {}).get('voltage', None)
            
            if voltage is not None:
                self.voltage_readings.append(voltage)
                self.last_update = time.time()
                
        except json.JSONDecodeError:
            pass
    
    def get_stable_voltage(self, min_wait=180, max_wait=420):
        """
        Espera voltaje estable con tiempo suficiente para el sensor CWT-BL
        
        Estrategia:
        1. Espera mínimo 3 minutos (180s) para estabilización inicial
        2. Después, espera hasta que σ < 0.002V por 25 lecturas (50s)
        3. Máximo 7 minutos (420s), luego toma promedio de últimos 60s
        """
        print("\n⏳ Esperando estabilización completa del sensor...")
        print(f"   Tiempo mínimo: {min_wait}s (~{min_wait//60} min)")
        print(f"   El sensor CWT-BL es lento, ¡paciencia!")
        print("\n   Tiempo | Voltaje    | σ (desv.) | Fase")
        print("   " + "─" * 55)
        
        start_time = time.time()
        stable_count = 0
        required_stable = 25  # 50 segundos de estabilidad
        
        self.voltage_readings.clear()
        
        while (time.time() - start_time) < max_wait:
            rclpy.spin_once(self, timeout_sec=0.5)
            
            elapsed = int(time.time() - start_time)
            
            if len(self.voltage_readings) >= 20:
                recent = list(self.voltage_readings)[-20:]
                avg = sum(recent) / len(recent)
                std_dev = (sum((x - avg) ** 2 for x in recent) / len(recent)) ** 0.5
                
                # Determinar fase
                if elapsed < min_wait:
                    phase = f"Inicial ({elapsed}/{min_wait}s)"
                    stable_count = 0
                else:
                    # Después del tiempo mínimo, buscar estabilidad
                    if std_dev < 0.002:
                        stable_count += 1
                        phase = f"Estable ({stable_count}/{required_stable})"
                        
                        if stable_count >= required_stable:
                            print(f"   {elapsed:4d}s | {avg:8.4f}V | {std_dev:8.5f} | ✓ COMPLETO")
                            # Tomar promedio de últimas 30 lecturas (60s)
                            final = list(self.voltage_readings)[-30:]
                            final_avg = sum(final) / len(final)
                            print(f"   → Voltaje final (promedio 60s): {final_avg:.4f}V")
                            return final_avg
                    else:
                        stable_count = 0
                        phase = "Estabilizando..."
                
                # Mostrar progreso cada 4 segundos
                if elapsed % 4 == 0:
                    print(f"   {elapsed:4d}s | {avg:8.4f}V | {std_dev:8.5f} | {phase}")
            
            # Check comunicación
            if (time.time() - self.last_update) > 10:
                print("\n✗ ERROR: Sin datos del sensor")
                return None
        
        # Si llegamos al máximo, tomar promedio de últimos 60s
        if len(self.voltage_readings) >= 30:
            final = list(self.voltage_readings)[-30:]
            avg = sum(final) / len(final)
            print(f"\n⏱ Tiempo máximo alcanzado. Usando promedio últimos 60s: {avg:.4f}V")
            return avg
        
        return None


def verify_voltage_divider():
    """Verifica el factor del voltage divider con multímetro"""
    print("\n" + "═" * 70)
    print("PASO 1: VERIFICAR VOLTAGE DIVIDER")
    print("═" * 70)
    
    print("\n📋 El voltage divider convierte 0-5V del sensor a 0-3.3V para ESP32")
    print("   Factor actual configurado: 1.596")
    
    print("\n🔧 Configuración del multímetro:")
    print("   1. Cable ROJO → Pin GPIO36 del ESP32")
    print("   2. Cable NEGRO → GND del ESP32")
    print("   3. Rango: 0-20V DC")
    
    print("\n📊 El sensor debe estar en alguna solución (no importa cuál)")
    
    input("\nPresiona ENTER cuando tengas el multímetro conectado...")
    
    print("\nAhora ejecuta en otra terminal:")
    print("   python3 scripts/monitor_voltage.py")
    print("\nCompara:")
    print("   • Voltaje en multímetro (V_gpio_real)")
    print("   • Voltaje en monitor (V_GPIO Software)")
    
    v_multimeter = float(input("\n📏 Ingresa el voltaje que ves en el MULTÍMETRO (V): "))
    v_software = float(input("📏 Ingresa el voltaje que muestra el MONITOR (V_GPIO Software): "))
    
    factor_actual = 1.596
    factor_correcto = v_software / v_multimeter if v_multimeter > 0 else factor_actual
    
    print(f"\n📐 Análisis:")
    print(f"   Multímetro (real):    {v_multimeter:.4f}V")
    print(f"   Software (corregido): {v_software:.4f}V")
    print(f"   Factor actual:        {factor_actual:.4f}")
    print(f"   Factor correcto:      {factor_correcto:.4f}")
    
    if abs(factor_correcto - factor_actual) > 0.1:
        print(f"\n⚠️  ADVERTENCIA: Diferencia significativa detectada!")
        print(f"   Es necesario actualizar el factor a {factor_correcto:.4f}")
        update = input("\n¿Actualizar factor? (s/n): ").strip().lower()
        
        if update == 's':
            return factor_correcto
    
    print(f"\n✓ Factor verificado: {factor_actual:.4f}")
    return factor_actual


def main():
    print("\n" + "═" * 70)
    print("  CALIBRACIÓN DE pH - 3 PUNTOS")
    print("═" * 70)
    print("\n✓ Buffers disponibles: pH 4.01, 6.86, 9.18")
    print("✓ Agua desmineralizada para limpieza")
    
    # Paso 1: Verificar voltage divider
    #divider_factor = verify_voltage_divider()
    divider_factor = 1.596  # Por ahora usamos el actual
    
    # Inicializar ROS
    rclpy.init()
    calibrator = PHCalibrator3Point()
    
    calibration_points = []
    
    try:
        # Definir los 3 puntos
        buffers = [
            (4.01, "Solución buffer pH 4.01 (ácido)"),
            (6.86, "Solución buffer pH 6.86 (neutro)"),
            (9.18, "Solución buffer pH 9.18 (alcalino)")
        ]
        
        for i, (ph_buffer, description) in enumerate(buffers, 1):
            print(f"\n" + "▶" * 35)
            print(f"PUNTO {i}/3: {description}")
            print("▶" * 35)
            
            if i == 1:
                print("\n📋 PREPARACIÓN:")
                print("   1. Enjuaga el sensor con agua desmineralizada")
                print("   2. Seca suavemente con papel absorbente")
            else:
                print("\n📋 PREPARACIÓN:")
                print("   1. RETIRA el sensor de la solución anterior")
                print("   2. Enjuaga MUY BIEN con agua desmineralizada")
                print("   3. Seca suavemente con papel absorbente")
            
            print(f"   4. Coloca el sensor en la solución buffer pH {ph_buffer}")
            print(f"   5. Agita suavemente para eliminar burbujas")
            print(f"   6. Espera 2-3 minutos a que se estabilice")
            print(f"\n   ⏱  El script esperará automáticamente el tiempo necesario")
            print(f"   📊 Puedes ver el progreso en tiempo real")
            
            input(f"\nPresiona ENTER cuando el sensor esté en pH {ph_buffer}...")
            
            print(f"\n📊 Midiendo voltaje para pH {ph_buffer}...")
            voltage = calibrator.get_stable_voltage(min_wait=180, max_wait=420)
            
            if voltage is None:
                print(f"✗ Error obteniendo voltaje para pH {ph_buffer}")
                return
            
            calibration_points.append((ph_buffer, voltage))
            print(f"\n✓ Punto {i} registrado: pH {ph_buffer} → {voltage:.4f}V")
        
        # Calcular calibración con 3 puntos (ajuste lineal por mínimos cuadrados)
        print("\n" + "═" * 70)
        print("CÁLCULO DE CALIBRACIÓN")
        print("═" * 70)
        
        ph_values = np.array([p[0] for p in calibration_points])
        voltages = np.array([p[1] for p in calibration_points])
        
        # Ajuste lineal: pH = slope * V + offset
        coeffs = np.polyfit(voltages, ph_values, 1)
        slope = coeffs[0]
        offset = coeffs[1]
        
        print(f"\n📐 Parámetros calculados (ajuste lineal):")
        print(f"   Slope:  {slope:.6f}")
        print(f"   Offset: {offset:.6f}")
        print(f"\n   Fórmula: pH = {slope:.6f} × V_sensor + {offset:.6f}")
        
        # Calcular R² (bondad de ajuste)
        ph_predicted = slope * voltages + offset
        ss_res = np.sum((ph_values - ph_predicted) ** 2)
        ss_tot = np.sum((ph_values - np.mean(ph_values)) ** 2)
        r_squared = 1 - (ss_res / ss_tot)
        
        print(f"\n📊 Calidad del ajuste:")
        print(f"   R² = {r_squared:.6f} (1.0 = perfecto)")
        
        # Verificación punto por punto
        print(f"\n✓ Verificación:")
        print(f"   {'pH Buffer':>10} | {'Voltaje':>10} | {'pH Calc.':>10} | {'Error':>10}")
        print("   " + "─" * 50)
        
        max_error = 0
        for ph_buffer, voltage in calibration_points:
            ph_calc = slope * voltage + offset
            error = abs(ph_calc - ph_buffer)
            max_error = max(max_error, error)
            print(f"   {ph_buffer:>10.2f} | {voltage:>9.4f}V | {ph_calc:>10.2f} | {error:>9.3f}")
        
        print(f"\n   Error máximo: {max_error:.3f} pH")
        
        if max_error > 0.1:
            print(f"   ⚠️  Error alto, verifica:")
            print(f"       • Limpieza entre buffers")
            print(f"       • Calidad de los buffers")
            print(f"       • Estabilización adecuada")
        else:
            print(f"   ✓ Excelente precisión!")
        
        # Verificar con medición actual (7.06)
        print(f"\n📊 Verificación con agua actual:")
        print(f"   pH medido con instrumento: 7.06")
        
        # Buscar qué voltaje daría pH 7.06
        v_for_7_06 = (7.06 - offset) / slope
        print(f"   Voltaje esperado: {v_for_7_06:.4f}V")
        print(f"   (Si el sensor lee esto en el agua actual, la calibración es correcta)")
        
        # Guardar resultados
        print("\n" + "═" * 70)
        print("APLICAR CALIBRACIÓN")
        print("═" * 70)
        
        print(f"\n📝 Código para main/main.c:")
        print(f"   (Reemplazar la línea de calibración actual)\n")
        print(f"   sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);")
        print(f"   ESP_LOGI(TAG_SENSOR, \"pH 3-point cal: R²={r_squared:.4f}, max_err={max_error:.3f}\");")
        
        # Guardar en archivo
        with open('/home/Biofloc-Firmware-ROS/calibration_3point_result.txt', 'w') as f:
            f.write(f"pH Calibration - 3 Points\n")
            f.write(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"Calibration Points:\n")
            for ph, v in calibration_points:
                f.write(f"  pH {ph:.2f} → {v:.4f}V\n")
            f.write(f"\nParameters:\n")
            f.write(f"  Slope:  {slope:.6f}\n")
            f.write(f"  Offset: {offset:.6f}\n")
            f.write(f"  R²:     {r_squared:.6f}\n")
            f.write(f"  Max Error: {max_error:.3f} pH\n\n")
            f.write(f"Formula: pH = {slope:.6f} * V_sensor + {offset:.6f}\n\n")
            f.write(f"Code for main.c:\n")
            f.write(f"sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);\n")
        
        print(f"\n💾 Resultados guardados en: calibration_3point_result.txt")
        
        print("\n" + "═" * 70)
        print("✓ CALIBRACIÓN DE 3 PUNTOS COMPLETADA")
        print("═" * 70)
        
    except KeyboardInterrupt:
        print("\n\n✗ Calibración cancelada")
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
