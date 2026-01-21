#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""
Calibración de pH en 2 Puntos - Versión Interactiva Mejorada

Este script guía paso a paso la calibración con soluciones buffer
o con el agua actual de pH conocido.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from collections import deque

class PHCalibratorV2(Node):
    def __init__(self):
        super().__init__('ph_calibrator_v2')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.voltage_readings = deque(maxlen=20)
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
    
    def get_stable_voltage(self, timeout=30):
        """Espera a que el voltaje se estabilice"""
        print("\n⏳ Esperando estabilización de lecturas...")
        print("   (Puede tomar hasta 30 segundos)")
        
        start_time = time.time()
        stable_count = 0
        required_stable = 8  # Más estricto
        
        self.voltage_readings.clear()  # Limpiar lecturas previas
        
        while (time.time() - start_time) < timeout:
            if len(self.voltage_readings) >= 10:
                recent = list(self.voltage_readings)[-10:]
                avg = sum(recent) / len(recent)
                std_dev = (sum((x - avg) ** 2 for x in recent) / len(recent)) ** 0.5
                
                # Estabilidad: desviación < 0.01V
                if std_dev < 0.01:
                    stable_count += 1
                    if stable_count >= required_stable:
                        print(f"✓ Estabilizado: {avg:.4f}V (σ={std_dev:.5f}V)")
                        return avg
                else:
                    stable_count = 0
                
                # Mostrar progreso cada 2 segundos
                elapsed = int(time.time() - start_time)
                if elapsed % 2 == 0 and elapsed > 0:
                    print(f"  [{elapsed}s] Actual: {avg:.4f}V (σ={std_dev:.4f}V) - {'Estabilizando...' if stable_count > 0 else 'Esperando...'}")
            
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # Check timeout de datos
            if (time.time() - self.last_update) > 10:
                print("\n✗ ERROR: No se reciben datos del sensor")
                print("  Verifica:")
                print("  - ESP32 está encendido y conectado")
                print("  - micro-ROS Agent está ejecutándose")
                print("  - Topic /biofloc/sensor_data está publicando")
                return None
        
        # Timeout, usar el promedio
        if self.voltage_readings:
            avg = sum(self.voltage_readings) / len(self.voltage_readings)
            print(f"⚠ Timeout: Usando promedio {avg:.4f}V")
            return avg
        
        return None


def print_header():
    print("\n" + "═" * 70)
    print("  CALIBRACIÓN DE pH EN 2 PUNTOS")
    print("═" * 70)
    print("\nEste proceso calibrará el sensor usando dos puntos de referencia")
    print("para maximizar la precisión en todo el rango de pH.")


def get_float_input(prompt, min_val=None, max_val=None, default=None):
    """Obtiene entrada validada del usuario"""
    while True:
        try:
            prompt_str = prompt
            if default is not None:
                prompt_str += f" [default: {default}]: "
            else:
                prompt_str += ": "
            
            user_input = input(prompt_str).strip()
            
            if user_input == "" and default is not None:
                return default
            
            value = float(user_input)
            
            if min_val is not None and value < min_val:
                print(f"✗ El valor debe ser >= {min_val}")
                continue
            if max_val is not None and value > max_val:
                print(f"✗ El valor debe ser <= {max_val}")
                continue
            
            return value
            
        except ValueError:
            print("✗ Entrada inválida, ingresa un número")
        except KeyboardInterrupt:
            print("\n\n✗ Calibración cancelada")
            return None


def main():
    print_header()
    
    # Inicializar ROS
    rclpy.init()
    calibrator = PHCalibratorV2()
    
    print("\n📋 Opciones de calibración:")
    print("  1. Tengo soluciones buffer pH 4.0 y pH 10.0 (recomendado)")
    print("  2. Tengo una solución buffer y el pH actual del agua (7.60)")
    print("  3. Tengo el pH actual del agua y haré otro punto después")
    
    option = input("\nElige opción (1/2/3) [default: 2]: ").strip() or "2"
    
    try:
        # PUNTO 1
        print("\n" + "▶" * 35)
        print("PASO 1: PRIMER PUNTO DE CALIBRACIÓN")
        print("▶" * 35)
        
        if option == "1":
            print("\n🧪 Solución Buffer 1: pH 4.0")
            input("\nPREPARACIÓN:")
            print("  1. Enjuaga el sensor con agua destilada")
            print("  2. Seca suavemente con papel")
            print("  3. Coloca el sensor en la solución buffer pH 4.0")
            print("  4. Espera a que se estabilice")
            input("\nPresiona ENTER cuando el sensor esté en la solución...")
            
            ph_low = 4.0
            
        elif option == "2":
            print("\n🧪 Opción: Agua actual (pH conocido)")
            print("\n¿El sensor está actualmente en el agua de pH 7.60?")
            in_water = input("(s/n) [default: s]: ").strip().lower() or "s"
            
            if in_water == "n":
                print("\nPREPARACIÓN:")
                print("  1. Coloca el sensor en el agua de pH 7.60")
                print("  2. Espera a que se estabilice")
                input("\nPresiona ENTER cuando esté listo...")
            
            ph_low = get_float_input("\nIngresa el pH del agua (medido con instrumento)", 0.0, 14.0, 7.60)
            if ph_low is None:
                return
            
        else:  # option 3
            ph_low = get_float_input("\nIngresa el pH actual del agua", 0.0, 14.0, 7.60)
            if ph_low is None:
                return
            print("\nEl sensor debe estar en esta agua.")
        
        print(f"\n📊 Midiendo voltaje para pH {ph_low:.2f}...")
        voltage_low = calibrator.get_stable_voltage(timeout=45)
        
        if voltage_low is None:
            print("✗ Error: No se pudo obtener voltaje estable")
            return
        
        print(f"\n✓ Punto 1 registrado:")
        print(f"  pH {ph_low:.2f} → {voltage_low:.4f}V")
        
        # PUNTO 2
        print("\n" + "▶" * 35)
        print("PASO 2: SEGUNDO PUNTO DE CALIBRACIÓN")
        print("▶" * 35)
        
        if option == "1":
            print("\n🧪 Solución Buffer 2: pH 10.0")
            input("\nPREPARACIÓN:")
            print("  1. RETIRA el sensor de la solución buffer pH 4.0")
            print("  2. Enjuaga MUY BIEN con agua destilada")
            print("  3. Seca suavemente con papel")
            print("  4. Coloca el sensor en la solución buffer pH 10.0")
            print("  5. Espera a que se estabilice")
            input("\nPresiona ENTER cuando el sensor esté en pH 10.0...")
            
            ph_high = 10.0
            
        else:  # option 2 o 3
            print("\n🧪 Segunda medición: Solución buffer")
            print("\nOpciones comunes:")
            print("  • pH 4.0 (buffer ácido)")
            print("  • pH 10.0 (buffer alcalino)")
            
            ph_high = get_float_input("\nIngresa el pH de la solución buffer", 0.0, 14.0, 10.0)
            if ph_high is None:
                return
            
            if abs(ph_high - ph_low) < 2.0:
                print(f"\n⚠ ADVERTENCIA: Los puntos están muy cerca ({abs(ph_high - ph_low):.1f} pH)")
                print("   Para mejor calibración, usa puntos más separados (≥4 pH)")
                cont = input("   ¿Continuar de todos modos? (s/n): ").strip().lower()
                if cont != "s":
                    return
            
            input("\nPREPARACIÓN:")
            print(f"  1. RETIRA el sensor del agua")
            print(f"  2. Enjuaga MUY BIEN con agua destilada")
            print(f"  3. Seca suavemente con papel")
            print(f"  4. Coloca el sensor en la solución buffer pH {ph_high:.1f}")
            print(f"  5. Espera a que se estabilice")
            input(f"\nPresiona ENTER cuando el sensor esté en pH {ph_high:.1f}...")
        
        print(f"\n📊 Midiendo voltaje para pH {ph_high:.2f}...")
        voltage_high = calibrator.get_stable_voltage(timeout=45)
        
        if voltage_high is None:
            print("✗ Error: No se pudo obtener voltaje estable")
            return
        
        # Validar que los voltajes sean diferentes
        if abs(voltage_high - voltage_low) < 0.1:
            print(f"\n✗ ERROR: Los voltajes son muy similares")
            print(f"  Punto 1: {voltage_low:.4f}V")
            print(f"  Punto 2: {voltage_high:.4f}V")
            print(f"  Diferencia: {abs(voltage_high - voltage_low):.4f}V")
            print("\n  Posibles causas:")
            print("  • El sensor no se estabilizó completamente")
            print("  • No se enjuagó bien entre mediciones")
            print("  • Sensor defectuoso")
            return
        
        print(f"\n✓ Punto 2 registrado:")
        print(f"  pH {ph_high:.2f} → {voltage_high:.4f}V")
        
        # CÁLCULO DE CALIBRACIÓN
        print("\n" + "═" * 70)
        print("RESULTADOS DE CALIBRACIÓN")
        print("═" * 70)
        
        # Asegurar orden correcto (low < high)
        if ph_low > ph_high:
            ph_low, ph_high = ph_high, ph_low
            voltage_low, voltage_high = voltage_high, voltage_low
            print(f"\n(Puntos reordenados: bajo → alto)")
        
        # Calcular slope y offset
        slope = (ph_high - ph_low) / (voltage_high - voltage_low)
        offset = ph_low - (slope * voltage_low)
        
        print(f"\n📐 Parámetros calculados:")
        print(f"  Slope:  {slope:.6f}")
        print(f"  Offset: {offset:.6f}")
        print(f"\n  Fórmula: pH = {slope:.6f} × V_sensor + ({offset:.6f})")
        
        # Verificación
        print(f"\n✓ Verificación:")
        calc_ph_low = slope * voltage_low + offset
        calc_ph_high = slope * voltage_high + offset
        print(f"  En {voltage_low:.4f}V → pH {calc_ph_low:.2f} (esperado: {ph_low:.2f}, error: {abs(calc_ph_low - ph_low):.3f})")
        print(f"  En {voltage_high:.4f}V → pH {calc_ph_high:.2f} (esperado: {ph_high:.2f}, error: {abs(calc_ph_high - ph_high):.3f})")
        
        # Comparación con fórmula anterior
        print(f"\n📊 Comparación:")
        print(f"  Fórmula anterior (datasheet): pH = 2.8000 × V")
        print(f"  Fórmula calibrada:            pH = {slope:.4f} × V + {offset:.4f}")
        
        # Código para aplicar
        print("\n" + "═" * 70)
        print("APLICAR CALIBRACIÓN")
        print("═" * 70)
        
        print("\n📝 Opción 1: Código en firmware (main/main.c)")
        print("   Agregar después de sensors_init():")
        print(f"\n   sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);")
        print(f"   ESP_LOGI(TAG_MAIN, \"pH calibration applied: slope={slope:.4f}, offset={offset:.4f}\");")
        
        print("\n📝 Opción 2: Comando para copiar a main.c:")
        print(f'\n   echo "sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);" >> main/main.c')
        
        # Guardar en archivo
        with open('/home/Biofloc-Firmware-ROS/calibration_result.txt', 'w') as f:
            f.write(f"# pH Calibration Results - {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            f.write(f"Calibration Points:\n")
            f.write(f"  Point 1: pH {ph_low:.2f} at {voltage_low:.4f}V\n")
            f.write(f"  Point 2: pH {ph_high:.2f} at {voltage_high:.4f}V\n\n")
            f.write(f"Calculated Parameters:\n")
            f.write(f"  Slope:  {slope:.6f}\n")
            f.write(f"  Offset: {offset:.6f}\n\n")
            f.write(f"Formula: pH = {slope:.6f} * V_sensor + {offset:.6f}\n\n")
            f.write(f"Code for main.c:\n")
            f.write(f"  sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);\n")
        
        print(f"\n💾 Resultados guardados en: calibration_result.txt")
        
        print("\n" + "═" * 70)
        print("✓ CALIBRACIÓN COMPLETADA")
        print("═" * 70)
        print("\nPróximos pasos:")
        print("  1. Editar main/main.c y agregar la línea de calibración")
        print("  2. Recompilar: idf.py build")
        print("  3. Flashear: idf.py flash")
        print("  4. Verificar con agua de pH conocido")
        
    except KeyboardInterrupt:
        print("\n\n✗ Calibración cancelada por usuario")
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
