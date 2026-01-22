#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
  Biofloc pH Sensor Calibration Tool v3.0
═══════════════════════════════════════════════════════════════════════════════

Professional calibration tool for CWT-BL pH sensors with ESP32/micro-ROS.

Features:
  • 1, 2, or 3-point calibration modes
  • Quick calibration (1 min) or precision mode (3+ min per point)
  • Statistical validation with R² and error analysis
  • Automatic voltage stabilization detection
  • Generates code for firmware integration
  • Saves calibration history

Usage:
  python3 calibrate_ph.py [--quick] [--verify]

Requirements:
  • micro-ROS Agent running: micro_ros_agent udp4 --port 8888
  • ESP32 publishing to /biofloc/sensor_data
  • pH buffer solutions (pH 4.01, 6.86, 9.18 recommended)

Author: Biofloc Systems Lab
Date: 2026-01-22
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys
import argparse
from datetime import datetime
from collections import deque
from typing import Optional, List, Tuple


class Colors:
    """ANSI color codes for terminal output."""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RESET = '\033[0m'


class PHCalibrationNode(Node):
    """ROS2 node for pH sensor calibration."""
    
    def __init__(self):
        super().__init__('ph_calibration')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self._sensor_callback,
            10
        )
        
        self.voltage_buffer = deque(maxlen=60)
        self.last_update = 0.0
        self.connected = False
        
    def _sensor_callback(self, msg: String):
        """Process incoming sensor data."""
        try:
            data = json.loads(msg.data)
            voltage = data.get('sensors', {}).get('ph', {}).get('voltage')
            
            if voltage is not None:
                self.voltage_buffer.append({
                    'voltage': voltage,
                    'timestamp': time.time()
                })
                self.last_update = time.time()
                self.connected = True
                
        except json.JSONDecodeError:
            pass
    
    def wait_for_connection(self, timeout: float = 15.0) -> bool:
        """Wait for sensor data to arrive."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.connected:
                return True
        return False
    
    def get_stable_voltage(self, 
                          min_wait: int = 60, 
                          max_wait: int = 300,
                          stability_threshold: float = 0.005,
                          stability_count: int = 15) -> Optional[float]:
        """
        Wait for voltage to stabilize.
        
        Args:
            min_wait: Minimum seconds to wait before checking stability
            max_wait: Maximum seconds to wait
            stability_threshold: Max std dev for stable reading (V)
            stability_count: Number of consecutive stable readings required
        
        Returns:
            Stable voltage or None if timeout
        """
        self.voltage_buffer.clear()
        start_time = time.time()
        stable_streak = 0
        
        print(f"\n  {'Tiempo':>7} │ {'Voltaje':>9} │ {'σ (desv)':>9} │ Estado")
        print(f"  {'─'*7}─┼─{'─'*9}─┼─{'─'*9}─┼─{'─'*20}")
        
        last_print = 0
        
        while (time.time() - start_time) < max_wait:
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = int(time.time() - start_time)
            
            # Check for data timeout
            if self.last_update > 0 and (time.time() - self.last_update) > 10:
                print(f"\n  {Colors.RED}✗ Sin datos del sensor (timeout){Colors.RESET}")
                return None
            
            if len(self.voltage_buffer) >= 10:
                recent = [r['voltage'] for r in list(self.voltage_buffer)[-20:]]
                avg = sum(recent) / len(recent)
                std_dev = (sum((x - avg) ** 2 for x in recent) / len(recent)) ** 0.5
                
                # Determine phase and status
                if elapsed < min_wait:
                    phase = f"Esperando ({elapsed}/{min_wait}s)"
                    stable_streak = 0
                    status_color = Colors.DIM
                else:
                    if std_dev < stability_threshold:
                        stable_streak += 1
                        if stable_streak >= stability_count:
                            # Get final average from last 30 readings
                            final_readings = [r['voltage'] for r in list(self.voltage_buffer)[-30:]]
                            if len(final_readings) < 10:
                                final_readings = [r['voltage'] for r in list(self.voltage_buffer)]
                            final_avg = sum(final_readings) / len(final_readings)
                            final_std = (sum((x - final_avg) ** 2 for x in final_readings) / len(final_readings)) ** 0.5
                            
                            print(f"  {elapsed:>6}s │ {final_avg:>8.4f}V │ {final_std:>8.5f}V │ {Colors.GREEN}✓ ESTABLE{Colors.RESET}")
                            return final_avg
                        
                        phase = f"Estable ({stable_streak}/{stability_count})"
                        status_color = Colors.CYAN
                    else:
                        stable_streak = 0
                        phase = "Estabilizando..."
                        status_color = Colors.YELLOW
                
                # Print progress every 3 seconds
                if elapsed - last_print >= 3:
                    print(f"  {elapsed:>6}s │ {avg:>8.4f}V │ {std_dev:>8.5f}V │ {status_color}{phase}{Colors.RESET}")
                    last_print = elapsed
        
        # Timeout - use average of last readings
        if len(self.voltage_buffer) >= 10:
            final_readings = [r['voltage'] for r in list(self.voltage_buffer)[-30:]]
            if len(final_readings) < 5:
                final_readings = [r['voltage'] for r in list(self.voltage_buffer)]
            final_avg = sum(final_readings) / len(final_readings)
            print(f"\n  {Colors.YELLOW}⏱ Timeout: usando promedio {final_avg:.4f}V{Colors.RESET}")
            return final_avg
        
        return None


class PHCalibrator:
    """Main calibration controller."""
    
    def __init__(self, quick_mode: bool = False):
        self.quick_mode = quick_mode
        self.node: Optional[PHCalibrationNode] = None
        self.calibration_points: List[Tuple[float, float]] = []
        
        # Timing settings
        if quick_mode:
            self.min_wait = 30
            self.max_wait = 90
            self.stability_threshold = 0.010
            self.stability_count = 8
        else:
            self.min_wait = 60
            self.max_wait = 300
            self.stability_threshold = 0.005
            self.stability_count = 15
    
    def print_header(self):
        """Print application header."""
        print(f"\n{Colors.BOLD}{'═' * 70}")
        print(f"  BIOFLOC pH CALIBRATION TOOL v3.0")
        print(f"{'═' * 70}{Colors.RESET}")
        print(f"\n  Modo: {'⚡ Rápido (~1 min/punto)' if self.quick_mode else '🔬 Precisión (~3 min/punto)'}")
        print(f"  Sensor: CWT-BL 0-5V → ESP32 GPIO36")
        print(f"  Topic: /biofloc/sensor_data")
    
    def print_menu(self) -> str:
        """Show calibration options menu."""
        print(f"\n{Colors.CYAN}┌{'─' * 68}┐")
        print(f"│ {'SELECCIONA TIPO DE CALIBRACIÓN':^66} │")
        print(f"├{'─' * 68}┤{Colors.RESET}")
        print(f"│  {Colors.BOLD}1{Colors.RESET}. Calibración rápida (1 punto)                                    │")
        print(f"│     → Ajusta offset usando pH conocido actual                       │")
        print(f"│                                                                      │")
        print(f"│  {Colors.BOLD}2{Colors.RESET}. Calibración estándar (2 puntos)                                 │")
        print(f"│     → pH 4.01 + pH 9.18 (o cualquier par)                           │")
        print(f"│                                                                      │")
        print(f"│  {Colors.BOLD}3{Colors.RESET}. Calibración de precisión (3 puntos)                             │")
        print(f"│     → pH 4.01 + pH 6.86 + pH 9.18                                   │")
        print(f"│     → Calcula R² y error máximo                                     │")
        print(f"│                                                                      │")
        print(f"│  {Colors.BOLD}V{Colors.RESET}. Solo verificar lectura actual                                   │")
        print(f"│                                                                      │")
        print(f"│  {Colors.BOLD}Q{Colors.RESET}. Salir                                                           │")
        print(f"{Colors.CYAN}└{'─' * 68}┘{Colors.RESET}")
        
        return input(f"\n  Opción [1/2/3/V/Q]: ").strip().upper() or '2'
    
    def get_float_input(self, prompt: str, min_val: float = 0.0, 
                       max_val: float = 14.0, default: Optional[float] = None) -> Optional[float]:
        """Get validated float input from user."""
        while True:
            try:
                if default is not None:
                    user_input = input(f"  {prompt} [{default}]: ").strip()
                    if user_input == "":
                        return default
                else:
                    user_input = input(f"  {prompt}: ").strip()
                
                value = float(user_input)
                
                if value < min_val or value > max_val:
                    print(f"  {Colors.RED}✗ Valor debe estar entre {min_val} y {max_val}{Colors.RESET}")
                    continue
                
                return value
                
            except ValueError:
                print(f"  {Colors.RED}✗ Ingresa un número válido{Colors.RESET}")
            except KeyboardInterrupt:
                return None
    
    def collect_calibration_point(self, ph_value: float, description: str) -> Optional[float]:
        """Collect a single calibration point."""
        print(f"\n{Colors.BOLD}{'▶' * 35}")
        print(f"  {description}")
        print(f"{'▶' * 35}{Colors.RESET}")
        
        print(f"\n  📋 Preparación:")
        if len(self.calibration_points) == 0:
            print(f"     1. Enjuaga el sensor con agua destilada")
            print(f"     2. Seca suavemente con papel absorbente")
        else:
            print(f"     1. {Colors.YELLOW}RETIRA{Colors.RESET} el sensor de la solución anterior")
            print(f"     2. Enjuaga {Colors.BOLD}MUY BIEN{Colors.RESET} con agua destilada")
            print(f"     3. Seca suavemente con papel absorbente")
        
        print(f"     4. Coloca el sensor en la solución pH {ph_value}")
        print(f"     5. Agita suavemente para eliminar burbujas")
        
        try:
            input(f"\n  Presiona {Colors.BOLD}ENTER{Colors.RESET} cuando el sensor esté en pH {ph_value}...")
        except KeyboardInterrupt:
            return None
        
        print(f"\n  📊 Midiendo voltaje para pH {ph_value}...")
        
        voltage = self.node.get_stable_voltage(
            min_wait=self.min_wait,
            max_wait=self.max_wait,
            stability_threshold=self.stability_threshold,
            stability_count=self.stability_count
        )
        
        if voltage is None:
            print(f"\n  {Colors.RED}✗ Error: No se pudo obtener voltaje estable{Colors.RESET}")
            return None
        
        self.calibration_points.append((ph_value, voltage))
        print(f"\n  {Colors.GREEN}✓ Punto registrado: pH {ph_value:.2f} → {voltage:.4f}V{Colors.RESET}")
        
        return voltage
    
    def calculate_calibration(self) -> Tuple[float, float, dict]:
        """
        Calculate calibration parameters using least squares.
        
        Returns:
            (slope, offset, stats_dict)
        """
        if len(self.calibration_points) < 1:
            raise ValueError("Se necesita al menos 1 punto de calibración")
        
        ph_values = [p[0] for p in self.calibration_points]
        voltages = [p[1] for p in self.calibration_points]
        
        if len(self.calibration_points) == 1:
            # Single point: adjust offset only, use default slope from datasheet
            default_slope = 2.8
            ph, v = self.calibration_points[0]
            offset = ph - (default_slope * v)
            slope = default_slope
            r_squared = None
            max_error = 0.0
            rms_error = 0.0
        else:
            # Multi-point: linear regression
            n = len(self.calibration_points)
            sum_v = sum(voltages)
            sum_ph = sum(ph_values)
            sum_v2 = sum(v**2 for v in voltages)
            sum_v_ph = sum(v * ph for v, ph in zip(voltages, ph_values))
            
            # Least squares formula
            denominator = n * sum_v2 - sum_v ** 2
            if abs(denominator) < 1e-10:
                raise ValueError("Voltajes demasiado similares para calibrar")
            
            slope = (n * sum_v_ph - sum_v * sum_ph) / denominator
            offset = (sum_ph - slope * sum_v) / n
            
            # Calculate R² and errors
            ph_predicted = [slope * v + offset for v in voltages]
            ss_res = sum((actual - pred) ** 2 for actual, pred in zip(ph_values, ph_predicted))
            ss_tot = sum((ph - sum_ph/n) ** 2 for ph in ph_values)
            
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 1.0
            
            errors = [abs(actual - pred) for actual, pred in zip(ph_values, ph_predicted)]
            max_error = max(errors)
            rms_error = (sum(e**2 for e in errors) / n) ** 0.5
        
        stats = {
            'r_squared': r_squared,
            'max_error': max_error,
            'rms_error': rms_error,
            'n_points': len(self.calibration_points)
        }
        
        return slope, offset, stats
    
    def print_results(self, slope: float, offset: float, stats: dict):
        """Print calibration results."""
        print(f"\n{Colors.BOLD}{'═' * 70}")
        print(f"  RESULTADOS DE CALIBRACIÓN")
        print(f"{'═' * 70}{Colors.RESET}")
        
        print(f"\n  {Colors.CYAN}📐 Parámetros calculados:{Colors.RESET}")
        print(f"     Slope:  {slope:.6f}")
        print(f"     Offset: {offset:.6f}")
        print(f"\n     Fórmula: {Colors.BOLD}pH = {slope:.6f} × V + {offset:.6f}{Colors.RESET}")
        
        if stats['n_points'] > 1:
            print(f"\n  {Colors.CYAN}📊 Estadísticas:{Colors.RESET}")
            if stats['r_squared'] is not None:
                r2_color = Colors.GREEN if stats['r_squared'] > 0.999 else Colors.YELLOW
                print(f"     R²:         {r2_color}{stats['r_squared']:.6f}{Colors.RESET} (1.0 = perfecto)")
            
            err_color = Colors.GREEN if stats['max_error'] < 0.05 else Colors.YELLOW
            print(f"     Error máx:  {err_color}{stats['max_error']:.4f} pH{Colors.RESET}")
            print(f"     Error RMS:  {stats['rms_error']:.4f} pH")
        
        # Verification table
        print(f"\n  {Colors.CYAN}✓ Verificación punto por punto:{Colors.RESET}")
        print(f"     {'pH Real':>9} │ {'Voltaje':>10} │ {'pH Calc.':>9} │ {'Error':>8}")
        print(f"     {'─'*9}─┼─{'─'*10}─┼─{'─'*9}─┼─{'─'*8}")
        
        for ph_real, voltage in self.calibration_points:
            ph_calc = slope * voltage + offset
            error = ph_calc - ph_real
            err_color = Colors.GREEN if abs(error) < 0.05 else Colors.YELLOW
            print(f"     {ph_real:>9.2f} │ {voltage:>9.4f}V │ {ph_calc:>9.3f} │ {err_color}{error:>+7.3f}{Colors.RESET}")
    
    def print_code_output(self, slope: float, offset: float):
        """Print code for firmware integration."""
        print(f"\n{Colors.BOLD}{'─' * 70}")
        print(f"  CÓDIGO PARA FIRMWARE")
        print(f"{'─' * 70}{Colors.RESET}")
        
        print(f"\n  📝 Agregar en {Colors.BOLD}main/main.c{Colors.RESET} después de sensors_init():\n")
        print(f"     {Colors.GREEN}sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);{Colors.RESET}")
        
        print(f"\n  📝 Después recompilar y flashear:")
        print(f"     {Colors.DIM}idf.py build && idf.py flash{Colors.RESET}")
    
    def save_results(self, slope: float, offset: float, stats: dict):
        """Save calibration results to file."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        filename = '/home/Biofloc-Firmware-ROS/calibration_result.txt'
        
        with open(filename, 'w') as f:
            f.write(f"{'='*60}\n")
            f.write(f"pH CALIBRATION RESULTS\n")
            f.write(f"{'='*60}\n")
            f.write(f"Date: {timestamp}\n")
            f.write(f"Mode: {'Quick' if self.quick_mode else 'Precision'}\n")
            f.write(f"Points: {stats['n_points']}\n\n")
            
            f.write(f"Calibration Points:\n")
            for ph, v in self.calibration_points:
                ph_calc = slope * v + offset
                f.write(f"  pH {ph:.2f} → {v:.4f}V → Calc: {ph_calc:.3f} (err: {ph_calc-ph:+.3f})\n")
            
            f.write(f"\nParameters:\n")
            f.write(f"  Slope:     {slope:.6f}\n")
            f.write(f"  Offset:    {offset:.6f}\n")
            
            if stats['r_squared'] is not None:
                f.write(f"  R²:        {stats['r_squared']:.6f}\n")
            f.write(f"  Max Error: {stats['max_error']:.4f} pH\n")
            f.write(f"  RMS Error: {stats['rms_error']:.4f} pH\n")
            
            f.write(f"\nFormula:\n")
            f.write(f"  pH = {slope:.6f} × V_sensor + {offset:.6f}\n")
            
            f.write(f"\nCode for main.c:\n")
            f.write(f"  sensors_calibrate_ph_manual({slope:.6f}f, {offset:.6f}f);\n")
            
            f.write(f"\n{'='*60}\n")
        
        print(f"\n  💾 Resultados guardados en: {Colors.DIM}{filename}{Colors.RESET}")
    
    def run_verification(self):
        """Run verification mode (read current pH without calibrating)."""
        print(f"\n{Colors.BOLD}{'═' * 70}")
        print(f"  MODO VERIFICACIÓN")
        print(f"{'═' * 70}{Colors.RESET}")
        
        print(f"\n  Leyendo valores actuales del sensor...")
        print(f"  (Presiona Ctrl+C para terminar)\n")
        
        try:
            while True:
                rclpy.spin_once(self.node, timeout_sec=1.0)
                
                if len(self.node.voltage_buffer) > 0:
                    recent = [r['voltage'] for r in list(self.node.voltage_buffer)[-10:]]
                    avg_v = sum(recent) / len(recent)
                    std_v = (sum((x - avg_v) ** 2 for x in recent) / len(recent)) ** 0.5 if len(recent) > 1 else 0
                    
                    # Calculate pH with current calibration (default slope 2.8)
                    ph_default = 2.8 * avg_v
                    
                    print(f"\r  Voltaje: {avg_v:.4f}V (σ={std_v:.4f})  │  pH (sin cal): {ph_default:.2f}  ", end='', flush=True)
                
        except KeyboardInterrupt:
            print(f"\n\n  {Colors.GREEN}✓ Verificación terminada{Colors.RESET}")
    
    def run_1point_calibration(self):
        """Run 1-point calibration (offset adjustment only)."""
        print(f"\n{Colors.BOLD}  CALIBRACIÓN DE 1 PUNTO (Ajuste de offset){Colors.RESET}")
        print(f"  ─────────────────────────────────────────")
        
        print(f"\n  El sensor debe estar en una solución de pH conocido.")
        print(f"  Esto ajustará el offset manteniendo la pendiente del datasheet.")
        
        ph_value = self.get_float_input("Ingresa el pH de la solución actual", 0, 14, 7.0)
        if ph_value is None:
            return False
        
        voltage = self.collect_calibration_point(ph_value, f"PUNTO ÚNICO: pH {ph_value}")
        if voltage is None:
            return False
        
        return True
    
    def run_2point_calibration(self):
        """Run standard 2-point calibration."""
        print(f"\n{Colors.BOLD}  CALIBRACIÓN DE 2 PUNTOS{Colors.RESET}")
        print(f"  ───────────────────────")
        
        print(f"\n  Buffers recomendados: pH 4.01 (ácido) + pH 9.18 (alcalino)")
        print(f"  También puedes usar cualquier par de soluciones conocidas.")
        
        # Point 1
        ph1 = self.get_float_input("pH del primer buffer (ácido)", 0, 14, 4.01)
        if ph1 is None:
            return False
        
        if self.collect_calibration_point(ph1, f"PUNTO 1/2: Buffer pH {ph1}") is None:
            return False
        
        # Point 2
        ph2 = self.get_float_input("pH del segundo buffer (alcalino)", 0, 14, 9.18)
        if ph2 is None:
            return False
        
        if abs(ph2 - ph1) < 2.0:
            print(f"\n  {Colors.YELLOW}⚠ Los puntos están cerca ({abs(ph2-ph1):.1f} pH). "
                  f"Recomendado: ≥4 pH de diferencia{Colors.RESET}")
        
        if self.collect_calibration_point(ph2, f"PUNTO 2/2: Buffer pH {ph2}") is None:
            return False
        
        return True
    
    def run_3point_calibration(self):
        """Run precision 3-point calibration."""
        print(f"\n{Colors.BOLD}  CALIBRACIÓN DE 3 PUNTOS (Precisión){Colors.RESET}")
        print(f"  ────────────────────────────────────")
        
        print(f"\n  Buffers estándar: pH 4.01, 6.86, 9.18")
        print(f"  Esto permite calcular R² y validar linealidad del sensor.")
        
        buffers = [
            (4.01, "PUNTO 1/3: Buffer ÁCIDO (pH 4.01)"),
            (6.86, "PUNTO 2/3: Buffer NEUTRO (pH 6.86)"),
            (9.18, "PUNTO 3/3: Buffer ALCALINO (pH 9.18)")
        ]
        
        for i, (default_ph, description) in enumerate(buffers):
            ph = self.get_float_input(f"pH del buffer {i+1}", 0, 14, default_ph)
            if ph is None:
                return False
            
            desc_updated = description.replace(str(default_ph), f"{ph:.2f}")
            if self.collect_calibration_point(ph, desc_updated) is None:
                return False
        
        return True
    
    def run(self):
        """Main execution loop."""
        self.print_header()
        
        # Initialize ROS
        rclpy.init()
        self.node = PHCalibrationNode()
        
        try:
            # Wait for sensor connection
            print(f"\n  Conectando con sensor...")
            if not self.node.wait_for_connection(timeout=15):
                print(f"\n  {Colors.RED}✗ No se detectó el sensor.{Colors.RESET}")
                print(f"  Verifica:")
                print(f"    • ESP32 está encendido y conectado a WiFi")
                print(f"    • micro-ROS Agent está ejecutándose:")
                print(f"      {Colors.DIM}micro_ros_agent udp4 --port 8888{Colors.RESET}")
                print(f"    • Topic /biofloc/sensor_data existe:")
                print(f"      {Colors.DIM}ros2 topic list{Colors.RESET}")
                return
            
            print(f"  {Colors.GREEN}✓ Sensor conectado{Colors.RESET}")
            
            # Show menu
            option = self.print_menu()
            
            success = False
            
            if option == '1':
                success = self.run_1point_calibration()
            elif option == '2':
                success = self.run_2point_calibration()
            elif option == '3':
                success = self.run_3point_calibration()
            elif option == 'V':
                self.run_verification()
                return
            elif option == 'Q':
                print(f"\n  {Colors.DIM}Saliendo...{Colors.RESET}")
                return
            else:
                print(f"\n  {Colors.RED}✗ Opción inválida{Colors.RESET}")
                return
            
            if not success:
                print(f"\n  {Colors.RED}✗ Calibración incompleta{Colors.RESET}")
                return
            
            # Calculate and display results
            slope, offset, stats = self.calculate_calibration()
            self.print_results(slope, offset, stats)
            self.print_code_output(slope, offset)
            self.save_results(slope, offset, stats)
            
            print(f"\n{Colors.GREEN}{Colors.BOLD}{'═' * 70}")
            print(f"  ✓ CALIBRACIÓN COMPLETADA EXITOSAMENTE")
            print(f"{'═' * 70}{Colors.RESET}")
            
            print(f"\n  Próximos pasos:")
            print(f"    1. Editar main/main.c con el código generado")
            print(f"    2. Recompilar: idf.py build")
            print(f"    3. Flashear: idf.py flash")
            print(f"    4. Verificar con solución de pH conocido\n")
            
        except KeyboardInterrupt:
            print(f"\n\n  {Colors.YELLOW}✗ Calibración cancelada{Colors.RESET}")
        
        finally:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Biofloc pH Sensor Calibration Tool v3.0',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 calibrate_ph.py              # Interactive mode (precision)
  python3 calibrate_ph.py --quick      # Quick mode (~1 min/point)
  python3 calibrate_ph.py --verify     # Just read current values
        """
    )
    
    parser.add_argument('--quick', '-q', action='store_true',
                       help='Quick calibration mode (shorter wait times)')
    parser.add_argument('--verify', '-v', action='store_true',
                       help='Verification mode (read values without calibrating)')
    
    args = parser.parse_args()
    
    calibrator = PHCalibrator(quick_mode=args.quick)
    
    if args.verify:
        rclpy.init()
        calibrator.node = PHCalibrationNode()
        try:
            print(f"\n{Colors.BOLD}  MODO VERIFICACIÓN{Colors.RESET}")
            print(f"  Conectando...")
            if calibrator.node.wait_for_connection(timeout=10):
                calibrator.run_verification()
            else:
                print(f"  {Colors.RED}✗ No se pudo conectar con el sensor{Colors.RESET}")
        except (KeyboardInterrupt, SystemExit):
            print(f"\n  {Colors.GREEN}✓ Verificación terminada{Colors.RESET}")
        except Exception:
            pass  # Ignore shutdown errors
        finally:
            try:
                calibrator.node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
    else:
        calibrator.run()


if __name__ == '__main__':
    main()
