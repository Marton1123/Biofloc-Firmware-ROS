#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""
pH Sensor Diagnostic Tool

Analyzes current pH sensor readings to diagnose calibration issues.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys
from collections import deque

class PHDiagnostic(Node):
    def __init__(self):
        super().__init__('ph_diagnostic')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.readings = deque(maxlen=20)
        self.count = 0
        
        self.get_logger().info("═" * 70)
        self.get_logger().info("pH SENSOR DIAGNOSTIC TOOL")
        self.get_logger().info("═" * 70)
        self.get_logger().info("Collecting 20 readings...")
    
    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            ph_sensor = data.get('sensors', {}).get('ph', {})
            
            reading = {
                'voltage': ph_sensor.get('voltage', 0),
                'voltage_adc': ph_sensor.get('voltage', 0) / 3.0,  # Approx
                'ph': ph_sensor.get('value', 0),
                'valid': ph_sensor.get('valid', False),
                'timestamp': data.get('timestamp', '')
            }
            
            self.readings.append(reading)
            self.count += 1
            
            if self.count <= 20:
                self.get_logger().info(f"[{self.count}/20] pH: {reading['ph']:.2f}, V: {reading['voltage']:.3f}V")
            
            if self.count == 20:
                self.analyze_and_report()
                rclpy.shutdown()
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f"Failed to parse message: {e}")
    
    def analyze_and_report(self):
        if not self.readings:
            self.get_logger().error("No readings collected!")
            return
        
        # Calculate statistics
        voltages = [r['voltage'] for r in self.readings]
        ph_values = [r['ph'] for r in self.readings]
        
        avg_voltage = sum(voltages) / len(voltages)
        avg_ph = sum(ph_values) / len(ph_values)
        
        min_voltage = min(voltages)
        max_voltage = max(voltages)
        min_ph = min(ph_values)
        max_ph = max(ph_values)
        
        std_voltage = (sum((v - avg_voltage) ** 2 for v in voltages) / len(voltages)) ** 0.5
        std_ph = (sum((p - avg_ph) ** 2 for p in ph_values) / len(ph_values)) ** 0.5
        
        # Diagnosis
        print("\n" + "═" * 70)
        print("DIAGNOSTIC REPORT")
        print("═" * 70)
        
        print(f"\n📊 Statistics (n={len(self.readings)}):")
        print(f"  Voltage: {avg_voltage:.3f}V ± {std_voltage:.3f}V (range: {min_voltage:.3f}-{max_voltage:.3f}V)")
        print(f"  pH:      {avg_ph:.2f} ± {std_ph:.2f} (range: {min_ph:.2f}-{max_ph:.2f})")
        
        print(f"\n🔍 Analysis:")
        
        # Check 1: Voltage out of range
        if avg_voltage > 5.0:
            print(f"  ⚠️  PROBLEM: Voltage {avg_voltage:.3f}V exceeds sensor range (0-5V)")
            print(f"     → Sensor should output 0-5V for pH 0-14")
            print(f"     → Currently reading {avg_voltage:.3f}V (after voltage divider correction)")
            print(f"     → This suggests:")
            print(f"        • Sensor disconnected (open circuit)")
            print(f"        • Voltage divider miscalculated")
            print(f"        • Sensor damaged/saturated")
        elif avg_voltage < 0.1:
            print(f"  ⚠️  PROBLEM: Voltage {avg_voltage:.3f}V too low")
            print(f"     → Sensor may be shorted to ground")
            print(f"     → Check wiring")
        else:
            print(f"  ✓ Voltage in valid range: {avg_voltage:.3f}V")
        
        # Check 2: pH out of range
        if avg_ph > 14.0:
            print(f"  ⚠️  PROBLEM: pH {avg_ph:.2f} exceeds valid range (0-14)")
            print(f"     → This confirms voltage is too high")
            print(f"     → Formula: pH = V * 2.8")
            print(f"     → {avg_voltage:.3f}V * 2.8 = {avg_ph:.2f} pH")
        elif avg_ph < 0.0:
            print(f"  ⚠️  PROBLEM: pH {avg_ph:.2f} is negative")
            print(f"     → Voltage too low or calibration error")
        else:
            print(f"  ✓ pH in valid range: {avg_ph:.2f}")
        
        # Check 3: Stability
        if std_voltage > 0.1:
            print(f"  ⚠️  WARNING: High voltage variation (σ={std_voltage:.3f}V)")
            print(f"     → Readings unstable")
            print(f"     → Check connections and sensor placement")
        else:
            print(f"  ✓ Stable readings (σ={std_voltage:.4f}V)")
        
        # Recommendations
        print(f"\n💡 Recommendations:")
        
        if avg_voltage > 5.0:
            print(f"\n  1️⃣  VERIFY HARDWARE:")
            print(f"     • Check sensor is connected to ESP32 GPIO{36}")
            print(f"     • Verify voltage divider: R1=20kΩ, R2=10kΩ")
            print(f"     • Measure with multimeter:")
            print(f"       - At sensor output: Should be 0-5V")
            print(f"       - At ESP32 GPIO: Should be 0-1.67V")
            
            print(f"\n  2️⃣  TEST SENSOR:")
            print(f"     • Disconnect sensor from ESP32")
            print(f"     • Measure sensor output with multimeter")
            print(f"     • Place in pH 7 buffer, should read ~2.5V")
            print(f"     • If reading > 5V, sensor is damaged")
            
            print(f"\n  3️⃣  VERIFY ADC READING:")
            adc_voltage = avg_voltage / 3.0
            adc_raw = int((adc_voltage / 3.3) * 4095)
            print(f"     • Expected ADC voltage: {adc_voltage:.3f}V (GPIO input)")
            print(f"     • Expected ADC raw: ~{adc_raw} (0-4095 scale)")
            print(f"     • Check ESP32 logs for actual ADC values")
            
            print(f"\n  4️⃣  IF SENSOR IS DISCONNECTED:")
            print(f"     • The ADC reads floating/pull-up voltage")
            print(f"     • This explains readings > 5V")
            print(f"     • Connect sensor and retest")
        
        else:
            print(f"\n  1️⃣  CALIBRATION NEEDED:")
            print(f"     • Current readings appear valid but uncalibrated")
            print(f"     • Run calibration tool:")
            print(f"       python3 scripts/calibrate_ph.py")
            
            print(f"\n  2️⃣  EXPECTED VOLTAGES:")
            print(f"     • pH 0:  0.00V")
            print(f"     • pH 4:  1.43V")
            print(f"     • pH 7:  2.50V")
            print(f"     • pH 10: 3.57V")
            print(f"     • pH 14: 5.00V")
            
            # Calculate what pH the current voltage represents if calibrated correctly
            expected_ph = avg_voltage * 2.8
            print(f"\n  3️⃣  INTERPRETATION:")
            print(f"     • Current voltage: {avg_voltage:.3f}V")
            print(f"     • If correctly calibrated, this would be: pH {expected_ph:.2f}")
            print(f"     • Actual reading: pH {avg_ph:.2f}")
        
        print("\n" + "═" * 70)
        print("For detailed calibration guide, see: docs/CALIBRATION.md")
        print("═" * 70 + "\n")


def main():
    rclpy.init()
    diagnostic = PHDiagnostic()
    
    try:
        rclpy.spin(diagnostic)
    except KeyboardInterrupt:
        print("\n\nDiagnostic interrupted")
    finally:
        diagnostic.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
