#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""
Voltage Divider Diagnostic and Calibration Tool

Helps determine the correct voltage divider factor by comparing
physical measurements with software readings.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class VoltageDividerCalibrator(Node):
    def __init__(self):
        super().__init__('voltage_divider_calibrator')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.readings = []
        self.count = 0
        
        self.get_logger().info("═" * 70)
        self.get_logger().info("VOLTAGE DIVIDER DIAGNOSTIC TOOL")
        self.get_logger().info("═" * 70)
        self.get_logger().info("Collecting 10 readings...")
    
    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            ph_sensor = data.get('sensors', {}).get('ph', {})
            
            reading = {
                'voltage_reported': ph_sensor.get('voltage', 0),  # After divider correction
                'ph': ph_sensor.get('value', 0)
            }
            
            self.readings.append(reading)
            self.count += 1
            
            if self.count <= 10:
                self.get_logger().info(
                    f"[{self.count}/10] Reported voltage: {reading['voltage_reported']:.3f}V, "
                    f"pH: {reading['ph']:.2f}"
                )
            
            if self.count == 10:
                self.analyze()
                rclpy.shutdown()
                
        except (json.JSONDecodeError, KeyError) as e:
            pass
    
    def analyze(self):
        if not self.readings:
            return
        
        avg_v_reported = sum(r['voltage_reported'] for r in self.readings) / len(self.readings)
        avg_ph = sum(r['ph'] for r in self.readings) / len(self.readings)
        
        print("\n" + "═" * 70)
        print("DIAGNOSTIC RESULTS")
        print("═" * 70)
        
        print(f"\n📊 Current Software Readings:")
        print(f"  Voltage (after divider correction): {avg_v_reported:.3f}V")
        print(f"  pH (calculated): {avg_ph:.2f}")
        
        print("\n" + "─" * 70)
        print("VOLTAGE DIVIDER ANALYSIS")
        print("─" * 70)
        
        # Get physical measurement from user
        print("\n🔧 You reported measuring ~1.7V physically at the ESP32 pin.")
        print("   This is the ACTUAL voltage at the GPIO (after voltage divider).")
        
        try:
            v_measured = float(input("\n📏 Enter exact voltage measured at ESP32 GPIO pin (V): "))
        except (ValueError, KeyboardInterrupt):
            print("\nUsing reported value of 1.7V")
            v_measured = 1.7
        
        # Calculate current divider factor being used
        current_factor = avg_v_reported / v_measured if v_measured > 0 else 3.0
        
        print(f"\n📐 Calculation:")
        print(f"  Measured at GPIO pin: {v_measured:.3f}V")
        print(f"  Software reports: {avg_v_reported:.3f}V")
        print(f"  Current divider factor: {current_factor:.2f}")
        print(f"    (Software calculates: V_sensor = V_gpio × {current_factor:.2f})")
        
        # Determine actual sensor voltage
        print("\n" + "─" * 70)
        print("SENSOR VOLTAGE DETERMINATION")
        print("─" * 70)
        
        print("\n❓ What is the ACTUAL sensor voltage (before any divider)?")
        print("   Option 1: No voltage divider exists → Sensor voltage = GPIO voltage")
        print("   Option 2: Voltage divider exists → Sensor voltage = GPIO × new_factor")
        print("   Option 3: I know the actual pH of the water")
        
        choice = input("\nChoose option (1/2/3) [default: 3]: ").strip() or "3"
        
        if choice == "1":
            # No divider
            v_sensor_actual = v_measured
            new_factor = 1.0
            print(f"\n✓ No voltage divider: V_sensor = {v_sensor_actual:.3f}V")
            
        elif choice == "2":
            # Known divider
            try:
                new_factor = float(input("Enter new voltage divider factor: "))
                v_sensor_actual = v_measured * new_factor
                print(f"\n✓ With divider factor {new_factor:.2f}: V_sensor = {v_sensor_actual:.3f}V")
            except ValueError:
                print("Invalid input, using factor 1.0")
                new_factor = 1.0
                v_sensor_actual = v_measured
        
        else:  # choice == "3"
            # Known pH
            try:
                ph_actual = float(input("\n📏 Enter actual pH of the water (e.g., 7.0): "))
                
                # Calculate required sensor voltage for that pH using datasheet formula
                v_sensor_actual = ph_actual / 2.8
                
                # Calculate what divider factor would give us that voltage
                new_factor = v_sensor_actual / v_measured if v_measured > 0 else 1.0
                
                print(f"\n📐 Calculation:")
                print(f"  Actual pH: {ph_actual:.2f}")
                print(f"  Required sensor voltage: {v_sensor_actual:.3f}V (pH / 2.8)")
                print(f"  Measured GPIO voltage: {v_measured:.3f}V")
                print(f"  Required divider factor: {new_factor:.4f}")
                
            except ValueError:
                print("Invalid input, using factor 1.0")
                new_factor = 1.0
                v_sensor_actual = v_measured
        
        # Summary and recommendations
        print("\n" + "═" * 70)
        print("CALIBRATION RECOMMENDATIONS")
        print("═" * 70)
        
        print(f"\n✓ Measurements:")
        print(f"  GPIO voltage (measured): {v_measured:.3f}V")
        print(f"  Sensor voltage (actual): {v_sensor_actual:.3f}V")
        print(f"  Required divider factor: {new_factor:.4f}")
        
        print(f"\n✓ Current configuration:")
        print(f"  Using factor: {current_factor:.2f}")
        print(f"  Gives pH: {avg_ph:.2f}")
        
        if choice == "3":
            print(f"  Expected pH: {ph_actual:.2f}")
            print(f"  Error: {abs(avg_ph - ph_actual):.2f} pH units")
        
        print(f"\n✓ Correction needed:")
        if abs(new_factor - 1.0) < 0.01:
            print(f"  ⚠️  NO VOLTAGE DIVIDER detected or needed")
            print(f"     Sensor connects directly to GPIO")
        else:
            print(f"  🔧 Voltage divider factor should be: {new_factor:.4f}")
        
        # Provide code changes
        print("\n" + "─" * 70)
        print("HOW TO APPLY CORRECTION")
        print("─" * 70)
        
        print("\n📝 Option 1: Update Kconfig (recommended)")
        print(f"   Run: idf.py menuconfig")
        print(f"   Navigate to: Biofloc Configuration → pH Sensor")
        print(f"   Set 'Voltage Divider Factor' to: {int(new_factor * 1000)}")
        print(f"   (Value in menuconfig is × 1000, so {new_factor:.4f} → {int(new_factor * 1000)})")
        
        print("\n📝 Option 2: Edit sdkconfig directly")
        print(f"   Edit: sdkconfig")
        print(f"   Find: CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR")
        print(f"   Change to: {int(new_factor * 1000)}")
        
        print("\n📝 Option 3: Edit sensors.c (for testing)")
        print(f"   Edit: main/sensors.c")
        print(f"   Find line: #define PH_DIVIDER_FACTOR 3.0f")
        print(f"   Change to: #define PH_DIVIDER_FACTOR {new_factor:.4f}f")
        
        print("\n Then rebuild and flash:")
        print("   idf.py build flash")
        
        # Verification
        expected_ph_after = v_sensor_actual * 2.8
        print(f"\n📊 After correction:")
        print(f"  GPIO voltage: {v_measured:.3f}V (measured)")
        print(f"  × factor {new_factor:.4f}")
        print(f"  = Sensor voltage: {v_sensor_actual:.3f}V")
        print(f"  × 2.8")
        print(f"  = pH: {expected_ph_after:.2f}")
        
        if choice == "3":
            print(f"\n  ✓ This matches the expected pH of {ph_actual:.2f}")
        
        print("\n" + "═" * 70)


def main():
    print("\n" + "═" * 70)
    print("VOLTAGE DIVIDER DIAGNOSTIC TOOL")
    print("═" * 70)
    print("\nThis tool helps determine the correct voltage divider factor")
    print("by comparing physical measurements with software readings.")
    print("\nYou mentioned:")
    print("  • GPIO pin voltage: ~1.7V (physical measurement)")
    print("  • Software reports: ~5V (after divider correction)")
    print("  • Sensor is in water (not air)")
    print("\nWe'll collect readings and guide you through correction.")
    print("═" * 70 + "\n")
    
    rclpy.init()
    calibrator = VoltageDividerCalibrator()
    
    try:
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
