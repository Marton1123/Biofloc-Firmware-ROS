#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""
pH Sensor Calibration Tool

Interactive tool to calibrate pH sensor using standard buffer solutions.
Monitors ROS topic for voltage readings and calculates calibration parameters.

Usage:
    1. Prepare pH buffer solutions (pH 4.0 and pH 10.0 recommended)
    2. Start micro-ROS Agent: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    3. Ensure ESP32 is running and publishing sensor data
    4. Run this script: python3 calibrate_ph.py
    5. Follow interactive prompts
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
from collections import deque
import time

class PHCalibrator(Node):
    def __init__(self):
        super().__init__('ph_calibrator')
        
        self.subscription = self.create_subscription(
            String,
            '/biofloc/sensor_data',
            self.sensor_callback,
            10
        )
        
        self.voltage_readings = deque(maxlen=10)  # Keep last 10 readings
        self.last_update = time.time()
        
        self.get_logger().info("═" * 70)
        self.get_logger().info("pH SENSOR CALIBRATION TOOL")
        self.get_logger().info("═" * 70)
        self.get_logger().info("Waiting for sensor data from /biofloc/sensor_data...")
    
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
        """Wait for stable voltage reading"""
        self.get_logger().info("Waiting for stable readings (may take up to 30s)...")
        
        start_time = time.time()
        stable_count = 0
        required_stable = 5
        
        while (time.time() - start_time) < timeout:
            if len(self.voltage_readings) >= 5:
                recent = list(self.voltage_readings)[-5:]
                avg = sum(recent) / len(recent)
                std_dev = (sum((x - avg) ** 2 for x in recent) / len(recent)) ** 0.5
                
                # Check if stable (std dev < 0.05V)
                if std_dev < 0.05:
                    stable_count += 1
                    if stable_count >= required_stable:
                        self.get_logger().info(f"✓ Stable voltage: {avg:.3f}V (σ={std_dev:.4f}V)")
                        return avg
                else:
                    stable_count = 0
                
                # Show current reading
                if int(time.time()) % 2 == 0:
                    self.get_logger().info(f"  Current: {avg:.3f}V (σ={std_dev:.3f}V)")
            
            rclpy.spin_once(self, timeout_sec=0.5)
            
            # Check if no new data
            if (time.time() - self.last_update) > 10:
                self.get_logger().error("✗ No sensor data received. Check ESP32 and Agent.")
                return None
        
        self.get_logger().warn("⚠ Timeout waiting for stable reading")
        if self.voltage_readings:
            avg = sum(self.voltage_readings) / len(self.voltage_readings)
            self.get_logger().warn(f"  Using average: {avg:.3f}V")
            return avg
        
        return None


def print_header():
    print("\n" + "═" * 70)
    print("  pH SENSOR 2-POINT CALIBRATION")
    print("═" * 70)
    print("\nYou will need:")
    print("  • Two pH buffer solutions (e.g., pH 4.0 and pH 10.0)")
    print("  • Clean water for rinsing sensor between buffers")
    print("  • Paper towel to dry sensor")
    print("\nCalibration Process:")
    print("  1. Rinse sensor with clean water")
    print("  2. Place sensor in LOW pH buffer (e.g., pH 4.0)")
    print("  3. Wait for voltage to stabilize")
    print("  4. Rinse sensor again")
    print("  5. Place sensor in HIGH pH buffer (e.g., pH 10.0)")
    print("  6. Wait for voltage to stabilize")
    print("  7. Calculate and apply calibration")
    print("\n" + "─" * 70)


def get_float_input(prompt, min_val=None, max_val=None):
    """Get validated float input from user"""
    while True:
        try:
            value = float(input(prompt))
            if min_val is not None and value < min_val:
                print(f"✗ Value must be >= {min_val}")
                continue
            if max_val is not None and value > max_val:
                print(f"✗ Value must be <= {max_val}")
                continue
            return value
        except ValueError:
            print("✗ Invalid number, try again")
        except KeyboardInterrupt:
            print("\n\n✗ Calibration cancelled")
            sys.exit(0)


def main():
    print_header()
    
    # Initialize ROS
    rclpy.init()
    calibrator = PHCalibrator()
    
    try:
        # Point 1: Low pH buffer
        print("\n" + "▶" * 35)
        print("STEP 1: LOW pH BUFFER")
        print("▶" * 35)
        input("\nRinse sensor and place in LOW pH buffer solution, then press ENTER...")
        
        ph_low = get_float_input("Enter pH value of LOW buffer (e.g., 4.0): ", 0.0, 14.0)
        voltage_low = calibrator.get_stable_voltage()
        
        if voltage_low is None:
            calibrator.get_logger().error("✗ Failed to read voltage for low buffer")
            return
        
        print(f"\n✓ Low pH calibration point recorded:")
        print(f"    pH {ph_low:.2f} → {voltage_low:.3f}V")
        
        # Point 2: High pH buffer
        print("\n" + "▶" * 35)
        print("STEP 2: HIGH pH BUFFER")
        print("▶" * 35)
        input("\nRinse sensor and place in HIGH pH buffer solution, then press ENTER...")
        
        ph_high = get_float_input("Enter pH value of HIGH buffer (e.g., 10.0): ", 0.0, 14.0)
        
        if ph_high <= ph_low:
            calibrator.get_logger().error(f"✗ High pH ({ph_high}) must be > Low pH ({ph_low})")
            return
        
        voltage_high = calibrator.get_stable_voltage()
        
        if voltage_high is None:
            calibrator.get_logger().error("✗ Failed to read voltage for high buffer")
            return
        
        if voltage_high <= voltage_low:
            calibrator.get_logger().error(f"✗ High voltage ({voltage_high:.3f}V) must be > Low voltage ({voltage_low:.3f}V)")
            calibrator.get_logger().error("   Check sensor connection and buffer solutions")
            return
        
        print(f"\n✓ High pH calibration point recorded:")
        print(f"    pH {ph_high:.2f} → {voltage_high:.3f}V")
        
        # Calculate calibration
        print("\n" + "═" * 70)
        print("CALIBRATION RESULTS")
        print("═" * 70)
        
        slope = (ph_high - ph_low) / (voltage_high - voltage_low)
        offset = ph_low - (slope * voltage_low)
        
        print(f"\n✓ Calibration Parameters:")
        print(f"    Slope:  {slope:.4f}")
        print(f"    Offset: {offset:.4f}")
        print(f"\n  Formula: pH = {slope:.4f} * V_sensor + {offset:.4f}")
        
        print(f"\n✓ Verification:")
        print(f"    At {voltage_low:.3f}V → pH {ph_low:.2f} (expected) vs {slope * voltage_low + offset:.2f} (calculated)")
        print(f"    At {voltage_high:.3f}V → pH {ph_high:.2f} (expected) vs {slope * voltage_high + offset:.2f} (calculated)")
        
        # Provide code for manual entry
        print("\n" + "─" * 70)
        print("TO APPLY CALIBRATION:")
        print("─" * 70)
        print("\nOption 1: Add to firmware (recommended for permanent calibration)")
        print("  Edit main/sensors.c and add after sensors_init():")
        print(f"\n    sensors_calibrate_ph_manual({slope:.4f}f, {offset:.4f}f);")
        
        print("\nOption 2: Apply via console command (temporary)")
        print("  Connect to ESP32 serial console and enter:")
        print(f"\n    calibrate_ph {slope:.4f} {offset:.4f}")
        
        print("\nOption 3: Store in NVS (persistent across reboots)")
        print("  Requires adding NVS storage code to firmware")
        
        print("\n" + "═" * 70)
        print("✓ Calibration complete!")
        print("═" * 70)
        
    except KeyboardInterrupt:
        print("\n\n✗ Calibration interrupted")
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
