#!/usr/bin/env python3
"""
Biofloc Sensor DB Bridge — Puente ROS 2 a MongoDB.

Suscribe a /biofloc/sensor_data (JSON) y almacena en MongoDB Atlas.

Uso:
    # 1. Configurar credenciales
    cp scripts/.env.example scripts/.env
    nano scripts/.env
    
    # 2. Ejecutar (con ROS 2 y Agent corriendo)
    cd /home/Biofloc-Firmware-ROS/scripts
    source /opt/ros/jazzy/setup.bash
    source ~/microros_ws/install/local_setup.bash
    python3 sensor_db_bridge.py

Dependencias:
    pip install pymongo python-dotenv

Autor: @Marton1123
Versión: 2.3.0 (Corregido para JSON String)
"""

import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def load_environment() -> None:
    """Load environment variables from .env file."""
    try:
        from dotenv import load_dotenv
        
        script_dir = Path(__file__).parent
        env_file = script_dir / '.env'
        
        if env_file.exists():
            load_dotenv(env_file)
            print(f"✓ Loaded environment from: {env_file}")
        else:
            env_file = script_dir.parent / '.env'
            if env_file.exists():
                load_dotenv(env_file)
                print(f"✓ Loaded environment from: {env_file}")
            else:
                print("⚠ No .env file found, using environment variables")
    except ImportError:
        print("WARNING: python-dotenv not installed. Run: pip install python-dotenv")


# Load environment before importing pymongo
load_environment()

try:
    from pymongo import MongoClient
    from pymongo.errors import ConnectionFailure, ServerSelectionTimeoutError
    PYMONGO_AVAILABLE = True
except ImportError:
    PYMONGO_AVAILABLE = False
    print("WARNING: pymongo not installed. Run: pip install pymongo")


class SensorDBBridge(Node):
    """
    ROS 2 node that receives sensor data and stores it in MongoDB.
    """
    
    def __init__(self):
        super().__init__('sensor_db_bridge')
        
        # Load configuration from environment
        self.mongodb_uri = os.getenv('MONGODB_URI', 'mongodb://localhost:27017/')
        self.db_name = os.getenv('MONGODB_DATABASE', 'SistemasLab')
        self.collection_name = os.getenv('MONGODB_COLLECTION', 'telemetria')
        self.topic_name = os.getenv('ROS_TOPIC', '/biofloc/sensor_data')
        self.log_data = os.getenv('LOG_DATA', 'true').lower() == 'true'
        
        # MongoDB connection
        self.mongo_client: Optional[MongoClient] = None
        self.db = None
        self.collection = None
        self.mongodb_connected = False
        
        # Statistics
        self.messages_received = 0
        self.messages_saved = 0
        self.messages_failed = 0
        
        # Connect to MongoDB
        if PYMONGO_AVAILABLE:
            self._connect_mongodb()
        else:
            self.get_logger().error("pymongo not available - data will only be logged")
        
        # Subscribe to sensor topic (JSON String)
        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.sensor_callback,
            10
        )
        
        # Status timer (every 60 seconds)
        self.status_timer = self.create_timer(60.0, self._log_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Sensor DB Bridge Started")
        self.get_logger().info(f"  Topic: {self.topic_name}")
        self.get_logger().info(f"  Database: {self.db_name}.{self.collection_name}")
        self.get_logger().info(f"  MongoDB Connected: {self.mongodb_connected}")
        self.get_logger().info("=" * 60)
    
    def _connect_mongodb(self) -> bool:
        """Establish connection to MongoDB."""
        try:
            self.mongo_client = MongoClient(
                self.mongodb_uri,
                serverSelectionTimeoutMS=5000
            )
            
            # Test connection
            self.mongo_client.admin.command('ping')
            
            self.db = self.mongo_client[self.db_name]
            self.collection = self.db[self.collection_name]
            
            self.mongodb_connected = True
            self.get_logger().info(f"✓ Connected to MongoDB")
            return True
            
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            self.get_logger().error(f"✗ Failed to connect to MongoDB: {e}")
            self.mongodb_connected = False
            return False
        except Exception as e:
            self.get_logger().error(f"✗ MongoDB error: {e}")
            self.mongodb_connected = False
            return False
    
    def sensor_callback(self, msg: String):
        """Process incoming JSON sensor messages."""
        self.messages_received += 1
        
        try:
            # Parse JSON from String message
            data = json.loads(msg.data)
            
            # Extract sensor values
            sensors = data.get('sensors', {})
            ph_data = sensors.get('ph', {})
            temp_data = sensors.get('temperature', {})
            
            temperature = temp_data.get('value', 0)
            ph = ph_data.get('value', 0)
            
            # Validate sensor ranges
            temp_valid = -20 <= temperature <= 80
            ph_valid = 0 <= ph <= 14
            
            # Create document (preserve original JSON structure from ESP32)
            doc = {
                'device_id': data.get('device_id', 'biofloc_esp32'),
                'location': data.get('location', 'lab'),
                'timestamp': data.get('timestamp', datetime.now().isoformat()),
                'sensors': {
                    'temperature': {
                        'value': round(temperature, 2),
                        'unit': temp_data.get('unit', 'C'),
                        'valid': temp_valid,
                        'voltage': temp_data.get('voltage', 0)
                    },
                    'ph': {
                        'value': round(ph, 2),
                        'unit': ph_data.get('unit', 'pH'),
                        'valid': ph_valid,
                        'voltage': ph_data.get('voltage', 0)
                    }
                },
                '_ros_topic': self.topic_name,
            }
            
            # Log sensor data
            if self.log_data:
                self._log_sensor_data(doc)
            
            # Save to MongoDB
            if self.mongodb_connected and self.collection is not None:
                try:
                    result = self.collection.insert_one(doc)
                    if result.inserted_id:
                        self.messages_saved += 1
                        self.get_logger().debug(f"Saved document: {result.inserted_id}")
                    else:
                        self.messages_failed += 1
                        self.get_logger().warning("Failed to save document")
                except Exception as e:
                    self.get_logger().error(f"MongoDB insert error: {e}")
                    self.messages_failed += 1
                    
                    # Try to reconnect
                    if not self.mongodb_connected and PYMONGO_AVAILABLE:
                        self.get_logger().warning("Attempting MongoDB reconnection...")
                        self._connect_mongodb()
            else:
                # Not connected to MongoDB
                if PYMONGO_AVAILABLE and not self.mongodb_connected:
                    self.get_logger().debug("MongoDB disconnected, attempting reconnection...")
                    self._connect_mongodb()
                    
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parse error: {e}")
            self.messages_failed += 1
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")
            self.messages_failed += 1
    
    def _log_sensor_data(self, data: dict) -> None:
        """Display sensor data in the log."""
        device = data.get('device_id', 'unknown')
        location = data.get('location', 'unknown')
        timestamp = data.get('timestamp', 'unknown')
        
        sensors = data.get('sensors', {})
        ph_data = sensors.get('ph', {})
        temp_data = sensors.get('temperature', {})
        
        ph_val = ph_data.get('value', 'N/A')
        ph_valid = ph_data.get('valid', False)
        temp_val = temp_data.get('value', 'N/A')
        temp_valid = temp_data.get('valid', False)
        
        status_ph = "✓" if ph_valid else "✗"
        status_temp = "✓" if temp_valid else "✗"
        
        self.get_logger().info(
            f"[{device}@{location}] "
            f"pH: {ph_val} {status_ph} | "
            f"Temp: {temp_val}°C {status_temp} | "
            f"{timestamp}"
        )
    
    def _log_status(self) -> None:
        """Log periodic statistics."""
        success_rate = 0.0
        if self.messages_received > 0:
            success_rate = (self.messages_saved / self.messages_received) * 100
        
        self.get_logger().info(
            f"Stats: received={self.messages_received}, "
            f"saved={self.messages_saved}, "
            f"failed={self.messages_failed}, "
            f"success_rate={success_rate:.1f}%, "
            f"mongodb={'connected' if self.mongodb_connected else 'disconnected'}"
        )
    
    def destroy_node(self):
        """Cleanup on node shutdown."""
        if self.mongo_client:
            self.mongo_client.close()
            self.get_logger().info("MongoDB connection closed")
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    if not PYMONGO_AVAILABLE:
        print("ERROR: pymongo is required. Install with: pip install pymongo")
        sys.exit(1)
    
    rclpy.init(args=args)
    
    node = SensorDBBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
