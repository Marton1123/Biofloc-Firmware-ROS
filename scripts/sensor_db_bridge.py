#!/usr/bin/env python3
"""
Biofloc Sensor DB Bridge - ROS 2 to MongoDB Bridge (Secure Gateway Architecture)

Subscribes to /biofloc/sensor_data (JSON) and stores in MongoDB Atlas.
Operates in a secure IoT gateway where ESP32 devices have no Internet access.
This node adds server-side timestamps to compensate for ESP32's lack of NTP.

Architecture:
    - Gateway (Raspberry Pi 4): WiFi Hotspot (10.42.0.1) without Internet
    - ESP32: Connects to hotspot, publishes to micro-ROS Agent
    - Bridge (this node): Adds timestamps, routes data to separate collections
    
Three-collection architecture (v3.2):
    - telemetria: time-series sensor readings (biological data)
    - devices: device metadata, state, and connection history
    - system_health: ESP32 telemetry (free_heap, uptime, reset_reason)

Usage:
    # 1. Configure credentials
    cp scripts/.env.example scripts/.env
    nano scripts/.env
    
    # 2. Run migration (first time only)
    python3 scripts/migrate_to_devices_collection.py
    
    # 3. Execute bridge (with ROS 2 and Agent running)
    cd /home/Biofloc-Firmware-ROS/scripts
    source /opt/ros/jazzy/setup.bash
    source ~/microros_ws/install/local_setup.bash
    python3 sensor_db_bridge.py

Dependencies:
    pip install pymongo python-dotenv

Author: @Marton1123
Version: 3.2.0 (Resilient PyMongo Pool & Atomic Upserts)
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
    
    Three-collection architecture (v3.2):
        - telemetria: sensor readings (biological data, time-series)
        - devices: device metadata, state, connection history
        - system_health: ESP32 telemetry (free_heap, uptime, reset_reason)
    """
    
    def __init__(self):
        super().__init__('sensor_db_bridge')
        
        # Load configuration from environment
        self.mongodb_uri = os.getenv('MONGODB_URI', 'mongodb://localhost:27017/')
        self.db_name = os.getenv('MONGODB_DATABASE', 'SistemasLab')
        self.collection_name = os.getenv('MONGODB_COLLECTION', 'telemetria')
        self.devices_collection_name = os.getenv('MONGODB_COLLECTION_DEVICES', 'devices')
        self.system_health_collection_name = 'system_health'  # New collection for telemetry
        self.topic_name = os.getenv('ROS_TOPIC', '/biofloc/sensor_data')
        self.log_data = os.getenv('LOG_DATA', 'true').lower() == 'true'
        
        # MongoDB connection
        self.mongo_client: Optional[MongoClient] = None
        self.db = None
        self.telemetria_collection = None
        self.devices_collection = None
        self.system_health_collection = None  # New collection
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
        
        # Subscribe to sensor topic (JSON String) con QoS BEST_EFFORT para compatibilidad con ESP32
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=500
        )
        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.sensor_callback,
            qos_profile
        )
        
        # Status timer (every 60 seconds)
        self.status_timer = self.create_timer(60.0, self._log_status)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Sensor DB Bridge v3.2 Started")
        self.get_logger().info(f"  Topic: {self.topic_name}")
        self.get_logger().info(f"  Database: {self.db_name}")
        self.get_logger().info(f"    - telemetria: {self.collection_name}")
        self.get_logger().info(f"    - devices: {self.devices_collection_name}")
        self.get_logger().info(f"    - system_health: {self.system_health_collection_name}")
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
            self.telemetria_collection = self.db[self.collection_name]
            self.devices_collection = self.db[self.devices_collection_name]
            self.system_health_collection = self.db[self.system_health_collection_name]  # New collection
            
            self.mongodb_connected = True
            self.get_logger().info(f"Connected to MongoDB: {self.db_name}")
            self.get_logger().info(f"  Collections: telemetria, devices, system_health")
            return True
            
        except (ConnectionFailure, ServerSelectionTimeoutError) as e:
            self.get_logger().error(f"Failed to connect to MongoDB: {e}")
            self.mongodb_connected = False
            return False
        except Exception as e:
            self.get_logger().error(f"MongoDB error: {e}")
            self.mongodb_connected = False
            return False
    
    def sensor_callback(self, msg: String):
        """Process incoming JSON sensor messages with telemetry routing."""
        self.messages_received += 1
        
        try:
            # Parse JSON from String message
            data = json.loads(msg.data)
            
            # SERVER-SIDE TIMESTAMP (ESP32 has no Internet/NTP)
            server_timestamp = datetime.now().isoformat()
            
            # Extract device information
            device_id = data.get('device_id', 'biofloc_esp32')
            location = data.get('location', 'lab')
            esp32_sample_id = data.get('timestamp', 'unknown')  # Sample counter from ESP32
            
            # === TELEMETRY ROUTING (v3.2 architecture) ===
            # Extract and route system telemetry to separate collection
            system_data = data.pop('system', None)  # Extract and remove from data
            
            if system_data and self.mongodb_connected and self.system_health_collection is not None:
                try:
                    # Create system_health document
                    system_health_doc = {
                        'device_id': device_id,
                        'timestamp_gw': server_timestamp,  # Gateway (server) timestamp
                        'timestamp_esp32': esp32_sample_id,  # ESP32 sample counter
                        'location': location,
                        'free_heap': system_data.get('free_heap', 0),
                        'uptime_sec': system_data.get('uptime_sec', 0),
                        'reset_reason': system_data.get('reset_reason', 'UNKNOWN'),
                        '_ros_topic': self.topic_name
                    }
                    
                    # Save to system_health collection
                    result_health = self.system_health_collection.insert_one(system_health_doc)
                    
                    if result_health.inserted_id:
                        self.get_logger().debug(
                            f"System health saved: {device_id} | "
                            f"heap: {system_data.get('free_heap', 0)} | "
                            f"uptime: {system_data.get('uptime_sec', 0)}s | "
                            f"reset: {system_data.get('reset_reason', 'UNKNOWN')}"
                        )
                    
                except Exception as e:
                    self.get_logger().error(f"Failed to save system_health: {e}")
            
            # === BIOLOGICAL DATA ROUTING ===
            # Extract sensor values (data is now clean, no 'system' key)
            sensors = data.get('sensors', {})
            ph_data = sensors.get('ph', {})
            temp_data = sensors.get('temperature', {})
            
            temperature = temp_data.get('value', 0)
            ph = ph_data.get('value', 0)
            
            # Validate sensor ranges
            temp_valid = -20 <= temperature <= 80
            ph_valid = 0 <= ph <= 14
            
            # Create telemetria document (time-series data, CLEAN - no system telemetry)
            telemetria_doc = {
                'device_id': device_id,
                'location': location,
                'timestamp': server_timestamp,  # SERVER TIMESTAMP
                'esp32_sample_id': esp32_sample_id,  # ESP32 counter for debugging
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
                self._log_sensor_data(telemetria_doc, system_data)
            
            # Save to MongoDB (three-collection architecture)
            if self.mongodb_connected and self.telemetria_collection is not None:
                try:
                    # 1. Insert reading into telemetria
                    result = self.telemetria_collection.insert_one(telemetria_doc)
                    
                    if result.inserted_id:
                        self.messages_saved += 1
                        
                        # 2. Update/create device in devices collection
                        self._update_device_metadata(device_id, location, server_timestamp)
                        
                        self.get_logger().debug(
                            f"Saved: {device_id} | pH: {ph:.2f} | Temp: {temperature:.2f}°C"
                        )
                except Exception as e:
                    # CORRECCIÓN 2: PyMongo maneja la reconexión. Solo registramos el error sin recrear el cliente.
                    self.get_logger().error(f"MongoDB operation failed (auto-reconnecting in background): {e}")
                    self.messages_failed += 1
                        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parse error: {e} | Raw data: {msg.data[:100]}")
            self.messages_failed += 1
        except KeyError as e:
            self.get_logger().error(f"Missing key in JSON: {e}")
            self.messages_failed += 1
        except Exception as e:
            self.get_logger().error(f"Unexpected error processing message: {type(e).__name__}: {e}")
            self.messages_failed += 1
    
    def _update_device_metadata(self, device_id: str, location: str, timestamp: str):
        """Update device metadata atomically using upsert, limpiando 'conexion' si es necesario."""
        try:
            # Validar y limpiar el campo 'conexion' si no es un objeto
            doc = self.devices_collection.find_one({'_id': device_id})
            if doc and 'conexion' in doc:
                self.get_logger().info(f"[DEBUG] Valor actual de 'conexion' en DB para {device_id}: {repr(doc['conexion'])} (tipo: {type(doc['conexion']).__name__})")
                if not isinstance(doc['conexion'], dict):
                    self.get_logger().warning(f"Corrigiendo campo 'conexion' corrupto para {device_id} (era {type(doc['conexion']).__name__})")
                    self.devices_collection.update_one({'_id': device_id}, {'$unset': {'conexion': ""}})

            update_dict = {
                '$set': {
                    'conexion.ultima': timestamp,
                    'estado': 'activo',
                    'location': location
                },
                '$inc': {
                    'conexion.total_lecturas': 1
                },
                '$setOnInsert': {
                    'alias': f'ESP32-{device_id[-4:]}',
                    'auto_registrado': True,
                    'firmware_version': 'unknown',
                    'intervalo_lectura_seg': 4,
                    'sensores_habilitados': ['ph', 'temperatura'],
                    'umbrales': {},
                    'unidades': {
                        'temperatura': '°C',
                        'ph': 'pH'
                    },
                    'conexion': {
                        'primera': timestamp
                    }
                }
            }
            self.get_logger().info(f"[DEBUG] update_one dict para {device_id}: {update_dict}")
            self.devices_collection.update_one(
                {'_id': device_id},
                update_dict,
                upsert=True
            )
        except Exception as e:
            import traceback
            tb = traceback.format_exc()
            self.get_logger().error(f"Error atomic updating device metadata: {e}\nTraceback:\n{tb}")
    
    def _log_sensor_data(self, data: dict, system_data: dict = None) -> None:
        """Display sensor data and system telemetry in the log."""
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
        
        status_ph = "OK" if ph_valid else "ERR"
        status_temp = "OK" if temp_valid else "ERR"
        
        # Base sensor data log
        log_msg = (
            f"[{device}@{location}] "
            f"pH: {ph_val} [{status_ph}] | "
            f"Temp: {temp_val}°C [{status_temp}]"
        )
        
        # Append system telemetry if available
        if system_data:
            free_heap_kb = system_data.get('free_heap', 0) / 1024
            uptime_min = system_data.get('uptime_sec', 0) / 60
            reset_reason = system_data.get('reset_reason', 'UNKNOWN')
            log_msg += f" | Heap: {free_heap_kb:.1f}KB | Uptime: {uptime_min:.1f}min | Reset: {reset_reason}"
        
        self.get_logger().info(log_msg)
    
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
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shut down — safe to ignore


if __name__ == '__main__':
    main()
