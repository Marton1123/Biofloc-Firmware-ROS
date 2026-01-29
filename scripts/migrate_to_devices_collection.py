#!/usr/bin/env python3
"""
MongoDB Migration Script - Devices Collection Architecture

Migrates device metadata from system_config.device_metadata to independent
devices collection and creates optimized indexes for telemetria collection.

Architecture:
    - telemetria: sensor readings (time-series data)
    - devices: device metadata (configuration, state, connection history)
    - system_config: global configuration

Indexes created:
    - telemetria: (device_id, timestamp) for efficient queries
    - telemetria: (timestamp) for recent readings across all devices
    - devices: (_id) - primary key on device_id

Usage:
    python3 scripts/migrate_to_devices_collection.py

Requirements:
    - .env file with MONGODB_URI configured
    - pymongo, python-dotenv installed

Author: @Marton1123
Version: 3.0.0
"""

import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

try:
    from dotenv import load_dotenv
    from pymongo import MongoClient, ASCENDING, DESCENDING
    from pymongo.errors import ConnectionFailure, DuplicateKeyError
except ImportError as e:
    print(f"ERROR: Missing required package: {e}")
    print("Install with: pip install pymongo python-dotenv")
    sys.exit(1)


class DevicesMigration:
    """Handles migration to devices collection architecture."""
    
    def __init__(self):
        """Initialize migration with MongoDB connection."""
        # Load environment
        self._load_environment()
        
        # MongoDB configuration
        self.mongodb_uri = os.getenv('MONGODB_URI')
        self.db_name = os.getenv('MONGODB_DATABASE', 'SistemasLab')
        
        if not self.mongodb_uri:
            print("ERROR: MONGODB_URI not set in .env file")
            sys.exit(1)
        
        # Connect to MongoDB
        try:
            self.client = MongoClient(self.mongodb_uri, serverSelectionTimeoutMS=5000)
            self.client.admin.command('ping')
            self.db = self.client[self.db_name]
            print(f"Connected to MongoDB: {self.db_name}")
        except ConnectionFailure as e:
            print(f"ERROR: Failed to connect to MongoDB: {e}")
            sys.exit(1)
        
        # Collections
        self.system_config = self.db['system_config']
        self.devices = self.db['devices']
        self.telemetria = self.db['telemetria']
    
    def _load_environment(self):
        """Load environment variables from .env file."""
        script_dir = Path(__file__).parent
        env_file = script_dir / '.env'
        
        if env_file.exists():
            load_dotenv(env_file)
            print(f"Loaded environment from: {env_file}")
        else:
            env_file = script_dir.parent / '.env'
            if env_file.exists():
                load_dotenv(env_file)
                print(f"Loaded environment from: {env_file}")
            else:
                print("ERROR: No .env file found")
                sys.exit(1)
    
    def run(self):
        """Execute complete migration process."""
        print("\n" + "=" * 70)
        print("MongoDB Migration: Devices Collection Architecture")
        print("=" * 70)
        
        # Step 1: Analyze current structure
        print("\n[1/5] Analyzing current structure...")
        stats = self._analyze_current_structure()
        
        # Step 2: Migrate device metadata
        print("\n[2/5] Migrating device metadata to devices collection...")
        migrated_devices = self._migrate_device_metadata()
        
        # Step 3: Enrich devices with telemetria statistics
        print("\n[3/5] Enriching devices with connection statistics...")
        self._enrich_device_statistics(migrated_devices)
        
        # Step 4: Create indexes
        print("\n[4/5] Creating optimized indexes...")
        self._create_indexes()
        
        # Step 5: Verify migration
        print("\n[5/5] Verifying migration...")
        self._verify_migration()
        
        print("\n" + "=" * 70)
        print("Migration completed successfully")
        print("=" * 70)
        print("\nNext steps:")
        print("  1. Update sensor_db_bridge.py to v3.0")
        print("  2. Add MONGODB_COLLECTION_DEVICES=devices to .env")
        print("  3. Restart bridge to use new architecture")
    
    def _analyze_current_structure(self) -> Dict:
        """Analyze current database structure."""
        stats = {
            'telemetria_count': self.telemetria.count_documents({}),
            'system_config_count': self.system_config.count_documents({}),
            'devices_count': self.devices.count_documents({}),
            'device_metadata_exists': False,
            'devices_in_metadata': 0
        }
        
        # Check for device_metadata in system_config
        device_metadata_doc = self.system_config.find_one({'_id': 'device_metadata'})
        if device_metadata_doc and 'devices' in device_metadata_doc:
            stats['device_metadata_exists'] = True
            stats['devices_in_metadata'] = len(device_metadata_doc['devices'])
        
        print(f"  Telemetria documents: {stats['telemetria_count']}")
        print(f"  System config documents: {stats['system_config_count']}")
        print(f"  Existing devices collection: {stats['devices_count']} documents")
        print(f"  Device metadata found: {stats['device_metadata_exists']}")
        if stats['device_metadata_exists']:
            print(f"  Devices in metadata: {stats['devices_in_metadata']}")
        
        return stats
    
    def _migrate_device_metadata(self) -> List[str]:
        """Migrate devices from system_config.device_metadata to devices collection."""
        migrated = []
        
        # Get device_metadata document
        device_metadata_doc = self.system_config.find_one({'_id': 'device_metadata'})
        
        if not device_metadata_doc or 'devices' not in device_metadata_doc:
            print("  No device_metadata found in system_config")
            print("  Skipping migration (will use auto-registration)")
            return migrated
        
        devices_dict = device_metadata_doc['devices']
        
        for device_id, device_data in devices_dict.items():
            try:
                # Build device document
                device_doc = {
                    '_id': device_id,
                    'alias': device_data.get('alias', f'ESP32-{device_id[-4:]}'),
                    'location': device_data.get('location', 'unknown'),
                    'estado': 'activo',  # Default to active for existing devices
                    'auto_registrado': False,  # These are manually configured
                    'firmware_version': 'unknown',
                    'intervalo_lectura_seg': 4,
                    'sensores_habilitados': ['ph', 'temperatura'],
                    'umbrales': device_data.get('thresholds', {}),
                    'unidades': {
                        'temperatura': '°C',
                        'ph': 'pH'
                    },
                    'conexion': {
                        'primera': None,
                        'ultima': None,
                        'total_lecturas': 0
                    },
                    'migrated_at': datetime.now().isoformat(),
                    'migrated_from': 'system_config.device_metadata'
                }
                
                # Insert or update
                result = self.devices.replace_one(
                    {'_id': device_id},
                    device_doc,
                    upsert=True
                )
                
                if result.upserted_id:
                    print(f"  Created device: {device_id} ({device_data.get('alias', 'unknown')})")
                else:
                    print(f"  Updated device: {device_id} ({device_data.get('alias', 'unknown')})")
                
                migrated.append(device_id)
                
            except DuplicateKeyError:
                print(f"  Device already exists: {device_id}")
                migrated.append(device_id)
            except Exception as e:
                print(f"  ERROR migrating device {device_id}: {e}")
        
        return migrated
    
    def _enrich_device_statistics(self, device_ids: List[str]):
        """Calculate connection statistics from telemetria for each device."""
        print(f"  Analyzing telemetria for {len(device_ids)} devices...")
        
        # Also check for devices in telemetria but not in metadata
        all_device_ids = self.telemetria.distinct('device_id')
        
        for device_id in all_device_ids:
            # Get first and last reading
            first_reading = self.telemetria.find_one(
                {'device_id': device_id},
                sort=[('timestamp', ASCENDING)]
            )
            
            last_reading = self.telemetria.find_one(
                {'device_id': device_id},
                sort=[('timestamp', DESCENDING)]
            )
            
            total_readings = self.telemetria.count_documents({'device_id': device_id})
            
            # Update device document
            update_doc = {
                'conexion.primera': first_reading['timestamp'] if first_reading else None,
                'conexion.ultima': last_reading['timestamp'] if last_reading else None,
                'conexion.total_lecturas': total_readings
            }
            
            # If device doesn't exist, create it with auto-registration
            if device_id not in device_ids:
                print(f"  Found unregistered device: {device_id} ({total_readings} readings)")
                update_doc.update({
                    '_id': device_id,
                    'alias': f'ESP32-{device_id[-4:]}',
                    'location': 'unknown',
                    'estado': 'pendiente',
                    'auto_registrado': True,
                    'firmware_version': 'unknown',
                    'intervalo_lectura_seg': 4,
                    'sensores_habilitados': ['ph', 'temperatura'],
                    'umbrales': {},
                    'unidades': {'temperatura': '°C', 'ph': 'pH'}
                })
                
                self.devices.replace_one(
                    {'_id': device_id},
                    update_doc,
                    upsert=True
                )
            else:
                self.devices.update_one(
                    {'_id': device_id},
                    {'$set': update_doc}
                )
            
            print(f"    {device_id}: {total_readings} readings, "
                  f"first: {first_reading['timestamp'] if first_reading else 'N/A'}, "
                  f"last: {last_reading['timestamp'] if last_reading else 'N/A'}")
    
    def _create_indexes(self):
        """Create optimized indexes for efficient queries."""
        # Index 1: telemetria - (device_id, timestamp DESC)
        # Use case: Get recent readings for specific device
        try:
            index_name = self.telemetria.create_index(
                [('device_id', ASCENDING), ('timestamp', DESCENDING)],
                name='idx_device_timestamp'
            )
            print(f"  Created index: telemetria.{index_name}")
        except Exception as e:
            print(f"  Index already exists or error: {e}")
        
        # Index 2: telemetria - (timestamp DESC)
        # Use case: Get most recent readings across all devices
        try:
            index_name = self.telemetria.create_index(
                [('timestamp', DESCENDING)],
                name='idx_timestamp'
            )
            print(f"  Created index: telemetria.{index_name}")
        except Exception as e:
            print(f"  Index already exists or error: {e}")
        
        # List all indexes
        print("\n  Current indexes in telemetria:")
        for index in self.telemetria.list_indexes():
            print(f"    - {index['name']}: {index['key']}")
    
    def _verify_migration(self):
        """Verify migration completed successfully."""
        devices_count = self.devices.count_documents({})
        telemetria_count = self.telemetria.count_documents({})
        
        print(f"  Devices collection: {devices_count} documents")
        print(f"  Telemetria collection: {telemetria_count} documents")
        
        # Verify all devices have connection data
        devices_without_stats = self.devices.count_documents({
            'conexion.total_lecturas': 0
        })
        
        if devices_without_stats > 0:
            print(f"  WARNING: {devices_without_stats} devices have no readings")
        
        # Sample device document
        sample_device = self.devices.find_one()
        if sample_device:
            print("\n  Sample device document:")
            print(f"    _id: {sample_device['_id']}")
            print(f"    alias: {sample_device.get('alias', 'N/A')}")
            print(f"    location: {sample_device.get('location', 'N/A')}")
            print(f"    estado: {sample_device.get('estado', 'N/A')}")
            print(f"    total_lecturas: {sample_device.get('conexion', {}).get('total_lecturas', 0)}")
        
        print("\n  Migration verification completed")


def main():
    """Main entry point."""
    try:
        migration = DevicesMigration()
        migration.run()
    except KeyboardInterrupt:
        print("\n\nMigration interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: Migration failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
