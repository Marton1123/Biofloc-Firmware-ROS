#!/usr/bin/env python3
"""
MongoDB Migration Verification Script

Verifies the two-collection architecture migration was successful.
Checks indexes, device documents, and data integrity.

Usage:
    python3 scripts/verify_migration.py

Requirements:
    - .env file with MONGODB_URI configured
    - pymongo, python-dotenv installed

Author: @Marton1123
Version: 3.0.0
"""

import os
import sys
from pathlib import Path
from typing import Dict, List

try:
    from dotenv import load_dotenv
    from pymongo import MongoClient
    from pymongo.errors import ConnectionFailure
except ImportError as e:
    print(f"ERROR: Missing required package: {e}")
    print("Install with: pip install pymongo python-dotenv")
    sys.exit(1)


class MigrationVerifier:
    """Verifies MongoDB migration to two-collection architecture."""
    
    def __init__(self):
        """Initialize verifier with MongoDB connection."""
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
            print(f"Connected to MongoDB: {self.db_name}\n")
        except ConnectionFailure as e:
            print(f"ERROR: Failed to connect to MongoDB: {e}")
            sys.exit(1)
        
        # Collections
        self.telemetria = self.db['telemetria']
        self.devices = self.db['devices']
        self.system_config = self.db['system_config']
    
    def _load_environment(self):
        """Load environment variables from .env file."""
        script_dir = Path(__file__).parent
        env_file = script_dir / '.env'
        
        if env_file.exists():
            load_dotenv(env_file)
        else:
            env_file = script_dir.parent / '.env'
            if env_file.exists():
                load_dotenv(env_file)
            else:
                print("ERROR: No .env file found")
                sys.exit(1)
    
    def run(self) -> bool:
        """Execute verification process."""
        print("=" * 70)
        print("MongoDB Migration Verification")
        print("=" * 70)
        
        all_passed = True
        
        # Test 1: Check collections exist
        print("\n[1/6] Checking collections exist...")
        if not self._check_collections():
            all_passed = False
        
        # Test 2: Verify indexes
        print("\n[2/6] Verifying indexes...")
        if not self._verify_indexes():
            all_passed = False
        
        # Test 3: Check device documents structure
        print("\n[3/6] Checking device documents structure...")
        if not self._check_devices_structure():
            all_passed = False
        
        # Test 4: Verify connection statistics
        print("\n[4/6] Verifying connection statistics...")
        if not self._verify_connection_stats():
            all_passed = False
        
        # Test 5: Check telemetria data integrity
        print("\n[5/6] Checking telemetria data integrity...")
        if not self._check_telemetria_integrity():
            all_passed = False
        
        # Test 6: Summary statistics
        print("\n[6/6] Summary statistics...")
        self._print_summary()
        
        print("\n" + "=" * 70)
        if all_passed:
            print("RESULT: Migration verification PASSED")
        else:
            print("RESULT: Migration verification FAILED (see errors above)")
        print("=" * 70)
        
        return all_passed
    
    def _check_collections(self) -> bool:
        """Verify required collections exist."""
        collections = self.db.list_collection_names()
        
        required = ['telemetria', 'devices']
        missing = [c for c in required if c not in collections]
        
        if missing:
            print(f"  ERROR: Missing collections: {', '.join(missing)}")
            return False
        
        print(f"  OK: All required collections exist")
        return True
    
    def _verify_indexes(self) -> bool:
        """Verify indexes on telemetria collection."""
        indexes = list(self.telemetria.list_indexes())
        index_names = [idx['name'] for idx in indexes]
        
        required_indexes = ['idx_device_timestamp', 'idx_timestamp']
        missing = [idx for idx in required_indexes if idx not in index_names]
        
        if missing:
            print(f"  WARNING: Missing indexes: {', '.join(missing)}")
            print("  Run: python3 scripts/migrate_to_devices_collection.py")
            return False
        
        print(f"  OK: All required indexes exist")
        for idx in indexes:
            if idx['name'] != '_id_':
                print(f"    - {idx['name']}: {idx['key']}")
        
        return True
    
    def _check_devices_structure(self) -> bool:
        """Verify device documents have correct structure."""
        required_fields = [
            '_id', 'alias', 'location', 'estado', 'auto_registrado',
            'firmware_version', 'intervalo_lectura_seg', 'sensores_habilitados',
            'unidades', 'conexion'
        ]
        
        conexion_fields = ['primera', 'ultima', 'total_lecturas']
        
        devices = list(self.devices.find())
        
        if len(devices) == 0:
            print(f"  WARNING: No devices found in devices collection")
            return False
        
        all_valid = True
        for device in devices:
            device_id = device.get('_id', 'unknown')
            
            # Check required fields
            missing = [f for f in required_fields if f not in device]
            if missing:
                print(f"  ERROR: Device {device_id} missing fields: {', '.join(missing)}")
                all_valid = False
                continue
            
            # Check conexion subfields
            conexion = device.get('conexion', {})
            missing_conn = [f for f in conexion_fields if f not in conexion]
            if missing_conn:
                print(f"  ERROR: Device {device_id} missing conexion fields: {', '.join(missing_conn)}")
                all_valid = False
        
        if all_valid:
            print(f"  OK: All {len(devices)} devices have correct structure")
        
        return all_valid
    
    def _verify_connection_stats(self) -> bool:
        """Verify connection statistics match telemetria data."""
        devices = list(self.devices.find())
        
        all_match = True
        for device in devices:
            device_id = device['_id']
            
            # Get stats from device document
            device_count = device.get('conexion', {}).get('total_lecturas', 0)
            
            # Count in telemetria
            actual_count = self.telemetria.count_documents({'device_id': device_id})
            
            if device_count != actual_count:
                print(f"  WARNING: Device {device_id} count mismatch:")
                print(f"    Device document: {device_count}")
                print(f"    Telemetria actual: {actual_count}")
                all_match = False
            else:
                print(f"  OK: Device {device_id} - {actual_count} readings")
        
        return all_match
    
    def _check_telemetria_integrity(self) -> bool:
        """Check telemetria data integrity."""
        total = self.telemetria.count_documents({})
        
        if total == 0:
            print(f"  WARNING: No documents in telemetria collection")
            return False
        
        # Check for documents without device_id
        missing_device = self.telemetria.count_documents({'device_id': {'$exists': False}})
        
        # Check for documents without timestamp
        missing_timestamp = self.telemetria.count_documents({'timestamp': {'$exists': False}})
        
        # Check for documents without sensors
        missing_sensors = self.telemetria.count_documents({'sensors': {'$exists': False}})
        
        issues = []
        if missing_device > 0:
            issues.append(f"{missing_device} without device_id")
        if missing_timestamp > 0:
            issues.append(f"{missing_timestamp} without timestamp")
        if missing_sensors > 0:
            issues.append(f"{missing_sensors} without sensors")
        
        if issues:
            print(f"  WARNING: Data integrity issues:")
            for issue in issues:
                print(f"    - {issue}")
            return False
        
        print(f"  OK: All {total} documents have required fields")
        return True
    
    def _print_summary(self):
        """Print summary statistics."""
        telemetria_count = self.telemetria.count_documents({})
        devices_count = self.devices.count_documents({})
        
        print(f"  Telemetria documents: {telemetria_count}")
        print(f"  Devices registered: {devices_count}")
        
        # List devices
        devices = list(self.devices.find())
        if devices:
            print(f"\n  Registered devices:")
            for device in devices:
                device_id = device['_id']
                alias = device.get('alias', 'N/A')
                location = device.get('location', 'N/A')
                estado = device.get('estado', 'N/A')
                lecturas = device.get('conexion', {}).get('total_lecturas', 0)
                auto = device.get('auto_registrado', False)
                
                reg_type = "auto" if auto else "manual"
                print(f"    - {device_id} ({alias})")
                print(f"        Location: {location} | Status: {estado} | Readings: {lecturas} | Reg: {reg_type}")


def main():
    """Main entry point."""
    try:
        verifier = MigrationVerifier()
        success = verifier.run()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nVerification interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: Verification failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
