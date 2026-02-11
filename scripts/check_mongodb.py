#!/home/Biofloc-Firmware-ROS/scripts/.venv/bin/python3
"""Quick script to check MongoDB data"""

import os
from pathlib import Path
from dotenv import load_dotenv
from pymongo import MongoClient
from datetime import datetime

# Load environment from project root
project_root = Path(__file__).parent.parent
env_file = project_root / '.env'
if env_file.exists():
    load_dotenv(env_file)
else:
    # Fallback to scripts/.env for backward compatibility
    load_dotenv(Path(__file__).parent / '.env')

# Connect to MongoDB
uri = os.getenv('MONGODB_URI')
db_name = os.getenv('MONGODB_DATABASE')
collection_name = os.getenv('MONGODB_COLLECTION')

client = MongoClient(uri, serverSelectionTimeoutMS=5000)
db = client[db_name]
collection = db[collection_name]

# Get count
count = collection.count_documents({})
print(f"✓ Total documents in {db_name}.{collection_name}: {count}")

# Get latest 5 documents
print("\n📊 Latest 5 documents:")
print("="*80)
for doc in collection.find().sort('_received_at', -1).limit(5):
    device_id = doc.get('device_id', 'N/A')
    timestamp = doc.get('timestamp', 'N/A')
    location = doc.get('location', 'N/A')
    ph = doc.get('sensors', {}).get('ph', {}).get('value', 'N/A')
    temp = doc.get('sensors', {}).get('temperature', {}).get('value', 'N/A')
    received_at = doc.get('_received_at', 'N/A')
    
    print(f"Device: {device_id} @ {location}")
    print(f"  ESP32 Time: {timestamp}")
    print(f"  Received:   {received_at}")
    print(f"  pH: {ph}  |  Temp: {temp}°C")
    print("-"*80)

client.close()
