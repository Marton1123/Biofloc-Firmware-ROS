#!/usr/bin/env python3
"""
Verificación de Arquitectura de Gateway Seguro

Script para verificar que todos los componentes de la nueva arquitectura
estén funcionando correctamente.

Uso:
    python3 scripts/verify_secure_gateway.py
"""

import subprocess
import sys
import socket
import json
from datetime import datetime

def print_header(title):
    """Print formatted header"""
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}\n")

def print_status(check, passed, message):
    """Print check status"""
    status = "✅ PASS" if passed else "❌ FAIL"
    print(f"[{status}] {check}")
    if message:
        print(f"       └─ {message}")

def check_network_config():
    """Verify network configuration"""
    print_header("1. Verificación de Red")
    
    # Check if wlo1 exists
    try:
        result = subprocess.run(['ip', 'addr', 'show', 'wlo1'], 
                              capture_output=True, text=True, check=False)
        wlo1_exists = result.returncode == 0
        
        if wlo1_exists:
            # Check if 10.42.0.1 is present
            has_gateway_ip = '10.42.0.1' in result.stdout
            print_status("Interfaz WiFi (wlo1)", wlo1_exists, 
                        "Detectada" if wlo1_exists else "No encontrada")
            print_status("IP Gateway (10.42.0.1)", has_gateway_ip,
                        "Configurada" if has_gateway_ip else "Falta configuración")
        else:
            print_status("Interfaz WiFi (wlo1)", False, 
                        "No encontrada - verifica el nombre de interfaz")
    except Exception as e:
        print_status("Verificación de red", False, f"Error: {e}")
    
    # Check UDP port 8888
    try:
        result = subprocess.run(['ss', '-ulnp'], 
                              capture_output=True, text=True, check=False)
        port_listening = ':8888' in result.stdout
        print_status("Puerto UDP 8888", port_listening,
                    "Agente escuchando" if port_listening else "Agente no detectado")
    except Exception as e:
        print_status("Puerto UDP 8888", False, f"Error: {e}")

def check_ros_environment():
    """Verify ROS 2 environment"""
    print_header("2. Verificación de ROS 2")
    
    # Check if ROS 2 is sourced
    ros_sourced = 'ROS_DISTRO' in subprocess.os.environ
    distro = subprocess.os.environ.get('ROS_DISTRO', 'N/A')
    print_status("ROS 2 Environment", ros_sourced,
                f"ROS_DISTRO={distro}" if ros_sourced else "No sourced")
    
    # Check micro-ROS agent
    try:
        result = subprocess.run(['which', 'micro_ros_agent'],
                              capture_output=True, text=True, check=False)
        agent_installed = result.returncode == 0
        print_status("micro-ROS Agent", agent_installed,
                    result.stdout.strip() if agent_installed else "No instalado")
    except Exception as e:
        print_status("micro-ROS Agent", False, f"Error: {e}")

def check_mongodb_connection():
    """Verify MongoDB connection"""
    print_header("3. Verificación de MongoDB")
    
    try:
        from dotenv import load_dotenv
        from pymongo import MongoClient
        from pathlib import Path
        import os
        
        # Load .env from project root
        project_root = Path(__file__).parent.parent
        env_file = project_root / '.env'
        if env_file.exists():
            load_dotenv(env_file)
        else:
            load_dotenv('scripts/.env')  # Fallback
        uri = os.getenv('MONGODB_URI', '')
        
        has_uri = bool(uri)
        print_status("Configuración .env", has_uri,
                    "MONGODB_URI configurado" if has_uri else "Falta MONGODB_URI")
        
        if has_uri:
            # Try to connect
            try:
                client = MongoClient(uri, serverSelectionTimeoutMS=5000)
                client.admin.command('ping')
                print_status("Conexión MongoDB", True,
                            "Conectado exitosamente")
                
                # Check collections
                db = client[os.getenv('MONGODB_DATABASE', 'SistemasLab')]
                collections = db.list_collection_names()
                has_telemetria = 'telemetria' in collections
                has_devices = 'devices' in collections
                
                print_status("Colección 'telemetria'", has_telemetria,
                            "Existe" if has_telemetria else "No encontrada")
                print_status("Colección 'devices'", has_devices,
                            "Existe" if has_devices else "No encontrada")
                
                client.close()
            except Exception as e:
                print_status("Conexión MongoDB", False, f"Error: {e}")
    except ImportError:
        print_status("Dependencias Python", False,
                    "Falta pymongo o python-dotenv")

def check_esp32_firmware():
    """Check ESP32 firmware configuration"""
    print_header("4. Verificación de Firmware ESP32")
    
    try:
        # Check if sdkconfig exists
        import os
        has_sdkconfig = os.path.exists('sdkconfig')
        print_status("Archivo sdkconfig", has_sdkconfig,
                    "Encontrado" if has_sdkconfig else "Ejecuta 'idf.py menuconfig'")
        
        if has_sdkconfig:
            with open('sdkconfig', 'r') as f:
                config = f.read()
                
            # Check critical settings
            checks = {
                'WiFi SSID': ('CONFIG_BIOFLOC_WIFI_SSID="lab-ros2-nuc"', 
                             'SSID configurado correctamente'),
                'Agent IP': ('CONFIG_BIOFLOC_AGENT_IP="10.42.0.1"',
                            'IP del gateway configurada'),
                'Agent Port': ('CONFIG_BIOFLOC_AGENT_PORT=8888',
                              'Puerto UDP correcto'),
            }
            
            for name, (setting, success_msg) in checks.items():
                found = setting in config
                print_status(name, found,
                            success_msg if found else f"Falta: {setting}")
    except Exception as e:
        print_status("Verificación firmware", False, f"Error: {e}")

def check_ros_topics():
    """Check ROS 2 topics"""
    print_header("5. Verificación de Topics ROS 2")
    
    try:
        result = subprocess.run(['ros2', 'topic', 'list'],
                              capture_output=True, text=True, check=False)
        
        if result.returncode == 0:
            topics = result.stdout
            has_sensor_data = '/biofloc/sensor_data' in topics
            
            print_status("Topic /biofloc/sensor_data", has_sensor_data,
                        "ESP32 publicando" if has_sensor_data else "ESP32 no conectado")
            
            if has_sensor_data:
                # Try to read one message
                try:
                    result = subprocess.run(
                        ['timeout', '5', 'ros2', 'topic', 'echo', 
                         '/biofloc/sensor_data', '--once'],
                        capture_output=True, text=True, check=False
                    )
                    
                    if result.returncode == 0:
                        # Parse JSON from message
                        lines = result.stdout.strip().split('\n')
                        for line in lines:
                            if line.startswith('data:'):
                                json_str = line.replace('data:', '').strip().strip("'\"")
                                try:
                                    data = json.loads(json_str)
                                    timestamp = data.get('timestamp', 'N/A')
                                    device_id = data.get('device_id', 'N/A')
                                    
                                    # Check timestamp format
                                    is_sample = timestamp.startswith('sample_')
                                    print_status("Formato Timestamp ESP32", is_sample,
                                                f"'{timestamp}' (será reemplazado por servidor)")
                                    print_status("Device ID", True, device_id)
                                    
                                    # Check sensor values
                                    sensors = data.get('sensors', {})
                                    ph = sensors.get('ph', {}).get('value', 'N/A')
                                    temp = sensors.get('temperature', {}).get('value', 'N/A')
                                    print_status("Datos de sensores", True,
                                                f"pH={ph}, Temp={temp}°C")
                                except json.JSONDecodeError:
                                    print_status("Parse JSON", False, "Formato inválido")
                except Exception as e:
                    print_status("Lectura de topic", False, f"Error: {e}")
        else:
            print_status("ROS 2 Topics", False,
                        "No se puede listar topics (¿ROS sourced?)")
    except FileNotFoundError:
        print_status("ROS 2 CLI", False, "ros2 command no encontrado")
    except Exception as e:
        print_status("Verificación topics", False, f"Error: {e}")

def check_bridge_process():
    """Check if sensor_db_bridge is running"""
    print_header("6. Verificación del Bridge Python")
    
    try:
        result = subprocess.run(['ps', 'aux'],
                              capture_output=True, text=True, check=False)
        
        bridge_running = 'sensor_db_bridge.py' in result.stdout
        print_status("Proceso sensor_db_bridge.py", bridge_running,
                    "Ejecutando" if bridge_running else "No detectado")
        
        if not bridge_running:
            print("       💡 Iniciar con: python3 scripts/sensor_db_bridge.py")
    except Exception as e:
        print_status("Verificación bridge", False, f"Error: {e}")

def main():
    """Run all checks"""
    print("\n" + "="*60)
    print("  🔒 Verificación de Arquitectura de Gateway Seguro")
    print("="*60)
    print(f"  Fecha: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)
    
    # Run all checks
    check_network_config()
    check_ros_environment()
    check_mongodb_connection()
    check_esp32_firmware()
    check_ros_topics()
    check_bridge_process()
    
    # Summary
    print_header("Resumen")
    print("Si todos los checks pasan (✅), la arquitectura está correctamente configurada.")
    print("Para más detalles, consulta: SECURE_GATEWAY_MIGRATION.md")
    print()

if __name__ == "__main__":
    main()
