#!/usr/bin/env python3
"""
Biofloc Firmware Manager - Gestor centralizado del sistema
Provides a unified interface for all system operations
"""

import os
import sys
import subprocess
import time
import json
from pathlib import Path
from datetime import datetime

# Load environment variables
try:
    from dotenv import load_dotenv
    env_file = Path(__file__).parent / '.env'
    if not env_file.exists():
        # Try scripts/.env
        env_file = Path(__file__).parent / 'scripts' / '.env'
    if env_file.exists():
        load_dotenv(env_file)
except ImportError:
    pass  # dotenv is optional

# Colors for terminal output
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Project paths
PROJECT_ROOT = Path(__file__).parent.absolute()
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
ESP_IDF_PATH = Path.home() / "esp" / "v5.3.4" / "esp-idf"
MICROROS_WS = Path.home() / "microros_ws"

# Detect if running on Raspberry Pi (ARM architecture)
import platform
IS_RASPBERRY_PI = platform.machine().startswith('aarch64') or platform.machine().startswith('armv')

# Network configuration from .env (with defaults for backward compatibility)
ESP32_MAC = os.getenv("ESP32_MAC", "XX:XX:XX:XX:XX:XX")
GATEWAY_IP = os.getenv("GATEWAY_IP", "10.42.0.1")
NETWORK_RANGE = os.getenv("GATEWAY_NETWORK", "10.42.0.0/24")

def detect_gateway_info():
    """Detect active WiFi gateway information (IP and SSID)"""
    try:
        # Get IP address of wlan0 interface
        result = subprocess.run(
            ["ip", "-4", "addr", "show", "wlan0"],
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            import re
            # Extract IP address
            match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', result.stdout)
            if match:
                gateway_ip = match.group(1)
                
                # Try to get SSID
                ssid_result = subprocess.run(
                    ["iwgetid", "-r"],
                    capture_output=True,
                    text=True
                )
                
                ssid = ssid_result.stdout.strip() if ssid_result.returncode == 0 else "<desconocido>"
                
                return gateway_ip, ssid
        
        # Fallback to default
        return "10.42.0.1", "<desconocido>"
        
    except Exception as e:
        print_warning(f"No se pudo detectar configuración de red: {e}")
        return "10.42.0.1", "<desconocido>"

def print_header(text):
    """Print formatted header"""
    print(f"\n{Colors.HEADER}{Colors.BOLD}{'='*70}")
    print(f"  {text}")
    print(f"{'='*70}{Colors.ENDC}\n")

def print_success(text):
    """Print success message"""
    print(f"{Colors.OKGREEN}✓ {text}{Colors.ENDC}")

def print_error(text):
    """Print error message"""
    print(f"{Colors.FAIL}✗ {text}{Colors.ENDC}")

def print_warning(text):
    """Print warning message"""
    print(f"{Colors.WARNING}⚠ {text}{Colors.ENDC}")

def print_info(text):
    """Print info message"""
    print(f"{Colors.OKCYAN}ℹ {text}{Colors.ENDC}")

def print_table_header(headers):
    """Print a formatted table header"""
    print(f"{Colors.BOLD}{'─' * 100}{Colors.ENDC}")
    header_line = " | ".join([f"{h:^15}" for h in headers])
    print(f"{Colors.BOLD}{header_line}{Colors.ENDC}")
    print(f"{Colors.BOLD}{'─' * 100}{Colors.ENDC}")

def print_table_row(values):
    """Print a formatted table row"""
    row_line = " | ".join([f"{str(v):^15}" for v in values])
    print(row_line)

def check_process_running(process_name):
    """Check if a process is running (robust check without grep pipes)"""
    result = subprocess.run(
        "ps aux",
        shell=True,
        capture_output=True,
        text=True
    )
    
    # Filter in Python to avoid grep pipe issues
    for line in result.stdout.splitlines():
        if process_name in line and 'grep' not in line:
            return True
    return False

def run_command(cmd, cwd=None, shell=True, check=True):
    """Execute shell command"""
    try:
        result = subprocess.run(
            cmd,
            shell=shell,
            cwd=cwd,
            check=check,
            text=True,
            capture_output=False
        )
        return result.returncode == 0
    except subprocess.CalledProcessError as e:
        print_error(f"Command failed with exit code {e.returncode}")
        return False
    except Exception as e:
        print_error(f"Error executing command: {e}")
        return False

def check_prerequisites():
    """Check if all required tools are available"""
    print_info("Verificando prerequisitos...")
    
    # On Raspberry Pi, ESP-IDF is optional (acts only as Gateway)
    if IS_RASPBERRY_PI:
        print_warning("Raspberry Pi detectada - Modo Gateway (sin compilación de firmware)")
        checks = [
            ("Workspace micro-ROS", MICROROS_WS.exists()),
            ("Directorio scripts", SCRIPTS_DIR.exists()),
        ]
    else:
        checks = [
            ("ESP-IDF", ESP_IDF_PATH.exists()),
            ("Workspace micro-ROS", MICROROS_WS.exists()),
            ("Directorio scripts", SCRIPTS_DIR.exists()),
        ]
    
    all_ok = True
    for name, exists in checks:
        if exists:
            print_success(f"{name}: OK")
        else:
            print_error(f"{name}: NO ENCONTRADO")
            all_ok = False
    
    return all_ok

def start_microros_agent():
    """Start micro-ROS Agent in background"""
    print_header("Iniciando micro-ROS Agent")
    
    # Detect gateway IP automatically
    gateway_ip, ssid = detect_gateway_info()
    
    cmd = f"source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
    
    print_info("El Agent correrá en puerto UDP 8888")
    print_info(f"IP del Gateway: {gateway_ip}")
    if ssid != "<desconocido>":
        print_info(f"Red WiFi: {ssid}")
    print_info("Presiona Ctrl+C para detener")
    print()
    
    # Use bash explicitly to support 'source' command
    subprocess.run(['bash', '-c', cmd], cwd=PROJECT_ROOT)

def start_sensor_bridge():
    """Start sensor DB bridge"""
    print_header("Iniciando Sensor DB Bridge")
    
    cmd = f"cd {SCRIPTS_DIR} && source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && python3 sensor_db_bridge.py"
    
    print_info("El Bridge se conectará a MongoDB y al topic ROS /biofloc/sensor_data")
    print_info("Presiona Ctrl+C para detener")
    print()
    
    # Use bash explicitly to support 'source' command
    subprocess.run(['bash', '-c', cmd], check=False)

def calibrate_ph():
    """Run pH calibration script"""
    print_header("Calibración de pH")
    
    print_info("Prepara 3 soluciones buffer: pH 4.01, 6.86, 9.18")
    input(f"{Colors.WARNING}Presiona Enter cuando estés listo...{Colors.ENDC}")
    
    cmd = f"cd {SCRIPTS_DIR} && python3 calibrate_ph.py"
    
    if run_command(cmd, check=False):
        print_success("Calibración de pH completada")
        print_info("Resultados guardados en calibration_3point_result.txt")
        print_warning("¡Recuerda recompilar y flashear el firmware!")
    else:
        print_error("Calibración de pH falló")

def calibrate_temperature():
    """Run temperature calibration script"""
    print_header("Calibración de Temperatura")
    
    print_info("Prepara un termómetro de referencia (TP101 o equivalente)")
    print_info("Necesitarás 3 puntos de temperatura (ej: agua helada, ambiente, agua tibia)")
    input(f"{Colors.WARNING}Presiona Enter cuando estés listo...{Colors.ENDC}")
    
    cmd = f"cd {SCRIPTS_DIR} && python3 calibrate_temperature.py"
    
    if run_command(cmd, check=False):
        print_success("Calibración de temperatura completada")
        
        # Try to read calibration results
        result_file = SCRIPTS_DIR / "temperature_calibration_result.txt"
        if result_file.exists():
            try:
                with open(result_file, 'r') as f:
                    content = f.read()
                    # Extract offset from results
                    import re
                    offset_match = re.search(r'Offset promedio:\s*([+-]?\d+\.?\d*)', content)
                    if offset_match:
                        offset_celsius = float(offset_match.group(1))
                        offset_millidegrees = int(offset_celsius * 1000)
                        
                        print_info(f"Offset detectado: {offset_celsius}°C ({offset_millidegrees} miligrados)")
                        
                        choice = input("\n¿Actualizar sdkconfig.defaults automáticamente? (S/n): ").strip().lower()
                        if choice != 'n':
                            update_calibration('temperature', slope=1000000, offset=offset_millidegrees)
            except Exception as e:
                print_warning(f"No se pudo auto-actualizar la configuración: {e}")
                print_info("Deberás actualizar manualmente")
        else:
            print_warning("Archivo de resultados de calibración no encontrado")
    else:
        print_error("Calibración de temperatura falló")

def calibrate_remote():
    """
    Remote calibration via ROS 2 topics (v3.1.0+)
    
    Calibrates sensors remotely without USB connection.
    Supports pH, temperature, and future sensors with N-point calibration.
    Includes device selection for multi-ESP32 environments.
    """
    print_header("Calibración Remota de Sensores")
    
    print_info("Sistema de calibración remota v3.6.2 (Multi-Device)")
    print_info("Compatible con: pH, Temperatura, y sensores futuros")
    print_info("")
    print_info("Ventajas:")
    print_info("  ✓ No requiere conexión USB")
    print_info("  ✓ ESP32 mantiene alimentación externa")
    print_info("  ✓ Sensores permanecen energizados")
    print_info("  ✓ Calibración de 2-5 puntos")
    print_info("  ✓ Persistencia automática en NVS")
    print_info("  ✓ Filtrado de dispositivo en redes multi-ESP32")
    print("")
    
    # Check if micro-ROS Agent is running
    if not check_process_running('micro_ros_agent'):
        print_error("El micro-ROS Agent NO está corriendo")
        print_info("Necesitas iniciarlo primero con la opción [1] del menú")
        print_info("O ejecuta manualmente:")
        print_info("  source /opt/ros/jazzy/setup.bash")
        print_info("  source ~/microros_ws/install/local_setup.bash")
        print_info("  ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888")
        print()
        return
    
    # === DEVICE DISCOVERY (NEW in v3.1.0) ===
    print()
    print_header("Paso 1: Selección de Dispositivo")
    print_info("Detectando dispositivos ESP32 activos en la red...")
    print()
    
    devices = discover_active_devices(duration_seconds=10)
    
    if not devices:
        print_error("No se detectaron dispositivos ESP32 en la red")
        print_info("Verifica que:")
        print_info("  - Los ESP32 estén encendidos y conectados")
        print_info("  - El micro-ROS Agent esté activo")
        print_info("  - Los dispositivos estén publicando en /biofloc/sensor_data")
        return
    
    # Let user select device
    print()
    print(f"{Colors.BOLD}Dispositivos disponibles para calibración:{Colors.ENDC}")
    device_list = list(devices.keys())
    for idx, device_id in enumerate(device_list, start=1):
        data = devices[device_id]
        location = data.get('location', 'unknown')
        sensors = data.get('sensors', {})
        ph = sensors.get('ph', {}).get('value', 'N/A')
        temp = sensors.get('temperature', {}).get('value', 'N/A')
        
        print(f"  [{idx}] {device_id}")
        print(f"      Ubicación: {location}")
        print(f"      Sensores: pH={ph}, Temp={temp}°C")
    print("  [0] Cancelar")
    print()
    
    device_choice = input(f"{Colors.OKBLUE}Selecciona dispositivo a calibrar: {Colors.ENDC}").strip()
    
    try:
        device_idx = int(device_choice)
        if device_idx == 0:
            print_info("Calibración cancelada")
            return
        if device_idx < 1 or device_idx > len(device_list):
            print_error("Opción inválida")
            return
        
        selected_device = device_list[device_idx - 1]
        print_success(f"✓ Dispositivo seleccionado: {selected_device}")
        print()
    except ValueError:
        print_error("Ingresa un número válido")
        return
    
    # === SENSOR SELECTION ===
    print()
    print_header("Paso 2: Selección de Sensor")
    
    # Select sensor type
    print(f"{Colors.BOLD}Sensores disponibles:{Colors.ENDC}")
    print("  [1] pH")
    print("  [2] Temperatura")
    print("  [3] Oxígeno Disuelto (futuro)")
    print("  [4] Conductividad (futuro)")
    print("  [5] Turbidez (futuro)")
    print("  [0] Cancelar")
    print("")
    
    sensor_choice = input(f"{Colors.OKBLUE}Selecciona sensor: {Colors.ENDC}").strip()
    
    sensor_map = {
        '1': ('ph', 'pH', [4.01, 6.86, 9.18]),
        '2': ('temperature', 'Temperatura', [0.0, 25.0, 50.0]),
        '3': ('dissolved_oxygen', 'Oxígeno Disuelto', [0.0, 5.0, 10.0]),
        '4': ('conductivity', 'Conductividad', [0.0, 500.0, 1000.0]),
        '5': ('turbidity', 'Turbidez', [0.0, 50.0, 100.0])
    }
    
    if sensor_choice == '0':
        print_info("Calibración cancelada")
        return
    
    if sensor_choice not in sensor_map:
        print_error("Opción inválida")
        return
    
    sensor_id, sensor_name, suggested_values = sensor_map[sensor_choice]
    
    # Check if sensor is implemented
    if sensor_choice in ['3', '4', '5']:
        print_warning(f"⚠ {sensor_name} aún no está implementado en hardware")
        print_info("Esta función estará disponible en futuras versiones")
        return
    
    print("")
    print_header(f"Calibración de {sensor_name}")
    
    # Select calibration type
    print(f"{Colors.BOLD}Tipo de calibración:{Colors.ENDC}")
    print("  [1] Calibración completa (3 puntos) - Recomendado")
    print("  [2] Calibración rápida (2 puntos)")
    print("  [3] Resetear a valores de fábrica")
    print("  [4] Consultar calibración actual")
    print("  [0] Cancelar")
    print("")
    
    cal_type = input(f"{Colors.OKBLUE}Selecciona opción: {Colors.ENDC}").strip()
    
    if cal_type == '0':
        print_info("Calibración cancelada")
        return
    
    # Handle reset
    if cal_type == '3':
        confirm = input(f"{Colors.WARNING}¿Resetear calibración de {sensor_name}? (S/n): {Colors.ENDC}").strip().lower()
        if confirm == 'n':
            print_info("Operación cancelada")
            return
        
        import json
        cmd_json = json.dumps({
            "sensor": sensor_id,
            "action": "reset"
        })
        
        print_info(f"Reseteando calibración de {sensor_name}...")
        result = publish_calibration_command(cmd_json, device_id=selected_device)
        
        if result:
            print_success(f"✓ Calibración de {sensor_name} reseteada a valores de fábrica")
        else:
            print_error("Falló el reset de calibración")
        return
    
    # Handle get current calibration
    if cal_type == '4':
        import json
        cmd_json = json.dumps({
            "sensor": sensor_id,
            "action": "get"
        })
        
        print_info(f"Consultando calibración actual de {sensor_name}...")
        result = publish_calibration_command(cmd_json, device_id=selected_device)
        
        if result:
            print_success(f"✓ Consulta completada (ver respuesta en /biofloc/calibration_status)")
        else:
            print_error("Falló la consulta")
        return
    
    # Determine number of points
    if cal_type == '1':
        num_points = 3
    elif cal_type == '2':
        num_points = 2
    else:
        print_error("Opción inválida")
        return
    
    print("")
    print_info(f"Calibración de {num_points} puntos para {sensor_name}")
    print("")
    
    # Collect calibration points
    points = []
    
    for i in range(num_points):
        print(f"{Colors.BOLD}═══ Punto {i+1}/{num_points} ═══{Colors.ENDC}")
        
        if sensor_id == 'ph':
            print_info(f"Sugerencia: Solución buffer pH {suggested_values[i]}")
            print_info("1. Enjuaga el sensor con agua destilada")
            print_info(f"2. Sumerge el sensor en la solución buffer pH {suggested_values[i]}")
            print_info("3. Espera 30-60 segundos para estabilización")
        elif sensor_id == 'temperature':
            print_info(f"Sugerencia: Temperatura de referencia {suggested_values[i]}°C")
            print_info("1. Coloca el sensor en el medio de temperatura conocida")
            print_info("2. Espera 30-60 segundos para estabilización")
            print_info("3. Mide con termómetro de referencia (TP101 o similar)")
        
        print("")
        input(f"{Colors.WARNING}Presiona Enter cuando el sensor esté estabilizado...{Colors.ENDC}")
        
        # Read current voltage from sensor (with device filter)
        print_info(f"Leyendo voltaje del sensor de {selected_device}...")
        voltage = read_sensor_voltage(sensor_id, device_id=selected_device)
        
        if voltage is None:
            print_error(f"No se pudo leer voltaje del sensor {sensor_name}")
            print_info("Verifica que:")
            print_info("  - El ESP32 esté conectado y publicando datos")
            print_info("  - El micro-ROS Agent esté activo")
            print_info("  - El topic /biofloc/sensor_data tenga mensajes")
            return
        
        print_success(f"✓ Voltaje leído: {voltage:.3f}V")
        
        # Get reference value
        while True:
            try:
                value_input = input(f"{Colors.OKBLUE}Valor de referencia {sensor_name}: {Colors.ENDC}").strip()
                value = float(value_input)
                break
            except ValueError:
                print_error("Ingresa un número válido")
        
        points.append({"voltage": voltage, "value": value})
        print_success(f"✓ Punto {i+1} registrado: {voltage:.3f}V → {value} {get_sensor_unit(sensor_id)}")
        print("")
    
    # Confirm calibration
    print(f"{Colors.BOLD}═══ Resumen de Calibración ═══{Colors.ENDC}")
    print(f"Sensor: {sensor_name}")
    print(f"Puntos: {num_points}")
    for i, point in enumerate(points):
        print(f"  Punto {i+1}: {point['voltage']:.3f}V → {point['value']} {get_sensor_unit(sensor_id)}")
    print("")
    
    confirm = input(f"{Colors.WARNING}¿Aplicar esta calibración? (S/n): {Colors.ENDC}").strip().lower()
    if confirm == 'n':
        print_info("Calibración cancelada")
        return
    
    # Send calibration command
    import json
    cmd_json = json.dumps({
        "sensor": sensor_id,
        "action": "calibrate",
        "points": points
    })
    
    print_info("Enviando comando de calibración al ESP32...")
    result = publish_calibration_command(cmd_json, device_id=selected_device)
    
    if result:
        print_success(f"✓ Calibración de {sensor_name} completada exitosamente")
        print_info("Los parámetros se han guardado en NVS del ESP32")
        print_info("La calibración persiste entre reinicios")
        print_info("")
        print_info("Próximos pasos:")
        print_info("  - Verifica las lecturas en tiempo real")
        print_info("  - Compara con instrumentos de referencia")
        print_info("  - Re-calibra si es necesario")
    else:
        print_error("Falló la calibración remota")
        print_info("Revisa los logs del ESP32 para más detalles")

def discover_active_devices(duration_seconds=10):
    """
    Discover active ESP32 devices by listening to /biofloc/sensor_data
    
    Args:
        duration_seconds: How long to listen for messages
        
    Returns:
        dict: Dictionary of device_id -> latest_data
    """
    try:
        import json
        import re
        
        print_info(f"Escaneando red para detectar dispositivos activos ({duration_seconds}s)...")
        print_info("Presiona Ctrl+C para terminar antes")
        
        devices = {}
        
        # Use timeout command to capture all messages
        cmd = [
            'bash', '-c',
            f'source /opt/ros/jazzy/setup.bash && '
            f'source ~/microros_ws/install/local_setup.bash && '
            f'timeout {duration_seconds} ros2 topic echo --full-length /biofloc/sensor_data std_msgs/msg/String 2>&1'
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=duration_seconds + 5)
            
            # Parse output: contains multiple "data: '...' \n---" blocks
            output = result.stdout
            
            if not output or 'data:' not in output:
                print_warning("No se recibieron mensajes del tópico")
                print_info("Verifica que:")
                print_info("  - El micro-ROS Agent esté corriendo: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888")
                print_info("  - Los ESP32 estén conectados y publicando")
                return {}
            
            # Split by separator "---" to get individual messages
            messages = output.split('---')
            
            for msg_block in messages:
                if 'data:' not in msg_block:
                    continue
                
                try:
                    # Extract content after "data:" 
                    # Format is: data: '{"device_id": "...", ...}'
                    match = re.search(r"data:\s*['\"](.+?)['\"](?:\n|$)", msg_block, re.DOTALL)
                    
                    if not match:
                        # Fallback: simple split
                        json_str = msg_block.split('data:', 1)[1].strip()
                        # Remove outer quotes
                        if json_str.startswith("'") or json_str.startswith('"'):
                            json_str = json_str[1:]
                        if json_str.endswith("'") or json_str.endswith('"'):
                            json_str = json_str[:-1]
                    else:
                        json_str = match.group(1)
                    
                    # Clean escape characters (ROS may add \\n, \\t, \\r)
                    json_str = json_str.replace('\\n', '').replace('\\r', '').replace('\\t', '').strip()
                    
                    # Parse JSON
                    data = json.loads(json_str)
                    device_id = data.get('device_id', 'unknown')
                    
                    # Store or update device data
                    devices[device_id] = data
                    
                    # Show progress
                    print(f"{Colors.OKGREEN}●{Colors.ENDC} {device_id}", end='\r', flush=True)
                    
                except (json.JSONDecodeError, KeyError, AttributeError) as e:
                    # Skip malformed messages
                    continue
        
        except subprocess.TimeoutExpired:
            print_info("\nTiempo de escaneo completado")
        except KeyboardInterrupt:
            print_info("\nEscaneo interrumpido por el usuario")
        
        print()  # New line after status updates
        
        if devices:
            print_success(f"✓ Detectados {len(devices)} dispositivo(s) activo(s)")
            for device_id in devices:
                print(f"  {Colors.OKCYAN}●{Colors.ENDC} {device_id}")
        else:
            print_warning("No se detectaron dispositivos en la red")
            print_info("Diagnóstico:")
            print_info("  1. Verifica que el tópico esté activo:")
            print_info("     ros2 topic list | grep sensor_data")
            print_info("  2. Verifica publicación manual:")
            print_info("     ros2 topic echo --once /biofloc/sensor_data")
            print_info("  3. Reinicia el micro-ROS Agent si es necesario")
        
        return devices
        
    except Exception as e:
        print_error(f"Error durante escaneo: {e}")
        import traceback
        traceback.print_exc()
        return {}

def monitor_device_health():
    """
    Monitor health telemetry of all active ESP32 devices (live dashboard)
    """
    print_header("Monitor de Salud de Dispositivos")
    
    print_info("Sistema de monitoreo en vivo de ESP32 v3.2.0")
    print_info("Muestra: Device ID, Uptime, Free Heap, Reset Reason, Sensores")
    print()
    
    # Check if micro-ROS Agent is running
    if not check_process_running('micro_ros_agent'):
        print_error("El micro-ROS Agent NO está corriendo")
        print_info("Inicia el Agent primero con la opción [1] del menú")
        return
    
    # Discover active devices
    devices = discover_active_devices(duration_seconds=15)
    
    if not devices:
        print_warning("No hay dispositivos para monitorear")
        return
    
    print()
    print_header("Dashboard de Salud de Dispositivos")
    
    # Print table
    headers = ["Device ID", "Uptime", "Free Heap", "Reset Reason", "pH", "Temp (°C)"]
    print_table_header(headers)
    
    for device_id, data in sorted(devices.items()):
        # Extract system telemetry
        system = data.get('system', {})
        uptime_sec = system.get('uptime_sec', 0)
        free_heap = system.get('free_heap', 0)
        reset_reason = system.get('reset_reason', 'UNKNOWN')
        
        # Format uptime
        if uptime_sec < 60:
            uptime_str = f"{uptime_sec}s"
        elif uptime_sec < 3600:
            uptime_str = f"{uptime_sec//60}min"
        else:
            hours = uptime_sec // 3600
            minutes = (uptime_sec % 3600) // 60
            uptime_str = f"{hours}h{minutes}m"
        
        # Format free heap
        free_heap_kb = free_heap / 1024
        heap_str = f"{free_heap_kb:.1f}KB"
        
        # Extract sensor values
        sensors = data.get('sensors', {})
        ph_value = sensors.get('ph', {}).get('value', 'N/A')
        temp_value = sensors.get('temperature', {}).get('value', 'N/A')
        
        # Format sensor values
        if isinstance(ph_value, (int, float)):
            ph_str = f"{ph_value:.2f}"
        else:
            ph_str = str(ph_value)
        
        if isinstance(temp_value, (int, float)):
            temp_str = f"{temp_value:.1f}"
        else:
            temp_str = str(temp_value)
        
        # Shorten device_id if too long
        device_display = device_id[-12:] if len(device_id) > 15 else device_id
        
        # Print row
        values = [device_display, uptime_str, heap_str, reset_reason[:12], ph_str, temp_str]
        print_table_row(values)
    
    print(f"{Colors.BOLD}{'─' * 100}{Colors.ENDC}")
    print()
    
    # Health analysis
    print_info("Análisis de Salud:")
    for device_id, data in devices.items():
        system = data.get('system', {})
        free_heap = system.get('free_heap', 0)
        reset_reason = system.get('reset_reason', 'UNKNOWN')
        uptime_sec = system.get('uptime_sec', 0)
        
        device_short = device_id[-12:]
        
        # Check for memory issues
        if free_heap < 100000:  # Less than 100 KB
            print_warning(f"  ⚠ {device_short}: Heap bajo ({free_heap/1024:.1f}KB) - Posible memory leak")
        
        # Check for recent resets
        if uptime_sec < 300 and reset_reason in ['TASK_WDT', 'PANIC', 'WDT']:
            print_warning(f"  ⚠ {device_short}: Reinicio reciente por {reset_reason}")
        
        # Check for normal operation
        if free_heap >= 100000 and uptime_sec > 7200:  # > 2 hours
            print_success(f"  ✓ {device_short}: Operación estable (uptime {uptime_sec//3600}h)")
    
    print()

def get_sensor_unit(sensor_id):
    """Get measurement unit for sensor"""
    units = {
        'ph': 'pH',
        'temperature': '°C',
        'dissolved_oxygen': 'mg/L',
        'conductivity': 'μS/cm',
        'turbidity': 'NTU'
    }
    return units.get(sensor_id, '')

def read_sensor_voltage(sensor_id, device_id=None):
    """
    Read sensor voltage with scientific stabilization (v3.6.0)
    
    METHODOLOGY (per lab professor):
    - Biological sensors take 3-7 minutes to stabilize
    - Single instant reading is scientifically INVALID
    - Uses Sliding Window (30 samples) + Statistical Analysis
    - Stability criteria: ≥3 minutes AND std dev <0.005V
    
    ARCHITECTURE: Uses subprocess (ros2 topic echo) instead of rclpy.
    This eliminates DDS port conflicts with micro-ROS Agent.
    
    Args:
        sensor_id: Type of sensor ('ph', 'temperature', etc.)
        device_id: Specific device to read from (None = first message)
    
    Returns:
        float: AVERAGED voltage after stabilization, or None if failed
    """
    try:
        import json as json_lib
        import re
        import statistics
        import threading
        import queue
        from collections import deque
        
        if device_id:
            print_info(f"Leyendo voltaje del sensor de {device_id} con estabilización...")
        else:
            print_info("Leyendo voltaje del sensor con estabilización...")
        
        print_info("")
        print_info("═══════════════════════════════════════════════════════════════")
        print_info("  ALGORITMO DE ESTABILIZACIÓN DE SENSOR BIOLÓGICO")
        print_info("═══════════════════════════════════════════════════════════════")
        print_info("Metodología:")
        print_info("  • Fase warmup: 10 muestras de estabilización (descartadas)")
        print_info("  • Ventana deslizante de 30 muestras reales")
        print_info("  • Cálculo de MEDIANA y desviación estándar en tiempo real")
        print_info("  • Estabilización exitosa: ≥3 min Y σ <0.005V")
        print_info("")
        print_info("Criterios de finalización:")
        print_info("  ✓ AUTOMÁTICO: Cuando sensor esté estable (≥3min, σ<0.005V)")
        print_info("  ⚡ MANUAL: Presiona Ctrl+C para forzar captura del promedio")
        print_info("═══════════════════════════════════════════════════════════════")
        print_info("")
        
        # Sliding window for statistical analysis
        voltage_window = deque(maxlen=30)  # Last 30 samples for median calculation
        warmup_samples = []  # First 10 samples discarded (stabilization phase)
        start_time = time.time()
        sample_count = 0
        warmup_count = 0
        stable_readings = 0
        min_warmup_samples = 10  # Discard first 10 samples (sensor stabilization)
        min_stable_time_sec = 180  # 3 minutes
        max_std_dev = 0.005  # 0.005V threshold
        
        # Message queue for background reading
        msg_queue = queue.Queue()
        stop_event = threading.Event()
        subprocess_proc = None  # CRITICAL: Keep reference for cleanup
        
        def read_messages():
            """Background thread to continuously read ROS messages"""
            nonlocal subprocess_proc  # Access parent scope
            
            cmd = [
                'bash', '-c',
                f'source /opt/ros/jazzy/setup.bash && '
                f'source ~/microros_ws/install/local_setup.bash && '
                f'ros2 topic echo --full-length /biofloc/sensor_data std_msgs/msg/String 2>&1'
            ]
            
            try:
                subprocess_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, 
                                                   stderr=subprocess.PIPE, text=True)
                
                buffer = ""
                while not stop_event.is_set():
                    try:
                        line = subprocess_proc.stdout.readline()
                        if not line:
                            break
                        buffer += line
                        
                        # Check for complete message (ends with ---)
                        if '---' in buffer:
                            msg_queue.put(buffer)
                            buffer = ""
                    except Exception:
                        break
            finally:
                # CRITICAL: Always cleanup subprocess
                if subprocess_proc:
                    try:
                        subprocess_proc.kill()
                        subprocess_proc.wait(timeout=2)
                    except Exception:
                        pass
        
        # Start background reader thread
        reader_thread = threading.Thread(target=read_messages, daemon=True)
        reader_thread.start()
        
        # Wait for first message to confirm topic is active
        try:
            first_msg = msg_queue.get(timeout=10)
        except queue.Empty:
            stop_event.set()
            print_error("Timeout: No se recibieron mensajes en 10s")
            print_info("Verifica que:")
            print_info("  - El ESP32 esté conectado y publicando datos")
            print_info("  - El micro-ROS Agent esté activo")
            return None
        
        # Process first message
        try:
            match = re.search(r"data:\s*['\"](.+?)['\"](?:\n|$)", first_msg, re.DOTALL)
            if match:
                json_str = match.group(1).replace('\\n', '').replace('\\r', '').replace('\\t', '').strip()
                data = json_lib.loads(json_str)
                msg_device_id = data.get('device_id', 'unknown')
                
                if not device_id or msg_device_id == device_id:
                    sensors_data = data.get('sensors', {})
                    sensor_data = sensors_data.get(sensor_id, {})
                    voltage = sensor_data.get('voltage')
                    
                    if voltage is not None:
                        # Warmup phase: discard first 10 samples
                        if warmup_count < min_warmup_samples:
                            warmup_samples.append(voltage)
                            warmup_count += 1
                        else:
                            voltage_window.append(voltage)
                            sample_count += 1
        except Exception:
            pass
        
        if warmup_count > 0:
            print_info(f"✓ Fase warmup completada ({warmup_count} muestras descartadas)")
        print_info("✓ Sensor detectado, iniciando monitoreo en tiempo real...")
        print_info("")
        print(f"{Colors.BOLD}{'─' * 110}{Colors.ENDC}")
        print(f"{Colors.BOLD}{'Tiempo':>8} | {'Voltaje':>10} | {'Mediana':>10} | {'σ (Std Dev)':>12} | {'Muestras':>10} | {'Estado':>30}{Colors.ENDC}")
        print(f"{Colors.BOLD}{'─' * 110}{Colors.ENDC}")
        
        try:
            while True:
                # Get next message with short timeout
                try:
                    msg_block = msg_queue.get(timeout=1)
                except queue.Empty:
                    # No new message, just update display
                    elapsed = time.time() - start_time
                    if len(voltage_window) > 0:
                        current_v = voltage_window[-1]
                        median_v = statistics.median(voltage_window)
                        std_dev = statistics.stdev(voltage_window) if len(voltage_window) > 1 else 0.0
                        
                        # Check stability
                        is_stable = (elapsed >= min_stable_time_sec and std_dev < max_std_dev)
                        
                        if is_stable:
                            stable_readings += 1
                            status = f"{Colors.OKGREEN}ESTABLE{Colors.ENDC} (✓ {stable_readings})"
                        elif elapsed >= min_stable_time_sec:
                            status = f"{Colors.WARNING}Esperando σ<{max_std_dev}V{Colors.ENDC}"
                        else:
                            remain_sec = int(min_stable_time_sec - elapsed)
                            status = f"{Colors.OKCYAN}Estabilizando... ({remain_sec}s){Colors.ENDC}"
                        
                        print(f"\r{int(elapsed):>7}s | {current_v:>9.4f}V | {median_v:>9.4f}V | {std_dev:>11.6f}V | {len(voltage_window):>10} | {status:<40}", end='', flush=True)
                        
                        # Auto-finish if stable for 3 consecutive readings
                        if stable_readings >= 3:
                            print()  # New line
                            print(f"{Colors.BOLD}{'─' * 110}{Colors.ENDC}")
                            print_success(f"✓ ESTABILIZACIÓN COMPLETADA AUTOMÁTICAMENTE")
                            print_info(f"  Voltaje mediana: {median_v:.4f}V (más robusto que promedio)")
                            print_info(f"  Desviación estándar: {std_dev:.6f}V")
                            print_info(f"  Tiempo total: {int(elapsed)}s ({int(elapsed/60)}:{int(elapsed%60):02d})")
                            print_info(f"  Muestras procesadas: {len(voltage_window)}")
                            stop_event.set()
                            return median_v
                    
                    continue
                
                # Parse new message
                try:
                    match = re.search(r"data:\s*['\"](.+?)['\"](?:\n|$)", msg_block, re.DOTALL)
                    if not match:
                        continue
                    
                    json_str = match.group(1).replace('\\n', '').replace('\\r', '').replace('\\t', '').strip()
                    data = json_lib.loads(json_str)
                    msg_device_id = data.get('device_id', 'unknown')
                    
                    # Filter by device_id if specified
                    if device_id and msg_device_id != device_id:
                        continue
                    
                    # Extract voltage
                    sensors_data = data.get('sensors', {})
                    sensor_data = sensors_data.get(sensor_id, {})
                    voltage = sensor_data.get('voltage')
                    
                    if voltage is None:
                        continue
                    
                    # Warmup phase: discard first 10 samples
                    if warmup_count < min_warmup_samples:
                        warmup_samples.append(voltage)
                        warmup_count += 1
                        continue
                    
                    # Add to sliding window
                    voltage_window.append(voltage)
                    sample_count += 1
                    stable_readings = 0  # Reset stable counter on new reading
                    
                except (json_lib.JSONDecodeError, KeyError, AttributeError):
                    continue
        
        except KeyboardInterrupt:
            print()  # New line
            print(f"{Colors.BOLD}{'─' * 110}{Colors.ENDC}")
            print_warning("⚡ Captura MANUAL forzada por usuario (Ctrl+C)")
            
            if len(voltage_window) == 0:
                print_error("No hay muestras disponibles")
                stop_event.set()
                return None
            
            median_v = statistics.median(voltage_window)
            std_dev = statistics.stdev(voltage_window) if len(voltage_window) > 1 else 0.0
            elapsed = time.time() - start_time
            
            print_info(f"  Voltaje mediana: {median_v:.4f}V (más robusto que promedio)")
            print_info(f"  Desviación estándar: {std_dev:.6f}V")
            print_info(f"  Tiempo transcurrido: {int(elapsed)}s ({int(elapsed/60)}:{int(elapsed%60):02d})")
            print_info(f"  Muestras warmup descartadas: {warmup_count}")
            print_info(f"  Muestras procesadas: {len(voltage_window)}")
            
            if std_dev >= max_std_dev:
                print_warning(f"  ⚠ Desviación alta (σ={std_dev:.6f}V > {max_std_dev}V)")
                print_warning(f"  Recomendación: Esperar más tiempo para mejor precisión")
            
            if elapsed < min_stable_time_sec:
                print_warning(f"  ⚠ Tiempo insuficiente ({int(elapsed)}s < {min_stable_time_sec}s)")
                print_warning(f"  Recomendación: Idealmente esperar ≥3 minutos")
            
            confirm = input(f"\n{Colors.WARNING}¿Usar esta mediana de todas formas? (s/N): {Colors.ENDC}").strip().lower()
            stop_event.set()
            
            if confirm == 's':
                print_success(f"✓ Usando voltaje mediana: {median_v:.4f}V")
                return median_v
            else:
                print_info("Medición cancelada")
                return None
    
    except Exception as e:
        print_error(f"Error leyendo voltaje: {e}")
        import traceback
        traceback.print_exc()
        return None
    finally:
        # CRITICAL CLEANUP: Prevent zombie subprocess and thread leaks
        stop_event.set()
        
        # Kill subprocess if still running
        if 'subprocess_proc' in locals() and subprocess_proc:
            try:
                subprocess_proc.kill()
                subprocess_proc.wait(timeout=2)
            except Exception:
                pass
        
        # Wait for thread to finish (max 3 seconds)
        if 'reader_thread' in locals() and reader_thread.is_alive():
            reader_thread.join(timeout=3)

def save_calibration_to_mongodb(calibration_request, calibration_response, device_id=None):
    """
    Save calibration data to MongoDB devices collection (Digital Twin)
    ONLY called after ESP32 confirms successful NVS save via ACK.
    
    Args:
        calibration_request: Original calibration command (dict)
        calibration_response: Response from ESP32 with calculated parameters (dict)
        device_id: ESP32 device ID string (e.g. 'biofloc_esp32_c8e0')
    
    Returns:
        bool: True if saved successfully, False otherwise
    """
    try:
        from pymongo import MongoClient
        from pymongo.errors import ConnectionFailure, OperationFailure
        
        # Get MongoDB credentials from .env
        mongodb_uri = os.getenv('MONGODB_URI')
        mongodb_database = os.getenv('MONGODB_DATABASE', 'SistemasLab')
        mongodb_devices_collection = os.getenv('MONGODB_COLLECTION_DEVICES', 'devices')
        
        if not mongodb_uri:
            print_warning("MONGODB_URI no configurado - calibración solo guardada en ESP32")
            return False
        
        if not device_id:
            print_warning("Device ID no proporcionado - no se puede guardar en MongoDB")
            return False
        
        # Connect to MongoDB
        client = MongoClient(mongodb_uri, serverSelectionTimeoutMS=5000)
        db = client[mongodb_database]
        devices_col = db[mongodb_devices_collection]
        
        # Verify connection
        client.admin.command('ping')
        
        # Prepare calibration document
        sensor_type = calibration_request.get('sensor')
        calibration_doc = {
            'fecha': datetime.utcnow().isoformat() + 'Z',
            'slope': calibration_response.get('slope'),
            'offset': calibration_response.get('offset'),
            'r_squared': calibration_response.get('r_squared'),
            'points': calibration_request.get('points', []),
            'num_points': len(calibration_request.get('points', [])),
            'status': calibration_response.get('status'),
            'message': calibration_response.get('message')
        }
        
        # Update device document
        result = devices_col.update_one(
            {'_id': device_id},
            {
                '$set': {
                    f'calibracion.{sensor_type}': calibration_doc,
                    'ultima_calibracion': datetime.utcnow().isoformat() + 'Z'
                }
            },
            upsert=False  # Don't create if device doesn't exist
        )
        
        if result.modified_count > 0 or result.matched_count > 0:
            print_success(f"✓ Calibración guardada en MongoDB (Digital Twin)")
            print_info(f"  Device: {device_id}")
            print_info(f"  Sensor: {sensor_type}")
            print_info(f"  R²: {calibration_doc['r_squared']:.4f}")
            return True
        else:
            print_warning(f"Dispositivo {device_id} no encontrado en MongoDB")
            return False
            
    except ConnectionFailure:
        print_error("✗ No se pudo conectar a MongoDB Atlas")
        return False
    except OperationFailure as e:
        print_error(f"✗ Error de MongoDB: {e}")
        return False
    except ImportError:
        print_warning("pymongo no instalado - ejecuta: pip install pymongo")
        return False
    except Exception as e:
        print_error(f"Error guardando calibración en MongoDB: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        try:
            client.close()
        except:
            pass

def publish_calibration_command(json_cmd, device_id=None):
    """
    Publish calibration command and wait for ESP32 ACK (30s timeout)
    
    ARCHITECTURE: Start ACK listener BEFORE sending command to avoid race condition.
    Uses subprocess for both send and receive — zero rclpy usage.
    MongoDB save ONLY happens after confirmed ACK with status=success.
    
    Args:
        json_cmd: JSON string with calibration command
        device_id: Device ID string for ACK verification and MongoDB save
    
    Returns:
        bool: True ONLY if ESP32 confirmed success via ACK
    """
    ack_process = None
    
    try:
        import json as json_lib
        import re
        
        cal_data = json_lib.loads(json_cmd)
        
        # ── Step 1: Start ACK listener BEFORE sending command ──
        # This prevents the race condition where the ESP32 responds
        # before we start listening, causing a "missed ACK" timeout.
        print_info("Preparando listener de ACK en /biofloc/calibration_status...")
        
        ack_cmd = [
            'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && '
            'source ~/microros_ws/install/local_setup.bash && '
            'timeout 30 ros2 topic echo --once --full-length '
            '/biofloc/calibration_status std_msgs/msg/String 2>&1'
        ]
        
        ack_process = subprocess.Popen(
            ack_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        
        # Give the DDS subscriber time to discover and connect
        time.sleep(2)
        
        # ── Step 2: Publish calibration command ──
        # CRITICAL FIX v3.6.2: Escape double quotes for shell, preserve JSON structure
        # Old method used single quotes which caused ROS2 to parse as YAML, stripping
        # all double quotes from JSON keys/values, breaking cJSON parsing in ESP32
        json_escaped = json_cmd.replace('"', '\\"')
        
        pub_cmd = [
            'bash', '-c',
            f'source /opt/ros/jazzy/setup.bash && '
            f'source ~/microros_ws/install/local_setup.bash && '
            f'ros2 topic pub --once /biofloc/calibration_cmd '
            f'std_msgs/msg/String "data: \\\"{json_escaped}\\\""'
        ]
        
        print_info(f"Enviando comando al ESP32: {json_cmd[:100]}{'...' if len(json_cmd) > 100 else ''}")
        
        pub_result = subprocess.run(pub_cmd, capture_output=True, text=True, timeout=15)
        
        if pub_result.returncode != 0:
            print_error(f"Error publicando comando: {pub_result.stderr}")
            ack_process.kill()
            ack_process.wait()
            return False
        
        print_info("✓ Comando enviado, esperando ACK del ESP32 (timeout: 30s)...")
        
        # ── Step 3: Wait for ACK response ──
        try:
            ack_stdout, ack_stderr = ack_process.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            ack_process.kill()
            ack_process.wait()
            print_error("✗ TIMEOUT: No se recibió ACK del ESP32 en 30 segundos")
            print_error("  El ESP32 pudo haber sufrido un PANIC y reiniciado")
            print_info("  Verifica los logs del ESP32 con: idf.py -p /dev/ttyUSB0 monitor")
            print_info("  La calibración NO se guardó (ni en NVS ni en MongoDB)")
            return False
        
        if ack_process.returncode != 0 or 'data:' not in (ack_stdout or ''):
            print_error("✗ No se recibió respuesta válida del ESP32")
            print_info("  Posibles causas:")
            print_info("  - ESP32 sufrió PANIC al procesar el comando")
            print_info("  - Timeout en la conexión micro-ROS")
            print_info("  - Agent desconectado del ESP32")
            return False
        
        # ── Step 4: Parse and validate ACK ──
        try:
            # Extract JSON from ros2 topic echo output
            response_data = ack_stdout.split('data:', 1)[1].strip()
            # Remove outer quotes
            if response_data.startswith("'") or response_data.startswith('"'):
                response_data = response_data[1:]
            if response_data.endswith("'") or response_data.endswith('"'):
                response_data = response_data[:-1]
            # Clean escape chars
            response_data = response_data.replace('\\n', '').replace('\\r', '').replace('\\t', '').strip()
            
            response_json = json_lib.loads(response_data)
        except (json_lib.JSONDecodeError, IndexError, AttributeError) as e:
            print_warning(f"ACK recibido pero JSON inválido: {e}")
            print_info(f"  Raw: {ack_stdout[:200]}")
            return False
        
        print_info(f"ACK recibido: {json_lib.dumps(response_json, ensure_ascii=False)[:200]}")
        
        # ── Step 5: Verify device_id matches ──
        ack_device = response_json.get('device_id', '')
        if device_id and ack_device and ack_device != device_id:
            print_error(f"✗ ACK de dispositivo incorrecto: {ack_device} (esperaba {device_id})")
            return False
        
        # ── Step 6: Check status ──
        if response_json.get('status') == 'success':
            print_success(f"✓ ESP32 confirmó calibración exitosa")
            if 'slope' in response_json:
                print_info(f"  Slope: {response_json['slope']:.6f}")
            if 'offset' in response_json:
                print_info(f"  Offset: {response_json['offset']:.6f}")
            if 'r_squared' in response_json:
                print_info(f"  R²: {response_json['r_squared']:.4f}")
            
            # ── Step 7: ONLY save to MongoDB after CONFIRMED success ACK ──
            save_calibration_to_mongodb(cal_data, response_json, device_id=device_id)
            
            return True
        else:
            error_msg = response_json.get('message', 'Error desconocido')
            print_error(f"✗ ESP32 reportó error: {error_msg}")
            return False
    
    except subprocess.TimeoutExpired:
        print_error("✗ TIMEOUT publicando comando ROS 2")
        return False
    except Exception as e:
        print_error(f"Error en calibración remota: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Cleanup: ensure subprocess is always terminated
        if ack_process and ack_process.poll() is None:
            try:
                ack_process.kill()
                ack_process.wait(timeout=3)
            except Exception:
                pass

def update_calibration(sensor_type, **kwargs):
    """
    Generic function to update calibration for any sensor
    
    Args:
        sensor_type: 'ph' or 'temperature'
        **kwargs: calibration parameters (slope, offset, etc.)
    """
    try:
        if sensor_type == 'ph':
            slope = kwargs.get('slope')
            offset = kwargs.get('offset')
            
            if slope is None or offset is None:
                print_error("pH calibration requires slope and offset")
                return False
            
            # Update sdkconfig.defaults
            config_file = PROJECT_ROOT / "sdkconfig.defaults"
            with open(config_file, 'r') as f:
                content = f.read()
            
            # Add pH calibration settings if not present
            import re
            if 'CONFIG_BIOFLOC_PH_SLOPE' not in content:
                # Append at end before temperature section
                temp_section = content.find('# ---- Temperature Sensor')
                if temp_section != -1:
                    ph_config = f"\n# ---- pH Sensor Calibration ----\n"
                    ph_config += f"CONFIG_BIOFLOC_PH_SLOPE={int(slope * 1000000)}\n"
                    ph_config += f"CONFIG_BIOFLOC_PH_OFFSET={int(offset * 1000)}\n\n"
                    content = content[:temp_section] + ph_config + content[temp_section:]
                else:
                    content += f"\n# ---- pH Sensor Calibration ----\n"
                    content += f"CONFIG_BIOFLOC_PH_SLOPE={int(slope * 1000000)}\n"
                    content += f"CONFIG_BIOFLOC_PH_OFFSET={int(offset * 1000)}\n"
            else:
                content = re.sub(
                    r'CONFIG_BIOFLOC_PH_SLOPE=\d+',
                    f'CONFIG_BIOFLOC_PH_SLOPE={int(slope * 1000000)}',
                    content
                )
                content = re.sub(
                    r'CONFIG_BIOFLOC_PH_OFFSET=-?\d+',
                    f'CONFIG_BIOFLOC_PH_OFFSET={int(offset * 1000)}',
                    content
                )
            
            with open(config_file, 'w') as f:
                f.write(content)
            
            print_success(f"Updated pH calibration: slope={slope:.6f}, offset={offset:.3f}")
            
        elif sensor_type == 'temperature':
            slope = kwargs.get('slope', 1000000)  # Default 1.0
            offset = kwargs.get('offset')
            
            if offset is None:
                print_error("Temperature calibration requires offset")
                return False
            
            # Update sdkconfig.defaults
            config_file = PROJECT_ROOT / "sdkconfig.defaults"
            with open(config_file, 'r') as f:
                content = f.read()
            
            import re
            content = re.sub(
                r'CONFIG_BIOFLOC_TEMP_SLOPE=\d+',
                f'CONFIG_BIOFLOC_TEMP_SLOPE={slope}',
                content
            )
            content = re.sub(
                r'CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES=-?\d+',
                f'CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES={offset}',
                content
            )
            
            with open(config_file, 'w') as f:
                f.write(content)
            
            print_success(f"Updated temperature calibration: offset={offset/1000:.3f}°C")
        
        else:
            print_error(f"Unknown sensor type: {sensor_type}")
            return False
        
        # Update Kconfig.projbuild defaults
        kconfig_file = PROJECT_ROOT / "main" / "Kconfig.projbuild"
        with open(kconfig_file, 'r') as f:
            content = f.read()
        
        import re
        if sensor_type == 'ph':
            # pH doesn't have calibration in Kconfig yet, would need to add it
            pass
        elif sensor_type == 'temperature':
            content = re.sub(
                r'(config BIOFLOC_TEMP_SLOPE\s+int[^\n]+\s+default\s+)\d+',
                f'\\g<1>{slope}',
                content
            )
            content = re.sub(
                r'(config BIOFLOC_TEMP_OFFSET_MILLIDEGREES\s+int[^\n]+\s+default\s+)-?\d+',
                f'\\g<1>{offset}',
                content
            )
            
            with open(kconfig_file, 'w') as f:
                f.write(content)
        
        print_success("Updated Kconfig.projbuild defaults")
        
        # Offer to regenerate and rebuild
        choice = input("\nRegenerate sdkconfig now? (Y/n): ").strip().lower()
        if choice != 'n':
            regenerate_sdkconfig()
            
            choice = input("\nBuild and flash firmware now? (Y/n): ").strip().lower()
            if choice != 'n':
                build_and_flash()
        
        return True
        
    except Exception as e:
        print_error(f"Failed to update calibration: {e}")
        return False

def quick_adjust_temperature():
    """Quick temperature offset adjustment without full calibration"""
    print_header("Ajuste Rápido de Temperatura")
    
    print_info("Offset actual: +1.382°C (+1382 miligrados)")
    print_info("Basado en tu última medición:")
    print("  ESP32 lee: 23.8°C")
    print("  Temp real: 22.2°C")
    print("  Error: +1.6°C")
    print()
    
    print_warning("Esto establecerá el offset a -1.6°C sin calibración completa")
    choice = input("¿Continuar? (s/N): ").strip().lower()
    
    if choice != 's':
        print_info("Cancelado")
        return
    
    # Update with -1.6°C offset
    update_calibration('temperature', slope=1000000, offset=-1600)


def configure_wifi():
    """Configure WiFi credentials"""
    print_header("Configuración WiFi")
    
    print_info("Configuración actual:")
    print(f"  SSID Gateway: <configurado en .env>")
    print(f"  IP Gateway: 10.42.0.1")
    print()
    
    print_warning("Esto modificará sdkconfig.defaults")
    choice = input("¿Deseas cambiar la configuración WiFi? (s/N): ").strip().lower()
    
    if choice != 's':
        print_info("No se realizaron cambios")
        return
    
    ssid = input("Ingresa el nuevo SSID: ").strip()
    password = input("Ingresa la nueva contraseña: ").strip()
    
    if not ssid:
        print_error("El SSID no puede estar vacío")
        return
    
    # Update sdkconfig.defaults
    config_file = PROJECT_ROOT / "sdkconfig.defaults"
    
    try:
        with open(config_file, 'r') as f:
            content = f.read()
        
        # Replace WiFi settings (both CONFIG_ESP_WIFI_* and CONFIG_BIOFLOC_WIFI_*)
        import re
        
        # Update ESP WiFi credentials (used by micro_ros component)
        content = re.sub(
            r'CONFIG_ESP_WIFI_SSID="[^"]*"',
            f'CONFIG_ESP_WIFI_SSID="{ssid}"',
            content
        )
        content = re.sub(
            r'CONFIG_ESP_WIFI_PASSWORD="[^"]*"',
            f'CONFIG_ESP_WIFI_PASSWORD="{password}"',
            content
        )
        
        # Update Biofloc WiFi credentials (used by main application)
        content = re.sub(
            r'CONFIG_BIOFLOC_WIFI_SSID="[^"]*"',
            f'CONFIG_BIOFLOC_WIFI_SSID="{ssid}"',
            content
        )
        content = re.sub(
            r'CONFIG_BIOFLOC_WIFI_PASSWORD="[^"]*"',
            f'CONFIG_BIOFLOC_WIFI_PASSWORD="{password}"',
            content
        )
        
        with open(config_file, 'w') as f:
            f.write(content)
        
        print_success(f"Credenciales WiFi actualizadas en sdkconfig.defaults (ambos conjuntos)")
        
        # Offer to regenerate and rebuild
        choice = input("\n¿Regenerar sdkconfig ahora? (S/n): ").strip().lower()
        if choice != 'n':
            regenerate_sdkconfig()
            
            choice = input("\n¿Compilar y flashear firmware ahora? (S/n): ").strip().lower()
            if choice != 'n':
                build_and_flash()
        
    except Exception as e:
        print_error(f"Falló la actualización de configuración: {e}")

def build_and_flash():
    """Build and flash firmware to ESP32"""
    print_header("Compilar y Flashear Firmware")
    
    # Check if ESP32 is connected
    if not Path("/dev/ttyUSB0").exists():
        print_error("ESP32 no detectado en /dev/ttyUSB0")
        print_info("Conecta el ESP32 via USB e intenta nuevamente")
        return
    
    print_success("ESP32 detectado en /dev/ttyUSB0")
    
    print_info("Esto hará:")
    print("  1. Cargar entorno ESP-IDF")
    print("  2. Compilar firmware")
    print("  3. Flashear al ESP32")
    print()
    
    choice = input("¿Continuar? (S/n): ").strip().lower()
    if choice == 'n':
        print_info("Cancelado")
        return
    
    # Use bash explicitly for source command
    cmd = [
        'bash', '-c',
        f'source {ESP_IDF_PATH}/export.sh && '
        f'cd {PROJECT_ROOT} && '
        f'idf.py build && '
        f'idf.py -p /dev/ttyUSB0 flash'
    ]
    
    try:
        result = subprocess.run(cmd, check=False, text=True)
        if result.returncode == 0:
            print_success("Firmware compilado y flasheado exitosamente")
            print_info("El ESP32 se reiniciará automáticamente")
        else:
            print_error("Falló la compilación o el flasheo")
    except Exception as e:
        print_error(f"Error: {e}")

def regenerate_sdkconfig():
    """Regenerate sdkconfig from defaults"""
    print_header("Regenerar sdkconfig")
    
    print_warning("Esto ELIMINARÁ el sdkconfig actual y lo regenerará desde sdkconfig.defaults")
    print_info("Usa esto después de cambiar credenciales WiFi o valores de calibración")
    print()
    
    choice = input("¿Estás seguro? (s/N): ").strip().lower()
    if choice != 's':
        print_info("Cancelado")
        return False
    
    # Use bash explicitly for source command
    cmd = [
        'bash', '-c',
        f'cd {PROJECT_ROOT} && '
        f'rm -f sdkconfig && '
        f'source {ESP_IDF_PATH}/export.sh && '
        f'idf.py reconfigure'
    ]
    
    try:
        result = subprocess.run(cmd, check=False, text=True)
        if result.returncode == 0:
            print_success("sdkconfig regenerado desde defaults")
            return True
        else:
            print_error("Falló la regeneración")
            return False
    except Exception as e:
        print_error(f"Error: {e}")
        return False
        
        # Offer to build immediately
        choice = input("\n¿Compilar y flashear firmware ahora? (S/n): ").strip().lower()
        if choice != 'n':
            build_and_flash()
        return True
    else:
        print_error("Failed to regenerate sdkconfig")
        return False

def monitor_sensors():
    """Monitor sensor readings in real-time"""
    print_header("Monitor de Sensores")
    
    cmd = f"cd {SCRIPTS_DIR} && python3 monitor_sensores.py"
    
    print_info("Monitoreo de sensores en tiempo real (sin escritura a base de datos)")
    print_info("Presiona Ctrl+C para detener")
    
    run_command(cmd, check=False)

def check_system_status():
    """Check overall system status"""
    print_header("Verificación de Estado del Sistema")
    
    # Check network configuration
    gateway_ip, ssid = detect_gateway_info()
    print_info(f"Red WiFi: {ssid}")
    print_info(f"IP Gateway: {gateway_ip}")
    print()
    
    # Check Agent
    if check_process_running("micro_ros_agent"):
        print_success("micro-ROS Agent: EJECUTÁNDOSE")
    else:
        print_warning("micro-ROS Agent: NO SE ESTÁ EJECUTANDO")
    
    # Check Bridge
    if check_process_running("sensor_db_bridge"):
        print_success("Sensor DB Bridge: EJECUTÁNDOSE")
    else:
        print_warning("Sensor DB Bridge: NO SE ESTÁ EJECUTANDO")
    
    # Check ESP32
    if Path("/dev/ttyUSB0").exists():
        print_success("ESP32: CONECTADO (USB)")
    else:
        print_info("ESP32: No detectado en USB (puede estar ejecutándose de forma autónoma)")
    
    # Check Gateway WiFi by checking wlan0 interface
    result = subprocess.run(
        ["ip", "addr", "show", "wlan0"],
        capture_output=True,
        text=True
    )
    
    if result.returncode == 0 and gateway_ip in result.stdout:
        print_success(f"Gateway WiFi: ACTIVO ({gateway_ip})")
    else:
        print_warning("Gateway WiFi: NO ACTIVO")
    
    # Check ROS topics - get full list and filter in Python (grep pipe fails in subprocess)
    cmd = f"bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic list 2>/dev/null'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if "/biofloc/sensor_data" in result.stdout:
        print_success("Topic ROS: /biofloc/sensor_data DISPONIBLE")
        
        # Check if messages are being published (ESP32 publishes every 4s, wait max 12s)
        print_info("Verificando mensajes (esperando hasta 12s)...")
        cmd_echo = f"timeout 12 bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic echo /biofloc/sensor_data --once --full-length 2>&1'"
        result_echo = subprocess.run(cmd_echo, shell=True, capture_output=True, text=True)
        
        if result_echo.returncode == 0 and result_echo.stdout.strip():
            print_success("ESP32 está publicando datos")
            
            # Ask user if they want to calculate rate (takes time)
            print()
            try:
                choice = input("¿Calcular tasa de publicación? (toma ~20s) (S/n): ").strip().lower()
                if choice != 'n':
                    print_info("Calculando tasa de publicación...")
                    cmd_hz = f"timeout 20 bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic hz /biofloc/sensor_data 2>&1'"
                    result_hz = subprocess.run(cmd_hz, shell=True, capture_output=True, text=True)
                    
                    if "average rate" in result_hz.stdout:
                        import re
                        rate_match = re.search(r'average rate: ([0-9.]+)', result_hz.stdout)
                        if rate_match:
                            rate = rate_match.group(1)
                            interval = 1/float(rate)
                            print_info(f"Tasa de publicación: {rate} Hz (~{interval:.1f}s de intervalo)")
                    else:
                        print_warning("No se pudo calcular la tasa en 20s")
            except (KeyboardInterrupt, EOFError):
                print_info("\nCálculo de tasa omitido")
        else:
            print_warning("No se recibieron mensajes en 12s (ESP32 publica cada 4s)")
            print_info("Verifica que el ESP32 esté encendido y conectado al gateway WiFi")
    else:
        print_warning("Topic ROS: /biofloc/sensor_data NO ENCONTRADO")

def check_esp32_connectivity():
    """Check ESP32 network connectivity and IP address"""
    print_header("Verificación de Conectividad ESP32")
    
    # Detect gateway info
    gateway_ip, ssid = detect_gateway_info()
    print_info(f"Red WiFi: {ssid}")
    print_info(f"IP Gateway: {gateway_ip}")
    print_info(f"Buscando ESP32 con MAC: {ESP32_MAC}")
    print()
    
    # Check if gateway WiFi is active
    result = subprocess.run(
        ["ip", "addr", "show", "wlan0"],
        capture_output=True,
        text=True
    )
    
    if result.returncode != 0 or gateway_ip not in result.stdout:
        print_error("Gateway WiFi NO ACTIVO en wlan0")
        print_info("Inicia el hotspot gateway primero")
        return
    
    print_success(f"Gateway WiFi: ACTIVO ({gateway_ip})")
    
    # Check DHCP leases for ESP32
    print_info("Verificando leases DHCP...")
    
    # Try dnsmasq lease file
    lease_files = [
        "/var/lib/misc/dnsmasq.leases",
        "/var/lib/dnsmasq/dnsmasq.leases",
        "/tmp/dnsmasq.leases"
    ]
    
    esp32_ip = None
    for lease_file in lease_files:
        if Path(lease_file).exists():
            try:
                with open(lease_file, 'r') as f:
                    content = f.read()
                # Filter in Python (grep pipes fail in subprocess)
                for line in content.splitlines():
                    if ESP32_MAC.lower() in line.lower():
                        # Parse lease file: timestamp mac ip hostname client-id
                        parts = line.strip().split()
                        if len(parts) >= 3:
                            esp32_ip = parts[2]
                            print_success(f"ESP32 encontrado en leases DHCP: {esp32_ip}")
                            break
                if esp32_ip:
                    break
            except:
                pass
    
    if not esp32_ip:
        # Try ARP/neighbor table (modern command)
        print_info("Verificando tabla de vecinos...")
        result = subprocess.run(
            "ip neigh",
            shell=True,
            capture_output=True,
            text=True
        )
        
        # Filter in Python (grep pipes fail in subprocess)
        for line in result.stdout.splitlines():
            if ESP32_MAC.lower() in line.lower():
                parts = line.strip().split()
                if len(parts) >= 1:
                    esp32_ip = parts[0]
                    print_success(f"ESP32 encontrado en tabla de vecinos: {esp32_ip}")
                    break
    
    if not esp32_ip:
        # Scan network range
        print_info("Escaneando red 10.42.0.0/24...")
        print_warning("Esto puede tomar 30-60 segundos...")
        
        result = subprocess.run(
            "nmap -sn 10.42.0.0/24 2>/dev/null | grep -B 2 'XX:XX:XX:XX:XX:XX' | grep 'Nmap scan' | awk '{print $5}'",
            shell=True,
            capture_output=True,
            text=True
        )
        
        if result.stdout:
            esp32_ip = result.stdout.strip()
            print_success(f"ESP32 encontrado via escaneo de red: {esp32_ip}")
    
    if esp32_ip:
        print()
        print_info(f"Dirección IP del ESP32: {Colors.BOLD}{esp32_ip}{Colors.ENDC}")
        
        # Ping test
        print_info("Probando conectividad...")
        result = subprocess.run(
            f"ping -c 3 -W 1 {esp32_ip}",
            shell=True,
            capture_output=True,
            text=True
        )
        
        if result.returncode == 0:
            print_success(f"ESP32 está ALCANZABLE (ping exitoso)")
        else:
            print_warning(f"ESP32 no responde al ping")
        
        # Check if publishing to ROS topic
        print_info("Verificando comunicación ROS (esperando mensaje hasta 12s)...")
        
        # First check if messages are being published (ESP32 publishes every 4s)
        cmd_echo = f"timeout 12 bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic echo /biofloc/sensor_data --once --full-length 2>&1'"
        result_echo = subprocess.run(cmd_echo, shell=True, capture_output=True, text=True)
        
        if result_echo.returncode == 0 and result_echo.stdout.strip():
            print_success("ESP32 está PUBLICANDO al topic ROS")
            
            # Ask user if they want to calculate rate
            print()
            try:
                choice = input("¿Calcular tasa de publicación? (toma ~20s) (S/n): ").strip().lower()
                if choice != 'n':
                    print_info("Calculando tasa de publicación...")
                    cmd_hz = f"timeout 20 bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic hz /biofloc/sensor_data 2>&1'"
                    result_hz = subprocess.run(cmd_hz, shell=True, capture_output=True, text=True)
                    
                    if "average rate" in result_hz.stdout:
                        import re
                        rate_match = re.search(r'average rate: ([0-9.]+)', result_hz.stdout)
                        if rate_match:
                            rate = rate_match.group(1)
                            interval = 1/float(rate)
                            print_info(f"Tasa de publicación: {rate} Hz (~{interval:.1f}s de intervalo)")
                    else:
                        print_warning("No se pudo calcular la tasa en 20s")
            except (KeyboardInterrupt, EOFError):
                print_info("\nCálculo de tasa omitido")
        else:
            print_warning("No se recibieron mensajes en 12s (ESP32 publica cada 4s)")
            print_info("Verifica que el ESP32 esté encendido y conectado al gateway WiFi")

        
        # Summary
        print()
        print_header("Resumen de Conexión")
        print(f"{Colors.OKGREEN}Estado del ESP32: CONECTADO{Colors.ENDC}")
        print(f"  Dirección MAC: {ESP32_MAC}")
        print(f"  Dirección IP: {esp32_ip}")
        print(f"  Gateway: {GATEWAY_IP}")
        print(f"  Red: Gateway IoT Seguro (sin acceso a internet)")
        
    else:
        print()
        print_error("ESP32 NO ENCONTRADO en la red")
        print_info("Pasos para solucionar:")
        print("  1. Verifica que el ESP32 esté encendido")
        print("  2. Revisa las credenciales WiFi en sdkconfig.defaults")
        print("  3. Verifica que el hotspot gateway esté activo")
        print("  4. Revisa el monitor serial del ESP32 para errores de conexión")

def quick_adjust_ph():
    """
    Quick pH calibration adjustment with Device Shadow architecture (v3.5.0)
    
    ARCHITECTURE:
    1. User selects device (multi-device support)
    2. User enters slope/offset manually
    3. Python saves to MongoDB FIRST (Cloud Shadow = Source of Truth)
    4. Then publishes command to ESP32 for sync
    5. ESP32 applies + persists to NVS + sends ACK
    
    This inverts the old flow: MongoDB is now authoritative, ESP32 syncs FROM it.
    """
    print_header("Ajuste Rápido de pH - Multi-Device")
    
    print_info("Modo Device Shadow: MongoDB → ESP32")
    print_info("Los valores se guardan primero en la nube, luego se sincronizan al dispositivo")
    print()
    
    # ── Step 1: Device selection (same as Option 7) ──
    if not check_process_running('micro_ros_agent'):
        print_error("El micro-ROS Agent NO está corriendo")
        print_info("Inicia el Agent primero con la opción [1] del menú")
        return
    
    print_info("Detectando dispositivos ESP32 activos...")
    devices = discover_active_devices(duration_seconds=10)
    
    if not devices:
        print_error("No se detectaron dispositivos ESP32 en la red")
        print_info("Verifica que los ESP32 estén encendidos y publicando")
        return
    
    print()
    print(f"{Colors.BOLD}Dispositivos disponibles:{Colors.ENDC}")
    device_list = list(devices.keys())
    for idx, device_id in enumerate(device_list, start=1):
        data = devices[device_id]
        location = data.get('location', 'unknown')
        sensors = data.get('sensors', {})
        ph = sensors.get('ph', {}).get('value', 'N/A')
        print(f"  [{idx}] {device_id}")
        print(f"      Ubicación: {location}, pH actual: {ph}")
    print("  [0] Cancelar")
    print()
    
    device_choice = input(f"{Colors.OKBLUE}Selecciona dispositivo: {Colors.ENDC}").strip()
    
    try:
        device_idx = int(device_choice)
        if device_idx == 0:
            print_info("Operación cancelada")
            return
        if device_idx < 1 or device_idx > len(device_list):
            print_error("Opción inválida")
            return
        
        selected_device = device_list[device_idx - 1]
        print_success(f"✓ Dispositivo seleccionado: {selected_device}")
        print()
    except ValueError:
        print_error("Ingresa un número válido")
        return
    
    # ── Step 2: Get calibration values from user ──
    print_info("Ingresa los valores de calibración manual")
    print_info("(Estos valores se aplicarán a la ecuación: pH = slope * V + offset)")
    print()
    
    try:
        slope_input = input("Slope (Enter para 2.56): ").strip()
        slope = float(slope_input) if slope_input else 2.56
        
        offset_input = input("Offset (Enter para 0.47): ").strip()
        offset = float(offset_input) if offset_input else 0.47
    except ValueError:
        print_error("Formato de número inválido")
        return
    
    print()
    print(f"{Colors.BOLD}═══ Resumen ═══{Colors.ENDC}")
    print(f"Dispositivo: {selected_device}")
    print(f"Sensor: pH")
    print(f"Slope: {slope:.6f}")
    print(f"Offset: {offset:.6f}")
    print(f"Ecuación: pH = {slope:.3f} * V + {offset:.3f}")
    print()
    
    confirm = input(f"{Colors.WARNING}¿Aplicar estos valores? (S/n): {Colors.ENDC}").strip().lower()
    if confirm == 'n':
        print_info("Operación cancelada")
        return
    
    # ── Step 3: CLOUD-FIRST - Save to MongoDB (Device Shadow) ──
    print_info("Guardando en MongoDB (Device Shadow)...")
    
    try:
        from pymongo import MongoClient
        from pymongo.errors import ConnectionFailure
        
        mongodb_uri = os.getenv('MONGODB_URI')
        if not mongodb_uri:
            print_warning("MONGODB_URI no configurado - solo se aplicará localmente en ESP32")
            mongodb_saved = False
        else:
            try:
                client = MongoClient(mongodb_uri, serverSelectionTimeoutMS=5000)
                db = client[os.getenv('MONGODB_DATABASE', 'SistemasLab')]
                devices_col = db[os.getenv('MONGODB_COLLECTION_DEVICES', 'devices')]
                
                client.admin.command('ping')
                
                # Build calibration document
                cal_doc = {
                    'fecha': datetime.utcnow().isoformat() + 'Z',
                    'slope': slope,
                    'offset': offset,
                    'r_squared': 0.0,  # Manual calibration has no R²
                    'points': [],      # No calibration points for manual adjustment
                    'num_points': 0,
                    'method': 'manual_quick_adjust',
                    'status': 'pending_sync',  # Will become 'synced' after ESP32 ACK
                    'message': 'Manual adjustment via quick calibration'
                }
                
                result = devices_col.update_one(
                    {'_id': selected_device},
                    {
                        '$set': {
                            'calibracion.ph': cal_doc,
                            'ultima_calibracion': datetime.utcnow().isoformat() + 'Z'
                        }
                    },
                    upsert=False
                )
                
                if result.modified_count > 0 or result.matched_count > 0:
                    print_success("✓ Valores guardados en MongoDB (Device Shadow)")
                    mongodb_saved = True
                else:
                    print_warning(f"Dispositivo {selected_device} no encontrado en MongoDB")
                    print_info("Los valores se aplicarán solo en el ESP32")
                    mongodb_saved = False
                
                client.close()
                
            except ConnectionFailure:
                print_error("✗ No se pudo conectar a MongoDB")
                print_info("Los valores se aplicarán solo en el ESP32")
                mongodb_saved = False
            except Exception as e:
                print_error(f"Error guardando en MongoDB: {e}")
                mongodb_saved = False
    except ImportError:
        print_warning("pymongo no instalado - valores solo en ESP32")
        mongodb_saved = False
    
    # ── Step 4: Publish command to ESP32 for sync ──
    print_info("Enviando comando de sincronización al ESP32...")
    
    import json as json_lib
    cmd_json = json_lib.dumps({
        "sensor": "ph",
        "action": "calibrate",
        "points": [
            {"voltage": 1.0, "value": slope * 1.0 + offset},
            {"voltage": 2.0, "value": slope * 2.0 + offset}
        ]
    })
    
    # Use the existing publish_calibration_command function
    success = publish_calibration_command(cmd_json, device_id=selected_device)
    
    if success:
        print_success("✓ Calibración aplicada exitosamente")
        print_info("El dispositivo confirmó la recepción y aplicó los valores")
        
        # Update MongoDB status to 'synced' if we saved there
        if mongodb_saved:
            try:
                client = MongoClient(mongodb_uri, serverSelectionTimeoutMS=5000)
                db = client[os.getenv('MONGODB_DATABASE', 'SistemasLab')]
                devices_col = db[os.getenv('MONGODB_COLLECTION_DEVICES', 'devices')]
                
                devices_col.update_one(
                    {'_id': selected_device},
                    {'$set': {'calibracion.ph.status': 'synced'}}
                )
                
                print_success("✓ Estado actualizado en MongoDB: synced")
                client.close()
            except Exception:
                pass  # Non-fatal
    else:
        print_error("✗ El ESP32 no confirmó la calibración")
        print_info("Posibles causas:")
        print_info("  - ESP32 sufrió un PANIC al procesar el comando")
        print_info("  - Conexión micro-ROS inestable")
        print_info("  - Agent desconectado")
        print_info("")
        print_info("Los valores ESTÁN guardados en MongoDB (Device Shadow)")
        print_info("Puedes reintentar la sincronización cuando el ESP32 esté estable")

def show_menu():
    """Display main menu"""
    print_header("Gestor de Firmware Biofloc")
    
    print(f"{Colors.BOLD}Operaciones del Sistema:{Colors.ENDC}")
    print("  [1] Iniciar micro-ROS Agent")
    print("  [2] Iniciar Sensor DB Bridge")
    print("  [3] Monitorear Sensores (sin BD)")
    print("  [4] Verificar Estado del Sistema")
    print("  [5] Verificar Conectividad ESP32")
    print("  [6] 🩺 Monitor de Salud de Dispositivos (NUEVO)")
    print()
    
    print(f"{Colors.BOLD}Calibración:{Colors.ENDC}")
    print("  [7] ⭐ Calibración Remota Multi-Device (Recomendado)")
    print("  [8] Calibrar Sensor pH (3 puntos, USB)")
    print("  [9] Calibrar Sensor Temperatura (3 puntos, USB)")
    print("  [10] ⚡ Ajuste Rápido pH Multi-Device (valores manuales)")
    print("  [11] Ajuste Rápido Temperatura (-1.6°C)")
    print()
    
    print(f"{Colors.BOLD}Configuración:{Colors.ENDC}")
    print("  [12] Configurar Credenciales WiFi")
    print("  [13] Regenerar sdkconfig desde defaults")
    print()
    
    print(f"{Colors.BOLD}Firmware:{Colors.ENDC}")
    print("  [14] Compilar y Flashear Firmware")
    print()
    
    print(f"{Colors.BOLD}Otros:{Colors.ENDC}")
    print("  [0] Salir")
    print()

def main():
    """Main program loop"""
    
    # Check prerequisites once at startup
    if not check_prerequisites():
        print_error("Falló la verificación de prerequisitos")
        print_info("Por favor instala las herramientas requeridas")
        sys.exit(1)
    
    actions = {
        '1': start_microros_agent,
        '2': start_sensor_bridge,
        '3': monitor_sensors,
        '4': check_system_status,
        '5': check_esp32_connectivity,
        '6': monitor_device_health,
        '7': calibrate_remote,
        '8': calibrate_ph,
        '9': calibrate_temperature,
        '10': quick_adjust_ph,
        '11': quick_adjust_temperature,
        '12': configure_wifi,
        '13': regenerate_sdkconfig,
        '14': build_and_flash,
    }
    
    while True:
        try:
            show_menu()
            choice = input(f"{Colors.OKBLUE}Selecciona una opción: {Colors.ENDC}").strip()
            
            if choice == '0':
                print_info("Saliendo...")
                break
            
            if choice in actions:
                actions[choice]()
                input(f"\n{Colors.WARNING}Presiona Enter para continuar...{Colors.ENDC}")
            else:
                print_error("Opción inválida")
        
        except KeyboardInterrupt:
            print(f"\n{Colors.WARNING}Interrumpido por el usuario{Colors.ENDC}")
            input(f"\n{Colors.WARNING}Presiona Enter para volver al menú...{Colors.ENDC}")
        except Exception as e:
            print_error(f"Error inesperado: {e}")
            input(f"\n{Colors.WARNING}Presiona Enter para continuar...{Colors.ENDC}")

if __name__ == "__main__":
    main()
