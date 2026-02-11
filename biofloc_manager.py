#!/usr/bin/env python3
"""
Biofloc Firmware Manager - Gestor centralizado del sistema
Provides a unified interface for all system operations
"""

import os
import sys
import subprocess
import time
from pathlib import Path

# Load environment variables
try:
    from dotenv import load_dotenv
    env_file = Path(__file__).parent / '.env'
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
    
    cmd = f"""
    source /opt/ros/jazzy/setup.bash && \
    source {MICROROS_WS}/install/local_setup.bash && \
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    """
    
    print_info("El Agent correrá en puerto UDP 8888")
    print_info("Presiona Ctrl+C para detener")
    print_info("IP del Gateway: 10.42.0.1")
    
    run_command(cmd, cwd=PROJECT_ROOT, check=False)

def start_sensor_bridge():
    """Start sensor DB bridge"""
    print_header("Iniciando Sensor DB Bridge")
    
    cmd = f"""
    cd {SCRIPTS_DIR} && \
    source /opt/ros/jazzy/setup.bash && \
    source {MICROROS_WS}/install/local_setup.bash && \
    python3 sensor_db_bridge.py
    """
    
    print_info("El Bridge se conectará a MongoDB y al topic ROS /biofloc/sensor_data")
    print_info("Presiona Ctrl+C para detener")
    
    run_command(cmd, check=False)

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
                            update_temp_calibration(slope=1000000, offset=offset_millidegrees)
            except Exception as e:
                print_warning(f"No se pudo auto-actualizar la configuración: {e}")
                print_info("Deberás actualizar manualmente")
        else:
            print_warning("Archivo de resultados de calibración no encontrado")
    else:
        print_error("Calibración de temperatura falló")

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
    update_temp_calibration(slope=1000000, offset=-1600)


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
    
    cmd = f"""
    source {ESP_IDF_PATH}/export.sh && \
    cd {PROJECT_ROOT} && \
    idf.py build && \
    idf.py -p /dev/ttyUSB0 flash
    """
    
    if run_command(cmd, cwd=PROJECT_ROOT, check=False):
        print_success("Firmware compilado y flasheado exitosamente")
        print_info("El ESP32 se reiniciará automáticamente")
    else:
        print_error("Falló la compilación o el flasheo")

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
    
    cmd = f"""
    cd {PROJECT_ROOT} && \
    rm -f sdkconfig && \
    source {ESP_IDF_PATH}/export.sh && \
    idf.py reconfigure
    """
    
    if run_command(cmd, check=False):
        print_success("sdkconfig regenerado desde defaults")
        
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
    
    # Check Gateway WiFi
    result = subprocess.run(
        "ip addr show wlo1 2>/dev/null | grep 10.42.0.1",
        shell=True,
        capture_output=True,
        text=True
    )
    
    if result.stdout:
        print_success("Gateway WiFi: ACTIVO (10.42.0.1)")
    else:
        print_warning("Gateway WiFi: NO ACTIVO")
    
    # Check ROS topics - get full list and filter in Python (grep pipe fails in subprocess)
    cmd = f"bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic list 2>/dev/null'"
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if "/biofloc/sensor_data" in result.stdout:
        print_success("Topic ROS: /biofloc/sensor_data DISPONIBLE")
        
        # Check if messages are being published (ESP32 publishes every 4s, wait max 8s)
        print_info("Verificando mensajes (esperando hasta 8s)...")
        cmd_echo = f"timeout 8 bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic echo /biofloc/sensor_data --once 2>&1'"
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
            print_warning("No se recibieron mensajes en 8s (ESP32 publica cada 4s)")
            print_info("Verifica que el ESP32 esté encendido y conectado al gateway WiFi")
    else:
        print_warning("Topic ROS: /biofloc/sensor_data NO ENCONTRADO")

def check_esp32_connectivity():
    """Check ESP32 network connectivity and IP address"""
    print_header("Verificación de Conectividad ESP32")
    

    print_info(f"Buscando ESP32 con MAC: {ESP32_MAC}")
    print()
    
    # Check if gateway WiFi is active
    result = subprocess.run(
        "ip addr show wlo1 2>/dev/null | grep 10.42.0.1",
        shell=True,
        capture_output=True,
        text=True
    )
    
    if not result.stdout:
        print_error("Gateway WiFi NO ACTIVO en wlo1")
        print_info("Inicia el hotspot gateway primero")
        return
    
    print_success(f"Gateway WiFi: ACTIVO ({GATEWAY_IP})")
    
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
        print_info("Verificando comunicación ROS (esperando mensaje hasta 8s)...")
        
        # First check if messages are being published (ESP32 publishes every 4s)
        cmd_echo = f"timeout 8 bash -c 'source /opt/ros/jazzy/setup.bash && source {MICROROS_WS}/install/local_setup.bash && ros2 topic echo /biofloc/sensor_data --once 2>&1'"
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
            print_warning("No se recibieron mensajes en 8s (ESP32 publica cada 4s)")
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
    """Quick pH calibration adjustment"""
    print_header("Ajuste Rápido de pH")
    
    print_info("Calibración actual: slope=2.559823, offset=0.469193")
    print_info("Esto permite ajuste manual de la calibración de pH")
    print()
    
    try:
        slope = float(input("Ingresa el slope de pH (o Enter para mantener actual): ").strip() or "2.559823")
        offset = float(input("Ingresa el offset de pH (o Enter para mantener actual): ").strip() or "0.469193")
        
        print()
        print_info(f"Se establecerá: slope={slope:.6f}, offset={offset:.3f}")
        
        choice = input("¿Continuar? (s/N): ").strip().lower()
        if choice == 's':
            update_calibration('ph', slope=slope, offset=offset)
        else:
            print_info("Cancelado")
    except ValueError:
        print_error("Formato de número inválido")

def show_menu():
    """Display main menu"""
    print_header("Gestor de Firmware Biofloc")
    
    print(f"{Colors.BOLD}Operaciones del Sistema:{Colors.ENDC}")
    print("  [1] Iniciar micro-ROS Agent")
    print("  [2] Iniciar Sensor DB Bridge")
    print("  [3] Monitorear Sensores (sin BD)")
    print("  [4] Verificar Estado del Sistema")
    print("  [5] Verificar Conectividad ESP32")
    print()
    
    print(f"{Colors.BOLD}Calibración:{Colors.ENDC}")
    print("  [6] Calibrar Sensor pH (3 puntos completo)")
    print("  [7] Calibrar Sensor Temperatura (3 puntos completo)")
    print("  [8] Ajuste Rápido pH (valores manuales)")
    print("  [9] Ajuste Rápido Temperatura (-1.6°C)")
    print()
    
    print(f"{Colors.BOLD}Configuración:{Colors.ENDC}")
    print("  [10] Configurar Credenciales WiFi")
    print("  [11] Regenerar sdkconfig desde defaults")
    print()
    
    print(f"{Colors.BOLD}Firmware:{Colors.ENDC}")
    print("  [12] Compilar y Flashear Firmware")
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
        '6': calibrate_ph,
        '7': calibrate_temperature,
        '8': quick_adjust_ph,
        '9': quick_adjust_temperature,
        '10': configure_wifi,
        '11': regenerate_sdkconfig,
        '12': build_and_flash,
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
