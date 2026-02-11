#!/bin/bash
# ============================================================================
# Script de Inicio Completo - Arquitectura de Gateway Seguro
# ============================================================================
# 
# Este script inicia todos los componentes del sistema en terminales separadas
# 
# Uso:
#   ./scripts/start_gateway_system.sh
#
# O ejecutar manualmente cada terminal como se muestra abajo
# ============================================================================

echo "============================================================"
echo "  🔒 Sistema Biofloc - Arquitectura de Gateway Seguro"
echo "============================================================"
echo ""
echo "Este script te guiará para iniciar el sistema completo."
echo "Necesitarás 3 terminales abiertas:"
echo ""
echo "  Terminal 1: micro-ROS Agent (UDP 8888)"
echo "  Terminal 2: Bridge Python (MongoDB)"
echo "  Terminal 3: ESP32 Monitor (Opcional)"
echo ""
echo "Presiona Enter para continuar..."
read

# ============================================================================
# Terminal 1: micro-ROS Agent
# ============================================================================
echo ""
echo "============================================================"
echo "  Terminal 1: micro-ROS Agent"
echo "============================================================"
echo ""
echo "Copia y ejecuta en la primera terminal:"
echo ""
echo "source /opt/ros/jazzy/setup.bash && \\"
echo "source ~/microros_ws/install/local_setup.bash && \\"
echo "ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
echo ""
echo "Deberías ver:"
echo "  [info] | UDPv4AgentLinux.cpp | init | running... | port: 8888"
echo ""
echo "Presiona Enter cuando el Agent esté corriendo..."
read

# ============================================================================
# Terminal 2: Bridge Python
# ============================================================================
echo ""
echo "============================================================"
echo "  Terminal 2: Bridge Python (MongoDB)"
echo "============================================================"
echo ""
echo "Copia y ejecuta en la segunda terminal:"
echo ""
echo "cd /home/Biofloc-Firmware-ROS/scripts && \\"
echo "source /opt/ros/jazzy/setup.bash && \\"
echo "source ~/microros_ws/install/local_setup.bash && \\"
echo "python3 sensor_db_bridge.py"
echo ""
echo "Deberías ver:"
echo "  [INFO] Sensor DB Bridge v3.1 Started (Secure Gateway Mode)"
echo "  [INFO] MongoDB Connected: True"
echo ""
echo "Presiona Enter cuando el Bridge esté corriendo..."
read

# ============================================================================
# Terminal 3: ESP32 Monitor (Opcional)
# ============================================================================
echo ""
echo "============================================================"
echo "  Terminal 3: ESP32 Monitor (Opcional)"
echo "============================================================"
echo ""
echo "Si quieres ver los logs del ESP32 en tiempo real:"
echo ""
echo "cd /home/Biofloc-Firmware-ROS && \\"
echo "source ~/esp/v5.3.4/esp-idf/export.sh && \\"
echo "idf.py -p /dev/ttyUSB0 monitor"
echo ""
echo "Deberías ver:"
echo "  I (3459) BIOFLOC: ⚠ No Internet access - Running in secure gateway mode"
echo "  I (4568) UROS: Agent is ONLINE"
echo "  I (5123) SENSOR: pH: 6.79 (2.452V) | Temp: 22.4°C (2.100V)"
echo ""
echo "Presiona Enter para continuar..."
read

# ============================================================================
# Verificación
# ============================================================================
echo ""
echo "============================================================"
echo "  Verificación del Sistema"
echo "============================================================"
echo ""
echo "Para verificar que todo funciona correctamente:"
echo ""
echo "python3 /home/Biofloc-Firmware-ROS/scripts/verify_secure_gateway.py"
echo ""
echo "Presiona Enter para ejecutar la verificación..."
read

cd /home/Biofloc-Firmware-ROS
python3 scripts/verify_secure_gateway.py

echo ""
echo "============================================================"
echo "  Sistema Iniciado"
echo "============================================================"
echo ""
echo "✅ Si todos los checks pasaron, el sistema está funcionando."
echo ""
echo "Para detener el sistema:"
echo "  - Ctrl+C en cada terminal"
echo ""
echo "Documentación:"
echo "  - Guía completa: SECURE_GATEWAY_MIGRATION.md"
echo "  - Troubleshooting: docs/TROUBLESHOOTING.md"
echo "  - Quick Start: QUICKSTART.md"
echo ""
