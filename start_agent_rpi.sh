#!/bin/bash
# Script para iniciar el micro-ROS Agent en la Raspberry Pi
# 
# Instrucciones:
# 1. Ejecuta este script en la RPi 192.168.0.71:
#    ssh biofloc@192.168.0.71 "bash ~/Biofloc-Firmware-ROS/start_agent_rpi.sh"
# 
# 2. O accede manualmente y ejecuta:
#    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

set -e

echo "======================================================================="
echo "  Iniciando micro-ROS Agent en Raspberry Pi"
echo "======================================================================="

# Verificar si ROS2 está instalado
if ! command -v ros2 &> /dev/null; then
    echo "❌ Error: ROS2 no está instalado"
    echo "   Instala ROS2 Jazzy primero"
    exit 1
fi

# Verificar si el agent ya está corriendo
if pgrep -f "micro_ros_agent" > /dev/null; then
    echo "⚠️  micro-ROS Agent ya está corriendo"
    echo "   PID: $(pgrep -f 'micro_ros_agent')"
    exit 0
fi

# Source ROS2 setup
echo "Sourcear ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Iniciar el Agent en background
echo "Iniciando micro-ROS Agent en puerto 8888..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 &
AGENT_PID=$!

echo "✅ Agent iniciado (PID: $AGENT_PID)"
echo "   Esperando conexiones en 0.0.0.0:8888"
echo ""
echo "Para detener: kill $AGENT_PID"
echo "Para ver logs: ros2 topic list"
echo ""

wait $AGENT_PID
