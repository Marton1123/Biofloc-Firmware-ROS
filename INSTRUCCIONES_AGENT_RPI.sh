#!/bin/bash
# PASOS PARA INICIAR EL MICRO-ROS AGENT EN LA RASPBERRY PI
# ============================================================
#
# El ESP32 está flashed y funcionando, pero no publica porque
# el micro-ROS Agent NO está corriendo en la RPi 192.168.0.71
#
# SOLUCIÓN: Ejecutar MANUALMENTE en la RPi:

echo "======================================================================="
echo "  INSTRUCCIONES PARA INICIAR AGENT ROS2"
echo "======================================================================="
echo ""
echo "1. Abre una terminal y conectate a la RPi:"
echo "   ssh biofloc@192.168.0.71"
echo ""
echo "2. Cuando pida contraseña, escribe la contraseña de biofloc"
echo ""
echo "3. Una vez conectado, ejecuta en la RPi:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
echo ""
echo "4. Verás salida como:"
echo "   [INFO] [1708270878.123456]: Starting Micro XRCE-DDS Agent..."
echo "   [INFO] [1708270878.234567]: Listening on 0.0.0.0:8888"
echo ""
echo "5. Luego en OTRA terminal (o en la del lab-ros2):"
echo "   python3 biofloc_manager.py"
echo "   Opción [6] Monitor de Salud - debería mostrar el ESP32"
echo ""
echo "======================================================================="
echo ""
echo "ALTERNATIVA (sin contraseña): Si tienes acceso local a RPi:"
echo "  1. Conecta monitor/keyboard a la RPi"
echo "  2. Abre terminal"
echo "  3. Ejecuta los comandos de paso 3"
echo ""
