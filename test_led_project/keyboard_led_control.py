#!/usr/bin/env python3
"""
Script de Control de LED por Teclado con ROS 2
==============================================

Controla un LED en ESP32 via micro-ROS presionando teclas:
  - Tecla 'e' o 'E' -> Enciende LED
  - Tecla 'a' o 'A' -> Apaga LED  
  - Tecla 't' o 'T' -> Alterna LED (toggle)
  - Tecla 'q' o 'Q' -> Salir

Uso:
    python3 keyboard_led_control.py
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardLEDController(Node):
    """Nodo ROS 2 que controla LED via teclado"""
    
    def __init__(self):
        super().__init__('keyboard_led_controller')
        
        # Publisher al tópico que escucha el ESP32
        self.publisher = self.create_publisher(String, '/led_control', 10)
        
        self.get_logger().info('🎮 Controlador de LED inicializado')
        self.get_logger().info('📡 Publicando en: /led_control')
    
    def send_command(self, command: str):
        """Envía comando al ESP32"""
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'📤 Comando enviado: {command}')


def get_key():
    """Lee una tecla sin esperar Enter (solo Linux/macOS)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def print_instructions():
    """Muestra instrucciones de uso"""
    print("\n" + "="*60)
    print("  🎮 Control de LED por Teclado")
    print("="*60)
    print("\n  Comandos disponibles:")
    print("    [E] - Encender LED  🟢")
    print("    [A] - Apagar LED    🔴")
    print("    [T] - Toggle LED    🔄")
    print("    [Q] - Salir         ❌")
    print("\n" + "="*60)
    print("  Presiona una tecla...\n")


def main():
    """Función principal"""
    
    # Inicializar ROS 2
    rclpy.init()
    controller = KeyboardLEDController()
    
    print_instructions()
    
    try:
        while True:
            # Leer tecla
            key = get_key().lower()
            
            # Procesar comando
            if key == 'e':
                print("  ➤ Encendiendo LED... 🟢")
                controller.send_command("ON")
                
            elif key == 'a':
                print("  ➤ Apagando LED... 🔴")
                controller.send_command("OFF")
                
            elif key == 't':
                print("  ➤ Alternando LED... 🔄")
                controller.send_command("TOGGLE")
                
            elif key == 'q':
                print("\n  ➤ Saliendo... 👋\n")
                break
                
            elif key == '\x03':  # Ctrl+C
                break
                
            else:
                print(f"  ⚠️  Tecla no reconocida: '{key}'")
    
    except KeyboardInterrupt:
        print("\n  ➤ Interrumpido por usuario 👋\n")
    
    finally:
        # Cleanup
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
