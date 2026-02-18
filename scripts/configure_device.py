#!/usr/bin/env python3
"""
Script de configuración dinámica para dispositivos Biofloc ESP32
Permite cambiar intervalos de publicación y modo de agregación vía ROS2

Uso:
    python3 configure_device.py --mode normal         # 4s tiempo real
    python3 configure_device.py --mode ahorro         # 30min mediana
    python3 configure_device.py --mode monitoreo      # 5min promedio
    python3 configure_device.py --mode produccion     # 1min última muestra
    python3 configure_device.py --custom              # Configuración personalizada

Ejemplos:
    # Modo ahorro de datos (30 minutos)
    python3 configure_device.py --mode ahorro
    
    # Configuración personalizada
    python3 configure_device.py --custom \
        --sample-interval 10000 \
        --publish-interval 600000 \
        --aggregation average \
        --samples 60
"""

import argparse
import json
import sys
import time

# Configuraciones predefinidas
PRESETS = {
    'normal': {
        'name': 'Modo Normal (Tiempo Real)',
        'description': 'Lectura cada 4s, publicación inmediata. Ideal para calibración y monitoreo en tiempo real.',
        'config': {
            'sample_interval_ms': 4000,
            'publish_interval_ms': 4000,
            'mode': 'instant',
            'samples_per_publish': 1,
            'enabled': True
        },
        'data_reduction': '0%',
        'use_case': 'Calibración, desarrollo, monitoreo intensivo'
    },
    'ahorro': {
        'name': 'Modo Ahorro de Datos (30 minutos)',
        'description': 'Acumula 450 muestras @ 4s y publica mediana cada 30 min. Máximo ahorro de datos.',
        'config': {
            'sample_interval_ms': 4000,
            'publish_interval_ms': 1800000,  # 30 min
            'mode': 'median',
            'samples_per_publish': 450,
            'enabled': True
        },
        'data_reduction': '99.8% (1 mensaje vs 450)',
        'use_case': 'Piscicultura estable, optimización de red y base de datos'
    },
    'monitoreo': {
        'name': 'Modo Monitoreo Continuo (5 minutos)',
        'description': 'Promedio de 75 muestras @ 4s, publicación cada 5 min. Balance entre resolución y eficiencia.',
        'config': {
            'sample_interval_ms': 4000,
            'publish_interval_ms': 300000,  # 5 min
            'mode': 'average',
            'samples_per_publish': 75,
            'enabled': True
        },
        'data_reduction': '98.7% (1 mensaje vs 75)',
        'use_case': 'Monitoreo diario, cultivo en crecimiento'
    },
    'produccion': {
        'name': 'Modo Producción (1 minuto)',
        'description': 'Lectura cada 60s, publicación inmediata de última muestra. Alertas rápidas con menor carga.',
        'config': {
            'sample_interval_ms': 60000,  # 1 min
            'publish_interval_ms': 60000,
            'mode': 'last',
            'samples_per_publish': 1,
            'enabled': True
        },
        'data_reduction': '93.3% vs modo 4s',
        'use_case': 'Producción estándar, alertas automáticas'
    }
}

def print_preset_info(preset_name):
    """Imprime información detallada de un preset"""
    preset = PRESETS[preset_name]
    print(f"\n{'='*70}")
    print(f"  {preset['name']}")
    print(f"{'='*70}")
    print(f"\n📋 Descripción:")
    print(f"   {preset['description']}")
    print(f"\n⚙️  Configuración:")
    for key, value in preset['config'].items():
        if 'interval' in key:
            # Convertir a formato legible
            seconds = value / 1000
            if seconds >= 60:
                minutes = seconds / 60
                print(f"   • {key}: {value} ms ({minutes:.0f} minutos)")
            else:
                print(f"   • {key}: {value} ms ({seconds:.0f} segundos)")
        else:
            print(f"   • {key}: {value}")
    print(f"\n📊 Reducción de datos: {preset['data_reduction']}")
    print(f"🎯 Caso de uso: {preset['use_case']}")
    print(f"\n{'='*70}\n")

def generate_config_command(device_id, config):
    """Genera el comando JSON para configurar dispositivo"""
    command = {
        'device_id': device_id,
        'action': 'set_config',
        'config': config
    }
    return json.dumps(command, indent=2)

def print_ros2_command(device_id, config):
    """Imprime comando ROS2 para publicar configuración"""
    json_data = generate_config_command(device_id, config)
    
    print("\n🚀 Comando ROS2 (Terminal):")
    print("-" * 70)
    print(f"ros2 topic pub /biofloc/config_cmd std_msgs/String \\")
    # Escapar comillas para bash
    escaped_json = json_data.replace('"', '\\"').replace('\n', '').replace('  ', '')
    print(f"  \"{{data: '{escaped_json}'}}\"")
    print("-" * 70)
    
    print("\n💾 JSON de configuración:")
    print("-" * 70)
    print(json_data)
    print("-" * 70)

def validate_custom_config(args):
    """Valida configuración personalizada"""
    errors = []
    
    # Validar intervalos
    if args.sample_interval < 1000 or args.sample_interval > 60000:
        errors.append("sample_interval debe estar entre 1000ms (1s) y 60000ms (1min)")
    
    if args.publish_interval < 1000 or args.publish_interval > 3600000:
        errors.append("publish_interval debe estar entre 1000ms (1s) y 3600000ms (1h)")
    
    if args.publish_interval < args.sample_interval:
        errors.append("publish_interval debe ser >= sample_interval")
    
    # Validar samples
    if args.samples < 1 or args.samples > 450:
        errors.append("samples debe estar entre 1 y 450")
    
    # Validar modo
    valid_modes = ['instant', 'average', 'median', 'min_max', 'last']
    if args.aggregation not in valid_modes:
        errors.append(f"aggregation debe ser uno de: {', '.join(valid_modes)}")
    
    if errors:
        print("\n❌ Errores de validación:")
        for error in errors:
            print(f"   • {error}")
        return False
    
    return True

def main():
    parser = argparse.ArgumentParser(
        description='Configurador dinámico para dispositivos Biofloc ESP32',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Modos predefinidos:
  normal       - Tiempo real (4s, instant)
  ahorro       - Ahorro máximo (30min, median)
  monitoreo    - Balance (5min, average)
  produccion   - Producción (1min, last)

Ejemplos:
  %(prog)s --mode ahorro
  %(prog)s --mode normal --device biofloc_esp32_1234
  %(prog)s --custom --sample-interval 10000 --publish-interval 600000 --aggregation average
        """
    )
    
    parser.add_argument('--mode', choices=['normal', 'ahorro', 'monitoreo', 'produccion'],
                        help='Modo de configuración predefinido')
    parser.add_argument('--device', default='biofloc_esp32_c8e0',
                        help='ID del dispositivo (default: biofloc_esp32_c8e0)')
    parser.add_argument('--list', action='store_true',
                        help='Listar todos los modos disponibles')
    parser.add_argument('--custom', action='store_true',
                        help='Usar configuración personalizada')
    
    # Parámetros para modo custom
    parser.add_argument('--sample-interval', type=int, default=4000,
                        help='Intervalo de muestreo en ms (1000-60000)')
    parser.add_argument('--publish-interval', type=int, default=4000,
                        help='Intervalo de publicación en ms (1000-3600000)')
    parser.add_argument('--aggregation', default='instant',
                        choices=['instant', 'average', 'median', 'min_max', 'last'],
                        help='Modo de agregación de datos')
    parser.add_argument('--samples', type=int, default=1,
                        help='Número de muestras a agregar (1-450)')
    parser.add_argument('--enabled', type=bool, default=True,
                        help='Habilitar publicación (default: True)')
    
    args = parser.parse_args()
    
    # Listar modos disponibles
    if args.list:
        print("\n" + "="*70)
        print("  MODOS DE CONFIGURACIÓN DISPONIBLES")
        print("="*70)
        for preset_name in PRESETS.keys():
            print_preset_info(preset_name)
        return 0
    
    # Validar que se especificó modo o custom
    if not args.mode and not args.custom:
        parser.print_help()
        print("\n❌ Error: Debe especificar --mode o --custom")
        return 1
    
    # Generar configuración
    if args.custom:
        # Validar configuración personalizada
        if not validate_custom_config(args):
            return 1
        
        config = {
            'sample_interval_ms': args.sample_interval,
            'publish_interval_ms': args.publish_interval,
            'mode': args.aggregation,
            'samples_per_publish': args.samples,
            'enabled': args.enabled
        }
        
        print("\n" + "="*70)
        print("  CONFIGURACIÓN PERSONALIZADA")
        print("="*70)
        print(f"\n⚙️  Parámetros:")
        for key, value in config.items():
            print(f"   • {key}: {value}")
        print("\n" + "="*70)
    else:
        # Usar preset
        print_preset_info(args.mode)
        config = PRESETS[args.mode]['config']
    
    # Generar y mostrar comando ROS2
    print_ros2_command(args.device, config)
    
    print("\n✅ Configuración lista para aplicar")
    print(f"📡 Dispositivo objetivo: {args.device}")
    print("\n💡 Tip: Copia el comando ROS2 de arriba y pégalo en tu terminal con ROS2 activo")
    print("💡 Tip: Escucha respuesta con: ros2 topic echo /biofloc/config_status\n")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
