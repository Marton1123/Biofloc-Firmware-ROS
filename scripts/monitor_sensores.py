#!/usr/bin/env python3
"""
Monitor de Sensores en Tiempo Real — Biofloc.

Muestra lecturas de pH y temperatura en tiempo real con estadísticas.
Útil para calibración y diagnóstico del sistema.

Uso:
    # Primero iniciar micro-ROS Agent en otro terminal
    python3 scripts/monitor_sensores.py
    
    # Opciones
    python3 scripts/monitor_sensores.py --samples 100  # Más muestras
    python3 scripts/monitor_sensores.py --verbose       # Más detalle

Principios aplicados:
    - Single Responsibility: Clases separadas para parseo, estadísticas y display
    - Clean Code: Nombres descriptivos, funciones pequeñas
    - Type Hints: Tipado estático para mejor documentación

Autor: Biofloc Engineering Team
Versión: 2.0.0
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional
import argparse
import json
import statistics
import sys

# =============================================================================
# MODELO DE DATOS
# =============================================================================

@dataclass
class SensorSample:
    """Una muestra individual del sensor."""
    ph: float
    temperature: float
    ph_voltage: float
    temp_voltage: float
    timestamp: datetime = field(default_factory=datetime.now)
    
    def __str__(self) -> str:
        return (
            f"pH: {self.ph:.2f} ({self.ph_voltage:.3f}V) | "
            f"Temp: {self.temperature:.1f}°C ({self.temp_voltage:.3f}V)"
        )


@dataclass
class SensorStatistics:
    """Estadísticas calculadas de las muestras."""
    ph_mean: float
    ph_std: float
    temp_mean: float
    temp_std: float
    ph_voltage_mean: float
    temp_voltage_mean: float
    sample_count: int
    
    def __str__(self) -> str:
        return (
            f"pH: {self.ph_mean:.2f} ± {self.ph_std:.3f} | "
            f"Temp: {self.temp_mean:.1f} ± {self.temp_std:.2f}°C | "
            f"Muestras: {self.sample_count}"
        )


# =============================================================================
# BUFFER DE MUESTRAS (Single Responsibility)
# =============================================================================

class SampleBuffer:
    """
    Buffer circular para almacenar muestras.
    
    Mantiene las últimas N muestras y calcula estadísticas.
    """
    
    def __init__(self, max_size: int = 50):
        self._samples: list[SensorSample] = []
        self._max_size = max_size
    
    def add(self, sample: SensorSample) -> None:
        """Agrega una muestra al buffer."""
        self._samples.append(sample)
        if len(self._samples) > self._max_size:
            self._samples.pop(0)
    
    def clear(self) -> None:
        """Limpia el buffer."""
        self._samples.clear()
    
    @property
    def count(self) -> int:
        """Número de muestras en el buffer."""
        return len(self._samples)
    
    @property
    def is_empty(self) -> bool:
        """Verifica si el buffer está vacío."""
        return len(self._samples) == 0
    
    @property
    def latest(self) -> Optional[SensorSample]:
        """Obtiene la última muestra."""
        return self._samples[-1] if self._samples else None
    
    def calculate_statistics(self) -> Optional[SensorStatistics]:
        """
        Calcula estadísticas de las muestras en el buffer.
        
        Returns:
            SensorStatistics si hay suficientes muestras, None si no
        """
        if len(self._samples) < 2:
            return None
        
        ph_values = [s.ph for s in self._samples]
        temp_values = [s.temperature for s in self._samples]
        ph_voltages = [s.ph_voltage for s in self._samples]
        temp_voltages = [s.temp_voltage for s in self._samples]
        
        return SensorStatistics(
            ph_mean=statistics.mean(ph_values),
            ph_std=statistics.stdev(ph_values),
            temp_mean=statistics.mean(temp_values),
            temp_std=statistics.stdev(temp_values),
            ph_voltage_mean=statistics.mean(ph_voltages),
            temp_voltage_mean=statistics.mean(temp_voltages),
            sample_count=len(self._samples),
        )


# =============================================================================
# PARSEADOR DE MENSAJES (Single Responsibility)
# =============================================================================

class MessageParser:
    """Parsea mensajes JSON del sensor a SensorSample."""
    
    def parse(self, json_string: str) -> Optional[SensorSample]:
        """
        Parsea un string JSON a SensorSample.
        
        Args:
            json_string: JSON del mensaje del sensor
            
        Returns:
            SensorSample si el parseo fue exitoso, None si falló
        """
        try:
            data = json.loads(json_string)
            sensors = data.get("sensors", {})
            
            ph_data = sensors.get("ph", {})
            temp_data = sensors.get("temperature", {})
            
            return SensorSample(
                ph=ph_data.get("value", 0.0),
                temperature=temp_data.get("value", 0.0),
                ph_voltage=ph_data.get("voltage", 0.0),
                temp_voltage=temp_data.get("voltage", 0.0),
            )
        except (json.JSONDecodeError, KeyError, TypeError):
            return None


# =============================================================================
# DISPLAY (Single Responsibility)
# =============================================================================

class Display:
    """Maneja la salida a consola."""
    
    SEPARATOR = "=" * 70
    LINE = "-" * 70
    
    def show_header(self) -> None:
        """Muestra encabezado del monitor."""
        print(self.SEPARATOR)
        print("📊 MONITOR DE SENSORES BIOFLOC")
        print(self.SEPARATOR)
        print()
        print("Recopilando lecturas... (Ctrl+C para finalizar)")
        print()
        print(self.LINE)
    
    def show_sample(self, sample: SensorSample) -> None:
        """Muestra una muestra individual."""
        timestamp = sample.timestamp.strftime("%H:%M:%S")
        print(f"[{timestamp}] {sample}")
    
    def show_statistics(self, stats: SensorStatistics) -> None:
        """Muestra estadísticas."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        print()
        print(f"📈 [{timestamp}] Estadísticas ({stats.sample_count} muestras):")
        print(f"   pH:          {stats.ph_mean:.2f} ± {stats.ph_std:.3f} (V: {stats.ph_voltage_mean:.3f})")
        print(f"   Temperatura: {stats.temp_mean:.1f}°C ± {stats.temp_std:.2f}")
        print(self.LINE)
    
    def show_final_report(self, stats: Optional[SensorStatistics]) -> None:
        """Muestra reporte final."""
        print()
        print(self.SEPARATOR)
        print("📋 REPORTE FINAL")
        print(self.SEPARATOR)
        
        if stats is None:
            print("⚠ No hay suficientes datos para generar estadísticas.")
            return
        
        print()
        print(f"   📊 Muestras recopiladas: {stats.sample_count}")
        print()
        print("   🔬 pH:")
        print(f"      Promedio:  {stats.ph_mean:.2f}")
        print(f"      Desv. Std: {stats.ph_std:.4f}")
        print(f"      Voltaje:   {stats.ph_voltage_mean:.3f}V")
        print()
        print("   🌡️  Temperatura:")
        print(f"      Promedio:  {stats.temp_mean:.1f}°C")
        print(f"      Desv. Std: {stats.temp_std:.3f}")
        print(f"      Voltaje:   {stats.temp_voltage_mean:.3f}V")
        print()
        print(self.SEPARATOR)
    
    def show_error(self, message: str) -> None:
        """Muestra mensaje de error."""
        print(f"✗ Error: {message}")
    
    def show_waiting(self) -> None:
        """Muestra mensaje de espera."""
        print("⏳ Esperando datos del sensor...")


# =============================================================================
# NODO ROS 2
# =============================================================================

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


if ROS_AVAILABLE:
    class SensorMonitorNode(Node):
        """
        Nodo ROS 2 para monitoreo de sensores.
        
        Suscribe a /biofloc/sensor_data y muestra lecturas en tiempo real.
        """
        
        def __init__(
            self,
            max_samples: int = 50,
            stats_interval: int = 10,
            verbose: bool = False,
        ):
            super().__init__("sensor_monitor")
            
            self._buffer = SampleBuffer(max_size=max_samples)
            self._parser = MessageParser()
            self._display = Display()
            self._stats_interval = stats_interval
            self._verbose = verbose
            self._message_count = 0
            
            # Suscripción
            self._subscription = self.create_subscription(
                String,
                "/biofloc/sensor_data",
                self._on_message,
                10,
            )
            
            self._display.show_header()
        
        def _on_message(self, msg: String) -> None:
            """Callback para mensajes recibidos."""
            sample = self._parser.parse(msg.data)
            
            if sample is None:
                if self._verbose:
                    self._display.show_error("Mensaje con formato inválido")
                return
            
            self._buffer.add(sample)
            self._message_count += 1
            
            # Mostrar cada muestra en modo verbose
            if self._verbose:
                self._display.show_sample(sample)
            
            # Mostrar estadísticas cada N mensajes
            if self._message_count % self._stats_interval == 0:
                stats = self._buffer.calculate_statistics()
                if stats:
                    self._display.show_statistics(stats)
        
        def show_final_report(self) -> None:
            """Muestra reporte final al cerrar."""
            stats = self._buffer.calculate_statistics()
            self._display.show_final_report(stats)


# =============================================================================
# PUNTO DE ENTRADA
# =============================================================================

def parse_arguments() -> argparse.Namespace:
    """Parsea argumentos de línea de comandos."""
    parser = argparse.ArgumentParser(
        description="Monitor de sensores Biofloc en tiempo real",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-s", "--samples",
        type=int,
        default=50,
        help="Número máximo de muestras en el buffer",
    )
    parser.add_argument(
        "-i", "--interval",
        type=int,
        default=10,
        help="Intervalo de mensajes para mostrar estadísticas",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Mostrar cada lectura individual",
    )
    return parser.parse_args()


def main() -> int:
    """
    Punto de entrada principal.
    
    Returns:
        Código de salida (0=éxito, 1=error)
    """
    if not ROS_AVAILABLE:
        print("✗ ERROR: ROS 2 (rclpy) no está instalado.")
        print("  Ejecuta: source /opt/ros/jazzy/setup.bash")
        return 1
    
    args = parse_arguments()
    
    rclpy.init()
    node = SensorMonitorNode(
        max_samples=args.samples,
        stats_interval=args.interval,
        verbose=args.verbose,
    )
    
    try:
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        node.show_final_report()
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
