#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

import serial
import threading
import time


class UARTBridge(Node):
    def __init__(self):
        super().__init__('uart_bridge')

        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Abrir puerto serie
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f"UART abierto en {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir UART: {e}")
            self.ser = None

        # Comandos S / H desde can_ps4_bridge
        self.create_subscription(String, '/uart_cmd',
                                 self.cmd_callback, 10)

        # Publicador de pH
        self.pub_ph = self.create_publisher(Float32, '/ph_value', 10)

        # Hilo de lectura continua
        self.running = True
        if self.ser is not None:
            self.read_thread = threading.Thread(target=self.read_loop,
                                                daemon=True)
            self.read_thread.start()

    def cmd_callback(self, msg: String):
        """Envía 'S' o 'H' por UART al ESP32."""
        if self.ser is None:
            return
        try:
            data = (msg.data + '\n').encode('utf-8')
            self.ser.write(data)
        except Exception as e:
            self.get_logger().error(f"Error enviando UART: {e}")

    def read_loop(self):
        """Lee líneas del ESP32 y publica el pH como Float32."""
        while self.running and self.ser is not None:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                try:
                    ph = float(line)
                    msg = Float32()
                    msg.data = ph
                    self.pub_ph.publish(msg)
                    self.get_logger().info(f"pH recibido: {ph:.2f}")
                except ValueError:
                    # línea no numérica
                    self.get_logger().warn(f"UART dato inválido: '{line}'")
            except Exception as e:
                self.get_logger().error(f"Error leyendo UART: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        self.running = False
        time.sleep(0.1)
        if hasattr(self, 'ser') and self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
