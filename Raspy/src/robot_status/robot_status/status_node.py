#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import can


class RobotStatus(Node):
    def __init__(self):
        super().__init__("robot_status")

        # Bus CAN
        self.bus = can.interface.Bus(channel="can0", bustype="socketcan")

        # Publicador de estado (rpm1, rpm2, vbat, source_id)
        self.pub = self.create_publisher(Float32MultiArray, "/robot_status", 10)

        # Timer para leer CAN
        self.timer = self.create_timer(0.01, self.read_can)

        self.get_logger().info("✔ robot_status escuchando CAN (IDs 0x140, 0x141)")

    def decode_frame(self, msg):
        """Convierte los bytes del frame CAN en datos reales."""

        # Bytes → enteros
        r1_raw = (msg.data[0] << 8) | msg.data[1]
        r2_raw = (msg.data[2] << 8) | msg.data[3]

        # Manejo de signo (int16)
        if r1_raw & 0x8000:
            r1_raw -= 65536

        if r2_raw & 0x8000:
            r2_raw -= 65536

        # RPM reales
        rpm1 = r1_raw / 10.0
        rpm2 = r2_raw / 10.0

        # Batería
        vbat_raw = (msg.data[4] << 8) | msg.data[5]
        vbat = vbat_raw / 100.0

        return rpm1, rpm2, vbat

    def read_can(self):
        """Lee un mensaje del CAN si existe."""
        msg = self.bus.recv(timeout=0.0005)
        if msg is None:
            return

        # Filtrar solo los IDs de los RP2040
        if msg.arbitration_id not in (0x140, 0x141):
            return

        rpm1, rpm2, vbat = self.decode_frame(msg)

        # Publicar mensaje en ROS2
        out = Float32MultiArray()
        out.data = [
            float(rpm1),
            float(rpm2),
            float(vbat),
            float(msg.arbitration_id)
        ]

        self.pub.publish(out)

        self.get_logger().info(
            f"[CAN {hex(msg.arbitration_id)}] RPM1={rpm1:.1f}  RPM2={rpm2:.1f}  VBAT={vbat:.2f}V"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
