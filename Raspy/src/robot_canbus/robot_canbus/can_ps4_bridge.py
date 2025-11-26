#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import can

# ================================
# CONFIG CAN (4 motores)
# ================================
bus = can.interface.Bus(channel='can0', bustype='socketcan')


def send_pwm(m1, m2, m3, m4):
    """Enviar 4 PWM por CAN (-100 a 100)"""

    def clamp100(v):
        v = int(v)
        return max(-100, min(100, v))

    m1 = clamp100(m1)
    m2 = clamp100(m2)
    m3 = clamp100(m3)
    m4 = clamp100(m4)

    msg12 = can.Message(
        arbitration_id=0x120,
        data=[m1 & 0xFF, m2 & 0xFF],
        is_extended_id=False
    )

    msg34 = can.Message(
        arbitration_id=0x121,
        data=[m3 & 0xFF, m4 & 0xFF],
        is_extended_id=False
    )

    bus.send(msg12)
    bus.send(msg34)


class CANPS4Bridge(Node):
    def __init__(self):
        super().__init__('can_ps4_bridge')

        # Recibe TODOS los datos del mando PS4
        self.create_subscription(Float32MultiArray, '/Data_PS4',
                                 self.ps4_callback, 10)

        # Para video_tx (toggle streaming)
        self.pub_stream = self.create_publisher(Float32MultiArray,
                                                '/stream_cmd', 10)

        # Para UART (S / H hacia ESP32)
        self.pub_uart = self.create_publisher(String, '/uart_cmd', 10)

        self.get_logger().info(
            "Nodo CAN-PS4 listo (4 motores, giro sobre eje, stream + UART)"
        )

        # Estados para detectar flancos
        self.last_share = 0
        self.last_btn_S = 0     # Triangle → 'S'
        self.last_btn_H = 0     # Circle → 'H'

    # ================================
    # LÓGICA DE PWM (skid-steer)
    # ================================
    def compute_pwm(self, LX, R2, L2):
        """
        LX: joystick izq X  (-1 izq, +1 der)
        R2 / L2: adelante / atrás (0 ó 1)
        """

        MAX_PWM = 50      # velocidad base
        SPIN_PWM = 70     # para giro puro

        # 1) Dirección adelante / atrás
        if R2 and not L2:
            direction = 1
        elif L2 and not R2:
            direction = -1
        else:
            direction = 0

        # 2) Giro puro sobre el eje (sin R2/L2)
        if direction == 0 and abs(LX) > 0.2:
            spin = int(SPIN_PWM * abs(LX))
            if LX > 0:  # giro a la derecha
                left_pwm = spin      # izquierdas adelante
                right_pwm = -spin    # derechas atrás
            else:       # LX < 0 giro a la izquierda
                left_pwm = -spin
                right_pwm = spin
        else:
            # 3) Avance normal con giro suave
            base = direction * MAX_PWM
            k = 0.7
            left_factor = 1.0 - k * LX
            right_factor = 1.0 + k * LX

            left_pwm = base * left_factor
            right_pwm = base * right_factor

        # Limitar
        left_pwm = int(max(-MAX_PWM, min(MAX_PWM, left_pwm)))
        right_pwm = int(max(-MAX_PWM, min(MAX_PWM, right_pwm)))

        return left_pwm, right_pwm

    # ================================
    # CALLBACK PS4
    # ================================
    def ps4_callback(self, msg):
        data = msg.data

        if len(data) < 20:
            self.get_logger().warn("Mensaje PS4 incompleto.")
            return

        # Mapeo (coincide con teleop_ps4_multiarray.py de la laptop) :contentReference[oaicite:1]{index=1}
        LX = data[0]
        LY = data[1]
        RX = data[2]
        RY = data[3]

        L2 = data[4]
        R2 = data[5]

        # Botones
        Square   = data[6]
        Cross    = data[7]
        Circle   = data[8]
        Triangle = data[9]
        L1       = data[10]
        R1       = data[11]
        Share    = data[12]
        Options  = data[13]
        L3       = data[14]
        R3       = data[15]
        PS       = data[16]
        UP       = data[17]
        DOWN     = data[18]
        LEFT     = data[19]
        RIGHT    = data[20] if len(data) > 20 else 0

        # ---------- 1) STREAMING (Share) ----------
        if Share == 1 and self.last_share == 0:
            cmd = Float32MultiArray()
            cmd.data = [1.0]
            self.pub_stream.publish(cmd)
            self.last_share = 1
            self.get_logger().info("SHARE → toggle streaming")
        elif Share == 0:
            self.last_share = 0

        # ---------- 2) UART: 'S' y 'H' ----------
        # Ejemplo: Triangle = 'S', Circle = 'H'
        if Triangle == 1 and self.last_btn_S == 0:
            msg_s = String()
            msg_s.data = 'S'
            self.pub_uart.publish(msg_s)
            self.get_logger().info("UART CMD → 'S'")
            self.last_btn_S = 1
        elif Triangle == 0:
            self.last_btn_S = 0

        if Circle == 1 and self.last_btn_H == 0:
            msg_h = String()
            msg_h.data = 'H'
            self.pub_uart.publish(msg_h)
            self.get_logger().info("UART CMD → 'H'")
            self.last_btn_H = 1
        elif Circle == 0:
            self.last_btn_H = 0

        # ---------- 3) MOVIMIENTO 4 MOTORES ----------
        left_pwm, right_pwm = self.compute_pwm(LX=LX, R2=R2, L2=L2)

        # M1,M2 = lado izquierdo ; M3,M4 = lado derecho
        send_pwm(left_pwm, left_pwm, right_pwm, right_pwm)


def main(args=None):
    rclpy.init(args=args)
    node = CANPS4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
