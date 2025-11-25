import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can


# ================================================
# CONFIGuración del bus CAN
# ================================================
bus = can.interface.Bus(channel='can0', bustype='socketcan')


def send_pwm(m1, m2, m3, m4):
    """Enviar 4 PWM por CAN (-100 a 100)"""

    m1 = max(-100, min(100, int(m1)))
    m2 = max(-100, min(100, int(m2)))
    m3 = max(-100, min(100, int(m3)))
    m4 = max(-100, min(100, int(m4)))

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

    #print(f"[CAN] PWM -> M1:{m1}  M2:{m2}  M3:{m3}  M4:{m4}")


class CANPS4Bridge(Node):
    def __init__(self):
        super().__init__('can_ps4_bridge')

        # Recibe TODOS los datos del mando PS4
        self.create_subscription(Float32MultiArray, '/Data_PS4', self.ps4_callback, 10)
        self.pub_stream = self.create_publisher(Float32MultiArray, '/stream_cmd', 10)

        self.get_logger().info("Nodo CAN-PS4 iniciado ✔ (con lógica de movimiento)")
        self.last_share = 0
        
    def compute_pwm(self, LX, R2, L2, turbo_btn, turn_right, turn_left):

        # ================================
        # CONFIGURACIÓN
        # ================================
        #print("Buttons:", f"L2={L2}", f"R2={R2}", f"Turbo={turbo_btn}", f"TurnR={turn_right}", f"TurnL={turn_left}")
        MAX_PWM = 50       # Limite principal (puedes cambiarlo)
        TURBO    = 20      # Extra cuando se presiona Square
        TURN_PWM = 30      # PWM para giro brusco

        # ================================
        # 1) Dirección principal: adelante / atrás
        # ================================
        if R2 and not L2:
            direction = 1
        elif L2 and not R2:
            direction = -1
        else:
            direction = 0

        # NO avanzar si no hay dirección:
        base = direction * MAX_PWM

        # ================================
        # 2) Turbo
        # ================================
        if turbo_btn:
            base = min(MAX_PWM, base + TURBO)

        # ================================
        # 3) Dirección proporcional LX
        # LX = -1.0 (izq) → 1.0 (der)
        # ================================
        # reducción/ incremento proporcional
        left_factor  = 1.0 - LX
        right_factor = 1.0 + LX

        pwm_left  = base * left_factor
        pwm_right = base * right_factor

        # ================================
        # 4) Giros bruscos (L1 / R1)
        # PRIORIDAD TOTAL
        # ================================
        if turn_right and not turn_left:
            pwm_left  =  TURN_PWM
            pwm_right = -TURN_PWM

        elif turn_left and not turn_right:
            pwm_left  = -TURN_PWM
            pwm_right =  TURN_PWM

        # ================================
        # 5) Limitar PWM
        # ================================
        pwm_left  = int(max(-MAX_PWM, min(MAX_PWM, pwm_left)))
        pwm_right = int(max(-MAX_PWM, min(MAX_PWM, pwm_right)))
        print("pwm_left, pwm_right:", pwm_left, pwm_right)

        return pwm_left, pwm_right


        
    def ps4_callback(self, msg):
        data = msg.data

        if len(data) < 20:
            self.get_logger().warn("Mensaje PS4 incompleto.")
            return

        # ================================
        # Mapeo del array
        # ================================
        LX = data[0]
        LY = data[1]
        RX = data[2]
        RY = data[3]

        L2 = data[4]
        R2 = data[5]
        #print(f"L2: {L2}, R2: {R2}")
        # Botones 
        Cross   = data[6]
        Circle  = data[7]
        Triangle= data[8]
        Square  = data[9]
        
        L1      = data[10]
        R1      = data[11]
        Share   = data[12]
        Options = data[13]
        L3      = data[14]
        R3      = data[15]
        PS      = data[16]

        UP      = data[17]
        DOWN    = data[18]
        LEFT    = data[19]
        RIGHT   = data[20] if len(data) > 20 else 0
        #print("All buttons:", f"Square={Square}", f"Cross={Cross}", f"Circle={Circle}", f"Triangle={Triangle}",)
        # ================================
        # PUBLICAR COMANDO DE STREAMING
        # ================================
        if Share == 1 and self.last_share == 0:
            # flanco ascendente
            cmd = Float32MultiArray()
            cmd.data = [1.0]     # float
            self.pub_stream.publish(cmd)
            self.last_share = 1
            print("SHARE pressed → TOGGLE STREAM")

        elif Share == 0:
            # reset del estado
            self.last_share = 0

        # ================================
        # LOGICA DE MOVIMIENTO
        # ================================
        
        left_pwm, right_pwm = self.compute_pwm(
            LX=LX,
            R2=R2,
            L2=L2,
            turbo_btn=Square,
            turn_right=R1,
            turn_left=L1,
        )



        # ================================
        # Enviar a los 4 motores
        # ================================
        send_pwm(left_pwm, right_pwm, left_pwm, right_pwm)

        # Debug
        #print(f"[PS4] LX:{LX:.2f} LY:{LY:.2f}  -> L:{left_pwm:.0f} R:{right_pwm:.0f}")


def main(args=None):
    rclpy.init(args=args)
    node = CANPS4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
