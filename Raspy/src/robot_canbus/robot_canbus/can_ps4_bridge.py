import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can


bus = can.interface.Bus(channel='can0', bustype='socketcan')


def send_pwm(m1, m2, m3, m4):
    m1 = max(-100, min(100, int(m1)))
    m2 = max(-100, min(100, int(m2)))
    m3 = max(-100, min(100, int(m3)))
    m4 = max(-100, min(100, int(m4)))

    msg12 = can.Message(arbitration_id=0x120, data=[m1 & 0xFF, m2 & 0xFF], is_extended_id=False)
    msg34 = can.Message(arbitration_id=0x121, data=[m3 & 0xFF, m4 & 0xFF], is_extended_id=False)

    bus.send(msg12)
    bus.send(msg34)

    print(f"[CAN] PWM -> {m1}, {m2}, {m3}, {m4}")





class CANPS4Bridge(Node):
    def __init__(self):
        super().__init__('can_ps4_bridge')

        self.create_subscription(Float32MultiArray, '/Data_PS4', self.callback, 10)

        self.get_logger().info("Bridge CAN-PS4 iniciado ✔")

    def callback(self, msg):
        if len(msg.data) < 3:
            return

        pwm_left = msg.data[0]
        pwm_right = msg.data[1]
        stick = msg.data[2]   # LStickX

        # PWM a 4 motores
        send_pwm(pwm_left, pwm_right, pwm_lefst, pwm_right)

        # LStickX → ÁNGULO SERVOS
        print(f"[CAN] Angle -> {stick}")




def main(args=None):
    rclpy.init(args=args)
    node = CANPS4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
