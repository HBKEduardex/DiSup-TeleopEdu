import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class PS4Controller:
    def __init__(self):
        self.LStickX = 0.0
        self.LStickY = 0.0
        self.RStickX = 0.0
        self.RStickY = 0.0

        self.L2 = 0
        self.R2 = 0

    def update(self, joy_msg):
        self.LStickX = joy_msg.axes[0]
        self.LStickY = joy_msg.axes[1]
        self.RStickX = joy_msg.axes[3]
        self.RStickY = joy_msg.axes[4]

        self.L2 = joy_msg.buttons[6]
        self.R2 = joy_msg.buttons[7]


class TeleopPS4(Node):
    def __init__(self):
        super().__init__('teleop_ps4_node')

        self.pub = self.create_publisher(Float32MultiArray, '/Data_PS4', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.controller = PS4Controller()

        self.get_logger().info("Nodo PS4 listo (envía PWM + LStickX)")

    def joy_callback(self, msg):
        self.controller.update(msg)

        # PWM temporal (como antes)
        if self.controller.R2:
            pwm_left = 50.0
            pwm_right = 50.0
        elif self.controller.L2:
            pwm_left = -50.0
            pwm_right = -50.0
        else:
            pwm_left = 0.0
            pwm_right = 0.0

        # Joystick de dirección
        stick = float(self.controller.LStickX)

        # Enviar:
        # [ PWM_LEFT, PWM_RIGHT, JOYSTICK_LX ]
        out = Float32MultiArray()
        out.data = [pwm_left, pwm_right, stick]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopPS4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()