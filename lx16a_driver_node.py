import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class LX16ADriver:
    def __init__(self, port='/dev/ttyUSB0'):
        self.ser = serial.Serial(port, 115200, timeout=0.1)

    def move_servo(self, ID, angle):
        pos = int((angle / 240.0) * 1000)
        cmd = bytearray([0x55, 0x55, ID, 7, 1, pos & 0xFF, (pos >> 8) & 0xFF, 0, 0])
        self.ser.write(cmd)

class LX16ADriverNode(Node):
    def __init__(self):
        super().__init__('lx16a_driver_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'leg_commands', self.listener_callback, 10)
        self.driver = LX16ADriver()

    def listener_callback(self, msg):
        for i, angle in enumerate(msg.data):
            self.driver.move_servo(i + 1, angle)

def main(args=None):
    rclpy.init(args=args)
    node = LX16ADriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
