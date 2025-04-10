import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt

class MQTTInputNode(Node):
    def __init__(self):
        super().__init__('mqtt_input_node')
        self.publisher_ = self.create_publisher(String, 'motion_cmd', 10)
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect("test.mosquitto.org", 1883, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("robot/move")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode()
        self.get_logger().info(f"Received MQTT command: {command}")
        ros_msg = String()
        ros_msg.data = command
        self.publisher_.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MQTTInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
