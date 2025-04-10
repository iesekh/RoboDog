import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time

class CrawlGaitNode(Node):
    def __init__(self):
        super().__init__('crawl_gait_node')
        self.subscription = self.create_subscription(String, 'motion_cmd', self.cmd_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'leg_commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.current_command = 'stop'
        self.step_index = 0

    def cmd_callback(self, msg):
        self.get_logger().info(f"Motion command: {msg.data}")
        self.current_command = msg.data

    def timer_callback(self):
        if self.current_command == 'stop':
            angles = [90.0] * 20
        else:
            angles = self.generate_crawl_step(self.step_index)
            self.step_index = (self.step_index + 1) % 4

        msg = Float32MultiArray()
        msg.data = angles
        self.publisher.publish(msg)

    def generate_crawl_step(self, step):
        neutral = 90.0
        lift = 70.0
        extend = 110.0

        # Which leg is moving in each step
        active_legs = [0, 3, 1, 2]  # RF, LB, LF, RB

        angles = []
        for leg_index in range(4):
            base_id = leg_index * 5
            if leg_index == active_legs[step]:
                # Moving leg: Hip yaw, hip pitch (2), knee (2)
                angles.append(extend)  # Hip yaw
                angles.append(lift)    # Hip pitch 1
                angles.append(180 - lift)  # Hip pitch 2 (inverse)
                angles.append(extend)  # Knee 1
                angles.append(180 - extend)  # Knee 2 (inverse)
            else:
                angles.extend([neutral, neutral, 180 - neutral, neutral, 180 - neutral])

        return angles

def main(args=None):
    rclpy.init(args=args)
    node = CrawlGaitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
