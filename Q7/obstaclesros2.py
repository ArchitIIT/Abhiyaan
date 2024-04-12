import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import String

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.obstacle_pub = self.create_publisher(String, '/obstacle', 1)
        self.timer_period = 20.0  # Set the initial timer period to 2.0 seconds
        self.timer = self.create_timer(self.timer_period, self.publish_obstacle)

    def publish_obstacle(self):
        block_width = 80
        block_height = 20
        block_x = random.randrange(100, 380 - block_width)
        block_y = -100

        obstacle_msg = f"{block_x},{block_y},{block_width},{block_height}"
        self.get_logger().info(obstacle_msg)
        msg = String()
        msg.data = obstacle_msg
        self.obstacle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    obstacle_publisher = ObstaclePublisher()
    try:
        rclpy.spin(obstacle_publisher)
    except KeyboardInterrupt:
        pass

    obstacle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
