import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('message_subscriber')

    def initialize_subscribers(self):
        self.subscriptions = {}
        topics = [
            "/start_here",
            "/pasta_way",
            "/you_are_welcome",
            "/bison",
            "/oops",
            "/rebooted"
        ]
        for topic in topics:
            self.subscriptions[topic] = self.create_subscription(
                String,
                topic,
                lambda msg, topic=topic: self.callback(msg, topic),
                10
            )

    def callback(self, msg, topic):
        self.get_logger().info(f"Received message on topic {topic}: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    node.initialize_subscribers()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
