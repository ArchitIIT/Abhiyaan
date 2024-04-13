import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random

class PongGame(Node):
    def __init__(self):
        super().__init__('pong_game')  # Provide a node name
        self.create_turtles()
        self.timer_period = 0.1
        self.ball_speed = 1.0
        self.paddle_speed = 2.0
        self.create_timer(self.timer_period, self.update_game)
        self.ball_direction = random.choice([-1, 1])  # Random initial direction
        self.ball_linear_vel = self.ball_speed * self.ball_direction

    def create_turtles(self):
        self.create_turtle('ball', 5, 5, 0)
        self.create_turtle('paddle', 1, 5, 0)

    def create_turtle(self, name, x, y, theta):
        self.create_subscription(Pose, f'/{name}/pose', lambda msg: self.pose_callback(name, msg), 10)
        publisher = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        msg = Twist()
        msg.linear.x = 0.0  # Ensure 'x' is set as a float
        msg.angular.z = 0.0
        publisher.publish(msg)
        self.get_logger().info(f'{name} turtle spawned at position ({x}, {y})')

    def pose_callback(self, name, msg):
        if name == 'paddle':
            self.paddle_pose = msg

    def update_game(self):
        ball_vel_msg = Twist()
        paddle_vel_msg = Twist()

        # Ball movement
        ball_vel_msg.linear.x = self.ball_linear_vel
        self.publish('ball', ball_vel_msg)

        # Paddle control
        if hasattr(self, 'paddle_pose'):
            paddle_vel_msg.linear.x = self.paddle_speed * math.sin(0.5 * math.pi * (self.paddle_pose.y - 5))
            self.publish('paddle', paddle_vel_msg)

        # Check ball collision with walls
        ball_x = self.get_parameter_or('ball/x', 5.0)  # Default to 5.0 if parameter is not found
        if ball_x <= 0 or ball_x >= 11:
            self.ball_linear_vel *= -1  # Reverse ball direction on collision

    def publish(self, name, msg):
        publisher = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pong_game = PongGame()
    rclpy.spin(pong_game)
    pong_game.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

