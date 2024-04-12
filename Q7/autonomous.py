import pygame
import time
import random
import rclpy
from std_msgs.msg import String

# Initialize ROS node and subscriber
rclpy.init()
node = rclpy.create_node('obstacle_subscriber')
blocks = []  # Define blocks as a global variable

def obstacle_callback(msg):
    global blocks
    print("Received obstacle message:", msg.data)
    block_info = msg.data
    block = spawn_block(block_info)
    blocks.append(block)

def spawn_block(block_info):
    obstacle_info = block_info.split(',')
    print("Obstacle info:", obstacle_info)
    block_x = int(obstacle_info[0])
    block_y = int(obstacle_info[1])
    block_width = int(obstacle_info[2])
    block_height = int(obstacle_info[3])
    print("Creating block at (x={}, y={}) with width={} and height={}".format(block_x, block_y, block_width, block_height))
    return Block(block_x, block_y, block_width, block_height)

class Player:
    def __init__(self):
        self.image = carimg
        self.width = self.image.get_width()
        self.height = self.image.get_height()

        self.rect = self.image.get_rect()
        self.rect.x = int(wn_width * 0.5)
        self.rect.y = int(wn_height * 0.5)

        self.speedx = 0
        self.move_speed = 5  # Adjust the move speed as needed

    def update(self, obstacles):
        # Move player
        self.rect.x += self.speedx

        # Check boundary
        if self.rect.left < west_b:
            self.rect.left = west_b
        if self.rect.right > east_b:
            self.rect.right = east_b

        # Avoid obstacles
        for obstacle in obstacles:
            if self.rect.colliderect(obstacle.rect):
                if self.speedx > 0:
                    self.rect.right = obstacle.rect.left
                elif self.speedx < 0:
                    self.rect.left = obstacle.rect.right

class Block:
    def __init__(self, x, y, width, height):
        self.image = pygame.Surface((width, height))
        self.image.fill((255, 0, 0))  # Red color for the obstacle
        self.rect = self.image.get_rect()
        self.rect.x = x
        self.rect.y = y
        self.speedy = 5  # Adjust the speed as needed
        self.dodged = 0

    def update(self):
        self.rect.y += self.speedy
        if self.rect.top > wn_height:
            self.rect.y = 0 - self.rect.height
            self.rect.x = random.randrange(west_b, east_b - self.rect.width)
            self.dodged += 1

def game_loop():
    player = Player()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                quit()

        # Check for obstacle messages
        rclpy.spin_once(node, timeout_sec=0.1)  # Process any incoming messages

        # Update player
        player.update(blocks)

        # Update blocks
        for block in blocks:
            block.update()

        # Render game
        wn.blit(bg, (0, 0))
        wn.blit(player.image, (player.rect.x, player.rect.y))
        for block in blocks:
            wn.blit(block.image, (block.rect.x, block.rect.y))

        # Update display
        pygame.display.update()
        clock.tick(60)

# Pygame initialization
pygame.init()
clock = pygame.time.Clock()

# Window setup
wn_width = 500
wn_height = 400
wn = pygame.display.set_mode((wn_width, wn_height))
pygame.display.set_caption('Race car with road block')

# Load images
bg = pygame.image.load('Q7. Autonomous NFS/images/3lane.png')
DEFAULT_IMAGE_SIZE = (wn_width, wn_height)
bg = pygame.transform.scale(bg, DEFAULT_IMAGE_SIZE)

# Boundary
west_b = 100
east_b = 380

# Initialize ROS subscriber
subscriber = node.create_subscription(String, '/obstacle', obstacle_callback, 10)  # Set queue size to 1 to slow down message processing

# Start game loop
game_loop()
q