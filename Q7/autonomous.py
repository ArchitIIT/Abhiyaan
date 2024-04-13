import pygame
import time
import random
import rclpy
from std_msgs.msg import String
from pygame import mixer

# Initialize ROS node and subscriber
rclpy.init()
node = rclpy.create_node('obstacle_subscriber')
subscriber = None
message_received = False  # Flag to track if a message has been received
blocks = []  # Define blocks as a global variable

def obstacle_callback(msg):
    global message_received
    print("Received obstacle message:", msg.data)
    block_info = msg.data
    block = spawn_block(block_info)
    blocks.append(block)
    message_received = True

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
        self.ai_control = False  # Flag to indicate AI control

    def update(self, obstacles):
        closest_obstacle = None
        closest_distance = float('inf')  # Initialize with a large value

        # Find the closest obstacle
        for obstacle in obstacles:
            obstacle_distance = abs(obstacle.x - self.rect.centerx)
            if obstacle_distance < closest_distance:
                closest_obstacle = obstacle
                closest_distance = obstacle_distance

        # Print debugging information
        print("Closest obstacle distance:", closest_distance)
        if closest_obstacle:
            print("Closest obstacle position:", closest_obstacle.x)
        
        # Adjust movement based on the closest obstacle
        if closest_obstacle:
            # Calculate distance to the center of the closest obstacle
            obstacle_center = closest_obstacle.x + closest_obstacle.width / 2
            car_center = self.rect.x + self.rect.width / 2
            distance_to_obstacle_center = obstacle_center - car_center

            # Determine movement direction based on obstacle position
            if distance_to_obstacle_center < -self.rect.width / 4:
                self.speedx = self.move_speed  # Move right
            elif distance_to_obstacle_center > self.rect.width / 4:
                self.speedx = -self.move_speed  # Move left
            else:
                self.speedx = 0  # Don't move if obstacle is centered
        else:
            # No obstacles, continue forward
            self.speedx = 0

        # Update player position
        self.rect.x += self.speedx

        # Check boundary
        if self.rect.left < west_b:
            self.rect.left = west_b
        if self.rect.right > east_b:
            self.rect.right = east_b

class Block:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.speedy = 5  # Adjust the speed as needed
        self.dodged = 0

    def update(self):
        self.y += self.speedy
        if self.y > wn_height:
            self.y = 0 - self.height
            self.x = random.randrange(west_b, east_b - self.width)
            self.dodged += 1

    def draw(self, wn):
        pygame.draw.rect(wn, RED, [self.x, self.y, self.width, self.height])

def game_loop():
    global message_received, blocks
    player = Player()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                quit()

        # Check for obstacle messages
        print("Checking for obstacle messages...")
        rclpy.spin_once(node, timeout_sec=0.1)  # Process any incoming messages
        if message_received:
            print("Obstacle message received.")
            message_received = False  # Reset the flag after processing the message
            player.ai_control = True  # Enable AI control when obstacles are present
        else:
            print("No obstacle message received.")
            player.ai_control = False  # Disable AI control when no obstacles are present

        # Update player
        print("Updating player...")
        player.update(blocks)

        # Update blocks
        print("Updating blocks...")
        for block in blocks:
            block.update()

        # Render game
        print("Rendering game...")
        wn.blit(bg, (0, 0))
        wn.blit(player.image, (player.rect.x, player.rect.y))
        for block in blocks:
            block.draw(wn)

            # Check for collisions
            if player.rect.colliderect(pygame.Rect(block.x, block.y, block.width, block.height)):
                crash()

        # Update display
        pygame.display.update()
        clock.tick(60)

# Pygame initialization
pygame.init()
mixer.init()
mixer.music.load('Initial D - Deja Vu.mp3')
mixer.music.set_volume(0.7)
mixer.music.play()
clock = pygame.time.Clock()

# RGB Color
BLACK = (0, 0, 0)
RED = (255, 0, 0)

# Window setup
wn_width = 500
wn_height = 400
wn = pygame.display.set_mode((wn_width, wn_height))
pygame.display.set_caption('Race car with road block')

# Load images
bg = pygame.image.load('images/3lane.png')
carimg = pygame.image.load('images/porsche.png')
DEFAULT_IMAGE_SIZE = (wn_width, wn_height)
bg = pygame.transform.scale(bg, DEFAULT_IMAGE_SIZE)
CAR_SIZE = (60, 100)
carimg = pygame.transform.scale(carimg, CAR_SIZE)

# Boundary
west_b = 100
east_b = 380

# Initialize ROS subscriber
subscriber = node.create_subscription(String, '/obstacle', obstacle_callback, 10)  # Set queue size to 1 to slow down message processing

# Start game loop
game_loop()

