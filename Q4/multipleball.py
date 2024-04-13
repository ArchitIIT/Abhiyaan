import cv2
import numpy as np
import matplotlib.pyplot as plt

# Function to track green objects
def track_green_objects(video_path):
    cap = cv2.VideoCapture(video_path)

    # Initialize lists to store ball positions
    ball_positions = [[] for _ in range(5)]

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range of green color in HSV
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw rectangles around detected contours
        for i, contour in enumerate(contours):
            if cv2.contourArea(contour) > 100:  # Ignore small contours
                x, y, w, h = cv2.boundingRect(contour)
                ball_positions[i].append((x + w // 2, y + h // 2))  # Store center of the bounding rectangle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        



    cap.release()
    cv2.destroyAllWindows()

    return ball_positions

# Function to plot velocity-time graph
def plot_velocity_time(ball_positions):
    plt.figure(figsize=(10, 6))

    for i, positions in enumerate(ball_positions):
        if len(positions) < 2:
            continue

        x = [pos[0] for pos in positions]
        y = [pos[1] for pos in positions]

        # Calculate velocity
        velocity = np.sqrt(np.diff(x)**2 + np.diff(y)**2)

        # Plot velocity-time graph
        plt.plot(range(len(velocity)), velocity, label=f'Ball {i+1}')

    plt.xlabel('Frame')
    plt.ylabel('Velocity')
    plt.title('Velocity-Time Graph')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    video_path = 'computer vision/multiple balls.mov'  # Replace with your video path
    ball_positions = track_green_objects(video_path)
    plot_velocity_time(ball_positions)
