import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the video
video_path = 'computer vision/singleball.mp4'  # Updated video path
cap = cv2.VideoCapture(video_path)

# Check if the video opened successfully
if not cap.isOpened():
    print("Error: Unable to open video.")
    exit()

# Initialize variables for tracking
prev_center = None
prev_time = None
velocity_data = []
frame_count = 0

# Read the video frame by frame
while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    frame_count += 1  # Increment frame count

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Threshold the grayscale image to get a binary image of the dot
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

    # Find contours in the binary image
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    if len(contours) > 0:
        # Get the largest contour
        contour = max(contours, key=cv2.contourArea)
        
        # Calculate the center of the contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            # Calculate velocity
            if prev_center is not None and prev_time is not None:
                # Get current time
                current_time = frame_count / cap.get(cv2.CAP_PROP_FPS)
                velocity = np.sqrt((center[0] - prev_center[0]) ** 2 + (center[1] - prev_center[1]) ** 2) / (current_time - prev_time)
                velocity_data.append(velocity)
            
            # Update prev_time with current_time
            prev_time = current_time if prev_center is not None and prev_time is not None else 0

            prev_center = center

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

# Release the VideoCapture object
cap.release()

# Close OpenCV window
cv2.destroyAllWindows()

# Create time array based on frame count and assumed frame rate (30 FPS)
default_fps = 30
time_array = np.arange(0, len(velocity_data), 1) / default_fps

'''
# Create time array
if cap.get(cv2.CAP_PROP_FPS) > 0:  # Check if frame rate is valid
    time_array = np.arange(0, len(velocity_data), 1) / cap.get(cv2.CAP_PROP_FPS)
else:
    print("Error: Invalid frame rate.")
    time_array = np.array([])  # Set time array to empty

was giving invalid frame rate
'''

# Plot velocity-time graph if both arrays have non-zero length
if len(velocity_data) > 0 and len(time_array) > 0:
    plt.plot(time_array, velocity_data)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (pixels/s)')
    plt.title('Velocity-Time Graph')
    plt.grid(True)
    plt.show()
else:
    print("Error: Unable to plot graph. Empty data arrays.")
