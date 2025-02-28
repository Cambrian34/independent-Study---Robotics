#Apply grayscale conversion and edge detection (using OpenCV). Gaussian blur to reduce image noise.

import cv2

# Initialize the camera (0 is the default camera)
cap = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to capture image")
        break
    
    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)  # Threshold values (50, 150) can be adjusted

    # Display the processed frames
    cv2.imshow('Original', frame)
    cv2.imshow('Grayscale', gray)
    cv2.imshow('Blurred', blurred)
    cv2.imshow('Edges', edges)

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()