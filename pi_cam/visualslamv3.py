import cv2
import numpy as np

'''
Author: Alistair Chambers

Version 3 of the visual SLAM script:
- Uses ORB feature tracking to map the room.
- Estimates camera motion from tracked features.
- Creates a room map using OpenCV and NumPy.
- Displays the map in real-time.
'''

# Open video capture
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# ORB Detector
orb = cv2.ORB_create(nfeatures=1000)
# BFMatcher with Hamming distance
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Read the first frame
ret, prev_frame = cap.read()
if not ret:
    print("Error: Couldn't capture the first frame.")
    cap.release()
    exit()

# Convert to grayscale and detect keypoints
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
prev_keypoints, prev_descriptors = orb.detectAndCompute(prev_gray, None)

# Initialize global map (2D points)
room_map = np.zeros((600, 600, 3), dtype=np.uint8)  # 2D map visualization
trajectory = np.array([[300, 300]], dtype=np.float32)  # Start in the center

# Camera pose relative to the first frame
R_total = np.eye(3)
t_total = np.zeros((3, 1))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(gray, None)

    if prev_descriptors is not None and descriptors is not None:
        matches = bf.match(prev_descriptors, descriptors)
        matches = sorted(matches, key=lambda x: x.distance)[:50]

        pts_prev = np.float32([prev_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts_curr = np.float32([keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        if len(pts_prev) > 8:
            E, mask = cv2.findEssentialMat(pts_curr, pts_prev, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            if E is not None:
                _, R, t, _ = cv2.recoverPose(E, pts_curr, pts_prev)
                t_total += R_total @ t  # Update total translation
                R_total = R @ R_total  # Update total rotation

                # Convert 3D motion to 2D for mapping
                map_x = int(300 + t_total[0, 0] * 50)  # Scale factor for visualization
                map_y = int(300 + t_total[2, 0] * 50)
                trajectory = np.vstack((trajectory, [map_x, map_y]))
                
                # Draw trajectory on the room map
                for i in range(1, len(trajectory)):
                    cv2.line(room_map, tuple(trajectory[i - 1]), tuple(trajectory[i]), (0, 255, 0), 1)

        matched_frame = cv2.drawMatches(prev_frame, prev_keypoints, frame, keypoints, matches, None, flags=2)
        cv2.imshow("Visual SLAM (ORB Feature Tracking)", matched_frame)
        cv2.imshow("Room Map", room_map)

    prev_gray = gray
    prev_keypoints = keypoints
    prev_descriptors = descriptors

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
