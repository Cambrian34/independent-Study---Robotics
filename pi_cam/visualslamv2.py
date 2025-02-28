import cv2
import numpy as np
''''
Author: Alistair Chambers

This script demonstrates visual SLAM using ORB feature tracking. It captures frames from the camera and tracks features
between consecutive frames using ORB (Oriented FAST and Rotated BRIEF) keypoints. The script estimates the camera motion
using the Essential Matrix and displays the feature tracking results.


'''


# Initialize video capture 
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# ORB Detector, uses Oriented FAST and Rotated BRIEF keypoints with 1000 features
orb = cv2.ORB_create(nfeatures=1000)  # Increase features for better tracking

# BFMatcher with Hamming distance (better for ORB)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Read first frame
ret, prev_frame = cap.read()
if not ret:
    print("Error: Couldn't capture the first frame.")
    cap.release()
    exit()

# Convert to grayscale and detect keypoints
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
prev_keypoints, prev_descriptors = orb.detectAndCompute(prev_gray, None)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale and detect keypoints
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(gray, None)

    # Match descriptors using BFMatcher
    if prev_descriptors is not None and descriptors is not None:
        matches = bf.match(prev_descriptors, descriptors)
        matches = sorted(matches, key=lambda x: x.distance)[:50]  # Keep best 50 matches

        # Extract matched keypoints
        pts_prev = np.float32([prev_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts_curr = np.float32([keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Estimate motion using Essential Matrix
        if len(pts_prev) > 8:
            E, mask = cv2.findEssentialMat(pts_curr, pts_prev, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            if E is not None:
                _, R, t, _ = cv2.recoverPose(E, pts_curr, pts_prev)
                print(f"Rotation:\n{R}\nTranslation:\n{t}")

        # Draw matches
        matched_frame = cv2.drawMatches(prev_frame, prev_keypoints, frame, keypoints, matches, None, flags=2)

        # Show feature tracking
        cv2.imshow("Visual SLAM (ORB Feature Tracking)", matched_frame)

    # Update previous frame data
    prev_gray = gray
    prev_keypoints = keypoints
    prev_descriptors = descriptors

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()