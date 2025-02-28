import cv2
import numpy as np

# Initialize video capture (0 for USB camera, change if using Pi Camera)
cap = cv2.VideoCapture(0)

# Initialize ORB detector
orb = cv2.ORB_create(500)  # Detect up to 500 keypoints

# Create FLANN matcher
FLANN_INDEX_LSH = 6
index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params, search_params)

# Read first frame
ret, prev_frame = cap.read()
if not ret:
    print("Error: Couldn't capture the first frame.")
    cap.release()
    exit()

#Convert to grayscale and detect keypoints
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
prev_keypoints, prev_descriptors = orb.detectAndCompute(prev_gray, None)


while cap.isOpened():
    # Read frame
    ret, frame = cap.read()
    if not ret:
        break
# Convert to grayscale and detect keypoints
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = orb.detectAndCompute(gray, None)

    # Match descriptors using FLANN
    if prev_descriptors is not None and descriptors is not None and len(prev_descriptors) > 10:
        
        matches = flann.knnMatch(prev_descriptors, descriptors, k=2)

        # Apply Lowe's ratio test to filter good matches
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        # Draw matches
        matched_frame = cv2.drawMatches(prev_frame, prev_keypoints, frame, keypoints, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        # Display
        cv2.imshow("Feature Tracking", matched_frame)

    # Update previous frame and keypoints
    prev_gray = gray
    prev_keypoints = keypoints
    prev_descriptors = descriptors

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()