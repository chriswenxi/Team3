import cv2
import mediapipe as mp
import time
import numpy as np

mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
pTime = 0

# Initialize empty list to store previous joint positions
prev_joint_positions = []

# Smoothing factor (adjust as needed)
smoothing_factor = 0.5

while True:
    success, img = cap.read()
    # rgb for skeleton
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = pose.process(imgRGB)

    # hsv masking
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([30, 100, 100])
    upper_green = np.array([90, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Create a masked image
    masked_img = cv2.bitwise_and(img, img, mask=mask)

    try:
        contours, hierarchy = cv2.findContours(mask, 1, 2)
        c = max(contours, key=cv2.contourArea)
        x_box, y_box, w_box, h_box = cv2.boundingRect(c)
        cv2.rectangle(img, (x_box, y_box), (x_box+w_box, y_box+h_box), (0, 255, 0), 2)

        # Clear previous joint positions for the new frame
        current_joint_positions = []

        if results.pose_landmarks:
            mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
            for id, lm in enumerate(results.pose_landmarks.landmark):
                h, w, c = img.shape
                print(id, lm)
                cx, cy = int(lm.x * w), int(lm.y * h)

                # Apply smoothing to the joint positions
                if len(prev_joint_positions) > 0:
                    smoothed_cx = int(smoothing_factor * cx + (1 - smoothing_factor) * prev_joint_positions[id][0])
                    smoothed_cy = int(smoothing_factor * cy + (1 - smoothing_factor) * prev_joint_positions[id][1])
                else:
                    smoothed_cx, smoothed_cy = cx, cy

                # Store the current smoothed joint positions
                current_joint_positions.append((smoothed_cx, smoothed_cy))

                # Check if the joint is within the bounding box
                if x_box < smoothed_cx < x_box + w_box and y_box < smoothed_cy < y_box + h_box:
                    # Draw a circle on the smoothed joint within the bounding box in red
                    cv2.circle(img, (smoothed_cx, smoothed_cy), 5, (0, 0, 255), cv2.FILLED)
                else:
                    cv2.circle(img, (smoothed_cx, smoothed_cy), 5, (255, 0, 0), cv2.FILLED)

        # Update previous joint positions for the next frame
        prev_joint_positions = current_joint_positions

    except Exception as e:
        print("Error:", e)
        continue

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, str(int(fps)), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
    
    # Display the combined image with transparency
    cv2.imshow("frame", img)

    cv2.waitKey(1)

