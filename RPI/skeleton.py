import cv2
import mediapipe as mp
import time
import numpy as np
from collections import Counter

mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)
pTime = 0

# Initialize empty list to store previous joint positions
prev_joint_positions = []

# Smoothing factor (adjust as needed)
smoothing_factor = 0.5

# Initialize a list to store bruh values during calibration
calibration_bruh_values = []

# Calibration period (in seconds)
calibration_period = 5  # Adjust as needed

calibration_start_time = time.time()

reps = 0

cooldown_period = 3  # Cooldown period in seconds
cooldown_start_time = time.time()

while True:
    bruh = 0
    
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
            #mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
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

                if x_box < smoothed_cx < x_box + w_box and y_box < smoothed_cy < y_box + h_box:
                # Draw a circle on the smoothed joint within the bounding box in green
                    cv2.circle(img, (smoothed_cx, smoothed_cy), 5, (0, 255, 0), cv2.FILLED)
                else:
                    # Draw a circle on the smoothed joint outside the bounding box in red
                    cv2.circle(img, (smoothed_cx, smoothed_cy), 5, (0, 0, 255), cv2.FILLED)
                if  smoothed_cy < y_box:
                    bruh += 1
                    

        # Update previous joint positions for the next frame
        prev_joint_positions = current_joint_positions

        ## Calibration section: As of rn its used only for rep counter
        
        # Check if the calibration period is over
        if time.time() - calibration_start_time < calibration_period:
            # During calibration, store bruh values
            calibration_bruh_values.append(bruh)
        else:
            # After calibration, find the most common bruh value
            most_common_bruh = Counter(calibration_bruh_values).most_common(1)[0][0]
            print(f"Most common bruh value during calibration: {most_common_bruh}")
            if time.time() - cooldown_start_time > cooldown_period:
                if bruh > most_common_bruh:
                    reps += 1
                    cooldown_start_time = time.time()


    except Exception as e:
        print("Error:", e)
        continue

    cTime = time.time()
    fps = 1 / (cTime - pTime)
    pTime = cTime

    cv2.putText(img, f"Points above mask: {bruh} Reps: {reps}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
    
    # Display the combined image with transparency
    cv2.imshow("frame", img)

    cv2.waitKey(1)
