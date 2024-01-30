import paho.mqtt.client as mqtt
import cv2
import mediapipe as mp
import time
from time import sleep
import numpy as np
from collections import Counter
import speech_recognition as sr  # Added import for speech recognition
import math


# Name functions for skeleton usage
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# Get video directly from camera
cap = cv2.VideoCapture(0)
pTime = 0

## Initialize variables used for OpenCV
# Initialize empty list to store previous joint positions
prev_joint_positions = []
# Smoothing factor (adjust as needed) (for skeleton)
smoothing_factor = 0.5
# Initialize a list to store pabove values during calibration
calibration_pabove_values = []
# Calibration period (in seconds)
calibration_period = 5  # Adjust as needed
# Rep counter 
reps = 0
# Err Counter
errCounterX = 0
errCounterY = 0
errPause = 0
# Cooldown start values for counting reps
cooldown_period = 4  # Cooldown period (in seconds)
# Init lists to store desired joint positional values during calibration
calibration_joint_values_x = []
calibration_joint_values_y = []
# int to store id of desired joint
desiredJoint = 0
Joint_x = 0
Joint_y = 0
Shoulder_x = 0
Shoulder_y = 0
hip_x = 0
hip_y = 0

## MQTT functionality
# MQTT callback functions
def on_connect(client, userdata, flags, rc):
   global flag_connected
   flag_connected = 1
   print("Connected to MQTT server")

def on_disconnect(client, userdata, rc):
   global flag_connected
   flag_connected = 0
   print("Disconnected from MQTT server")


client = mqtt.Client("openCV client") # this should be a unique name
flag_connected = 0

client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.connect('192.168.99.113',1883)
#client.connect('131.179.39.148',1883)
# start a new thread
client.loop_start()
print("......client setup complete............")

## Speech Recognition
exercise_flag = 0

def ask_Exercise():
    global exercise_flag
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Please state your desired exercise.\n")
        print("Options: Bicep curl, Pushup, Squat, Leg Raise, Plank\n")
        audio = recognizer.listen(source)
    try:
        command = recognizer.recognize_google(audio).lower()
        if "bicep curl" in command:
            print("Bicep curl chosen\n")
            exercise_flag = 1
            return True
        if "push-up" in command:
            print("Pushup chosen\n")
            exercise_flag = 2
            return True
        if "push up" in command:
            print("Pushup chosen\n")
            exercise_flag = 2
            return True
        if "squat" in command:
            print("Squat chosen\n")
            exercise_flag = 3
            return True
        if "leg raise" in command:
            print("Leg Raise chosen\n")
            exercise_flag = 4
            return True
        if "plank" in command:
            print("Plank chosen\n")
            exercise_flag = 5
            return True
        else:
            print("The exercise you requested was: " + command)
            print("This exercise was not recognized. Please choose one of the options stated.\n")
            return False
    except sr.UnknownValueError:
        print("Could not understand audio. Please try again.")
        return False
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
        return False

def listen_for_start_command():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Say 'start' to begin the program.")
        audio = recognizer.listen(source)
    try:
        command = recognizer.recognize_google(audio).lower()
        if "start" in command:
            print("Start command recognized. Starting the program.")
            client.publish("Control", exercise_flag)
            client.publish("Control", "Start")
            return True
        else:
            print(command)
            print("Start command not recognized. Please say 'start' to begin.")
            return False
    except sr.UnknownValueError:
        print("Could not understand audio. Please try again.")
        return False
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
        return False

# Calculate angle.
def findAngle(x1, y1, x2, y2):
    theta = math.acos((y2 - y1) * (-y1) / (math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) * y1))
    degree = int(180 / math.pi) * theta
    return degree
## These will continualy prompt the user until they pass the prompts
# Ask for excercise
while not ask_Exercise():
    pass
# Ask for prompt to start the execerise program
while not listen_for_start_command():
    pass


## Bicep Curl code
if(exercise_flag == 1):
    # Set all time relevant values after we recieve the start command so that these dont begin at the wrong time and mess everything up
    cooldown_start_time = time.time()
    cooldown_start_time_err = time.time()
    calibration_start_time = time.time()

    breakflag = False
    counter = 0
    # This is your actual processing code
    while True:
        if counter >= 30:
            client.publish("Control", "Workout Complete!")
            sleep(1)
            break
        if reps >= 10 and counter == 0:
            breakflag = True
        # Reset pabove for every frame
        pabove = 0
        
        # Read frame
        success, img = cap.read()

        # rgb for skeleton
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = pose.process(imgRGB)

        # hsv masking
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([20, 100, 100])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Create a masked image
        masked_img = cv2.bitwise_and(img, img, mask=mask)

        try:
            # Bounding box code for masked item
            contours, hierarchy = cv2.findContours(mask, 1, 2)
            c = max(contours, key=cv2.contourArea)
            x_box, y_box, w_box, h_box = cv2.boundingRect(c)
            cv2.rectangle(img, (x_box, y_box), (x_box+w_box, y_box+h_box), (0, 255, 0), 2)

            # Clear previous joint positions for the new frame
            current_joint_positions = []

            if results.pose_landmarks:
                # Following line is the original skeleton drawing, but we dont really need the connections drawn or the points due to later code
                #mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
                for id, lm in enumerate(results.pose_landmarks.landmark):
                    h, w, c = img.shape
                    print(id, lm)
                    print(f"Image shape: {h} w {w}")
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
                        # Store desired joint since we're currently handling the desired joint if we havent alr set an id
                        if desiredJoint == 0:
                            desiredJoint = id
                        # If we are currently handling our desired joint, store the x and y for calibration/later testing
                        if desiredJoint == id:
                            Joint_x = smoothed_cx
                            Joint_y = smoothed_cy

                    else:
                        # Draw a circle on the smoothed joint outside the bounding box in red
                        cv2.circle(img, (smoothed_cx, smoothed_cy), 5, (0, 0, 255), cv2.FILLED)
                    # If a joint is above the y height of the bounding box, count it to pabove
                    if  smoothed_cy < y_box:
                        pabove += 1
                        

            # Update previous joint positions for the next frame
            prev_joint_positions = current_joint_positions

            ## Calibration section: As of rn its used only for rep counter

            # Check if the calibration period is over
            if time.time() - calibration_start_time < calibration_period:
                # During calibration, store pabove values
                calibration_pabove_values.append(pabove)
                # Find average position of desired joint (within mask) 
                if Joint_x != 0:
                    calibration_joint_values_x.append(Joint_x)
                    calibration_joint_values_y.append(Joint_y)

            ## Post Calibration functionality, this will all be done after the cal is over

            else:
                # After calibration, find the most common pabove value and store it
                most_common_pabove = Counter(calibration_pabove_values).most_common(1)[0][0]
                print(f"Most common pabove value during calibration: {most_common_pabove}")

                # After calibration, find the average position of the desired joint
                most_common_joint_x = np.mean(calibration_joint_values_x)
                most_common_joint_y = np.mean(calibration_joint_values_y)
                # Draw a box around my calibrated joint so I can visualize my movement threshold
                box_x = int(most_common_joint_x - 50)
                box_y = int(most_common_joint_y - 75)
                cv2.rectangle(img, (box_x, box_y), (box_x + 100, box_y + 150), (255, 255, 255), 2)

                print(f"Most common joint value during calibration_x: {most_common_joint_x}")
                print(f"Most common joint value during calibration_y: {most_common_joint_y}")
                print(f"DesiredJoint id : {desiredJoint}")
                # Count reps (currently only allow one rep per cooldown period length)
                if time.time() - cooldown_start_time > cooldown_period:
                    if pabove > most_common_pabove:
                        reps += 1
                        cooldown_start_time = time.time()
                if time.time() - cooldown_start_time_err > cooldown_period:
                    # Check for movement of our desired joint
                    if (Joint_x > most_common_joint_x + 50) or (Joint_x < most_common_joint_x - 50):
                        errCounterX += 1
                        data_to_publish = "Error from X position"
                        client.publish("TurnerOpenCV", data_to_publish)
                        cooldown_start_time_err = time.time()
                    if (Joint_y > most_common_joint_y + 75) or (Joint_y < most_common_joint_y - 75):
                        errCounterY += 1
                        data_to_publish = "Error from Y position"
                        client.publish("TurnerOpenCV", data_to_publish)
                        cooldown_start_time_err = time.time()


        except Exception as e:
            print("Error:", e)
            continue

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, f"Points above mask: {pabove} Reps: {reps} ErrorsX: {errCounterX} ErrorsY: {errCounterY}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        
        resized_img = cv2.resize(img, (1300, 700))
        # Display the combined image with transparency
        cv2.imshow("frame", resized_img)

        cv2.waitKey(1)
        if breakflag:
            counter += 1

## Pushup code
if( exercise_flag == 2):
    # Set all time relevant values after we recieve the start command so that these dont begin at the wrong time and mess everything up
    cooldown_start_time = time.time()
    calibration_start_time = time.time()
    # This is variables to stop processing
    breakflag = False
    counter = 0
    # This is your actual processing code
    while True:
        if counter >= 30:
            client.publish("Control", "Workout Complete!")
            sleep(1)
            break
        if reps >= 10 and counter == 0:
            breakflag = True
        
        # Read frame
        success, img = cap.read()

        # rgb for skeleton
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = pose.process(imgRGB)

        # hsv masking for our green sleeve
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([20, 100, 100])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Create a masked image
        masked_img = cv2.bitwise_and(img, img, mask=mask)

        try:
            # Bounding box code for masked item
            contours, hierarchy = cv2.findContours(mask, 1, 2)
            c = max(contours, key=cv2.contourArea)
            x_box, y_box, w_box, h_box = cv2.boundingRect(c)
            cv2.rectangle(img, (x_box, y_box), (x_box+w_box, y_box+h_box), (0, 255, 0), 2)

            # Clear previous joint positions for the new frame
            current_joint_positions = []

            if results.pose_landmarks:
                # Following line is the original skeleton drawing, but we dont really need the connections drawn or the points due to later code
                #mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
                for id, lm in enumerate(results.pose_landmarks.landmark):
                    h, w, c = img.shape
                    print(id, lm)
                    print(f"Image shape: {h} w {w}")
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
                        # Store desired joint since we're currently handling the desired joint if we havent alr set an id
                        if desiredJoint == 0:
                            desiredJoint = id
                        # If we are currently handling our desired joint, store the x and y for calibration/later testing
                        if desiredJoint == id:
                            Joint_x = smoothed_cx
                            Joint_y = smoothed_cy
            
                    else:
                        if id == 12:
                            Shoulder_x = smoothed_cx
                            Shoulder_y = smoothed_cy
                        # Draw a circle on the smoothed joint outside the bounding box in red
                        cv2.circle(img, (smoothed_cx, smoothed_cy), 5, (0, 0, 255), cv2.FILLED)
                        

            # Update previous joint positions for the next frame
            prev_joint_positions = current_joint_positions

            ## Calibration section: As of rn its used only for rep counter

            # Check if the calibration period is over
            if time.time() - calibration_start_time < calibration_period:
                # Find average position of desired joint (within mask) 
                if Joint_x != 0:
                    calibration_joint_values_x.append(Joint_x)
                    calibration_joint_values_y.append(Joint_y)

            ## Post Calibration functionality, this will all be done after the cal is over

            else:
                # After calibration, find the average position of the desired joint
                most_common_joint_x = np.mean(calibration_joint_values_x)
                most_common_joint_y = np.mean(calibration_joint_values_y)
                # draw the horizontal line for rep counting
                cv2.line(img, (0, int(most_common_joint_y)), (w, int(most_common_joint_y)), (255,0,0), 2)
                print(f"Most common joint value during calibration_x: {most_common_joint_x}")
                print(f"Most common joint value during calibration_y: {most_common_joint_y}")
                print(f"DesiredJoint id : {desiredJoint}")
                # Count reps (currently only allow one rep per cooldown period length)
                # TBD for pushups
                # I think create a horizontal line from start position of elbows, shoulder joint must go below that line
                # can also just handle the pause notification here, when rep get counted, aka when you've
                # hit the bottom of the rep, notify the IMU the rep has started and look for no movement for 1 second
                if time.time() - cooldown_start_time > 4: # only one rep per 4 seconds
                    if Shoulder_y > int(most_common_joint_y): # check if shoulder joint passes line
                        client.publish("TurnerOpenCV", "pushup pause begin")
                        reps += 1
                        cooldown_start_time = time.time()


        except Exception as e:
            print("Error:", e)
            continue

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, f"Reps: {reps} ErrorsX: {errCounterX} ErrorsY: {errCounterY}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        
        resized_img = cv2.resize(img, (1300, 700))
        # Display the combined image with transparency
        cv2.imshow("frame", resized_img)

        cv2.waitKey(1)
        if breakflag:
            counter += 1

## Squat code
if( exercise_flag == 3):
    # Set all time relevant values after we recieve the start command so that these dont begin at the wrong time and mess everything up
    cooldown_start_time = time.time()
    calibration_start_time = time.time()
    # This is variables to stop processing
    breakflag = False
    counter = 0
    # This is your actual processing code
    while True:
        if counter >= 30:
            client.publish("Control", "Workout Complete!")
            sleep(1)
            break
        if reps >= 10 and counter == 0:
            breakflag = True
        
        # Read frame
        success, img = cap.read()

        # rgb for skeleton
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = pose.process(imgRGB)

        # hsv masking for our green sleeve
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([20, 100, 100])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Create a masked image
        masked_img = cv2.bitwise_and(img, img, mask=mask)

        try:
            # Bounding box code for masked item
            contours, hierarchy = cv2.findContours(mask, 1, 2)
            c = max(contours, key=cv2.contourArea)
            x_box, y_box, w_box, h_box = cv2.boundingRect(c)
            cv2.rectangle(img, (x_box, y_box), (x_box+w_box, y_box+h_box), (0, 255, 0), 2)

            # Clear previous joint positions for the new frame
            current_joint_positions = []

            if results.pose_landmarks:
                # Following line is the original skeleton drawing, but we dont really need the connections drawn or the points due to later code
                #mpDraw.draw_landmarks(img, results.pose_landmarks, mpPose.POSE_CONNECTIONS)
                for id, lm in enumerate(results.pose_landmarks.landmark):
                    h, w, c = img.shape
                    print(id, lm)
                    print(f"Image shape: {h} w {w}")
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
                        # Store desired joint since we're currently handling the desired joint if we havent alr set an id
                        if desiredJoint == 0:
                            desiredJoint = id
                        # If we are currently handling our desired joint, store the x and y for calibration/later testing
                        if desiredJoint == id:
                            Joint_x = smoothed_cx
                            Joint_y = smoothed_cy
            
                    else:
                        # hip
                        if id == 24:
                            hip_x = smoothed_cx
                            hip_y = smoothed_cy
                        # Draw a circle on the smoothed joint outside the bounding box in red
                        cv2.circle(img, (smoothed_cx, smoothed_cy), 5, (0, 0, 255), cv2.FILLED)
                        

            # Update previous joint positions for the next frame
            prev_joint_positions = current_joint_positions

            ## Calibration section: As of rn its used only for rep counter

            # Check if the calibration period is over
            if time.time() - calibration_start_time < calibration_period:
                # Find average position of desired joint (within mask) 
                if Joint_x != 0:
                    calibration_joint_values_x.append(Joint_x)
                    calibration_joint_values_y.append(Joint_y)

            ## Post Calibration functionality, this will all be done after the cal is over

            else:
                # After calibration, find the average position of the desired joint
                most_common_joint_x = np.mean(calibration_joint_values_x)
                most_common_joint_y = np.mean(calibration_joint_values_y)
                # draw the horizontal line for rep counting
                cv2.line(img, (0, int(most_common_joint_y)), (w, int(most_common_joint_y)), (255,0,0), 2)
                print(f"Most common joint value during calibration_x: {most_common_joint_x}")
                print(f"Most common joint value during calibration_y: {most_common_joint_y}")
                print(f"DesiredJoint id : {desiredJoint}")
                # Count reps (currently only allow one rep per cooldown period length)
                # TBD for pushups
                # I think create a horizontal line from start position of elbows, shoulder joint must go below that line
                # can also just handle the pause notification here, when rep get counted, aka when you've
                # hit the bottom of the rep, notify the IMU the rep has started and look for no movement for 1 second
                if time.time() - cooldown_start_time > 4: # only one rep per 4 seconds
                    if hip_y > int(most_common_joint_y): # check if shoulder joint passes line
                        client.publish("TurnerOpenCV", "squat pause begin")
                        reps += 1
                        cooldown_start_time = time.time()


        except Exception as e:
            print("Error:", e)
            continue

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, f"Reps: {reps} ErrorsX: {errCounterX} ErrorsY: {errCounterY}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        
        resized_img = cv2.resize(img, (1300, 700))
        # Display the combined image with transparency
        cv2.imshow("frame", resized_img)

        cv2.waitKey(1)
        if breakflag:
            counter += 1

## Leg Raise code
if( exercise_flag == 4):
    blue = (255, 127, 0)
    red = (50, 50, 255)
    green = (127, 255, 0)
    dark_blue = (127, 20, 0)
    light_green = (127, 233, 100)
    yellow = (0, 255, 255)
    pink = (255, 0, 255)
    # Set all time relevant values after we recieve the start command so that these dont begin at the wrong time and mess everything up
    cooldown_start_time = time.time()
    calibration_start_time = time.time()
    # This is variables to stop processing
    breakflag = False
    counter = 0
    # variables for the line
    angle_deg = 15
    angle_rad = math.radians(angle_deg)
    line_len = 700
    # This is your actual processing code
    while True:
        if counter >= 30:
            client.publish("Control", "Workout Complete!")
            sleep(1)
            break
        if reps >= 10 and counter == 0:
            breakflag = True
        
        # Read frame
        success, img = cap.read()

        # rgb for skeleton
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = pose.process(imgRGB)

        # Clear previous joint positions for the new frame
        current_joint_positions = []

        # Following line is the original skeleton drawing, but we dont really need the connections drawn or the points due to later code
    
        lm = results.pose_landmarks
        lmPose = mpPose.PoseLandmark
        h, w, c = img.shape

        # Right shoulder
        r_shldr_x = int(lm.landmark[lmPose.RIGHT_SHOULDER].x * w)
        r_shldr_y = int(lm.landmark[lmPose.RIGHT_SHOULDER].y * h)
        # right ear
        r_ear_x = int(lm.landmark[lmPose.RIGHT_EAR].x * w)
        r_ear_y = int(lm.landmark[lmPose.RIGHT_EAR].y * h)
        # right hip
        r_hip_x = int(lm.landmark[lmPose.RIGHT_HIP].x * w)
        r_hip_y = int(lm.landmark[lmPose.RIGHT_HIP].y * h)
        # Right knee
        r_knee_x = int(lm.landmark[lmPose.RIGHT_KNEE].x * w)
        r_knee_y = int(lm.landmark[lmPose.RIGHT_KNEE].y * h)
        
        joints = [(r_shldr_x, r_shldr_y), (r_ear_x, r_ear_y), (r_hip_x, r_hip_y), (r_knee_x, r_knee_y)]
        # Apply smoothing to the joint positions
        for joint, (joint_x, joint_y) in enumerate(joints):
            if len(prev_joint_positions) > 0:
                smoothed_x = int(smoothing_factor * joint_x + (1 - smoothing_factor) * prev_joint_positions[joint][0])
                smoothed_y = int(smoothing_factor * joint_y + (1 - smoothing_factor) * prev_joint_positions[joint][1])
            else:
                smoothed_x, smoothed_y = joint_x, joint_y
            # Store the current smoothed joint positions
            current_joint_positions.append((smoothed_x, smoothed_y))

        
        # set for the next frame
        prev_joint_positions = current_joint_positions

        cv2.circle(img, (r_shldr_x, r_shldr_y), 7, yellow, -1)
        cv2.circle(img, (r_ear_x, r_ear_y), 7, yellow, -1)
        cv2.circle(img, (r_hip_x, r_hip_y), 7, yellow, -1)
        cv2.circle(img, (r_knee_x, r_knee_y), 7, yellow, -1)
        
        for joint_position in (current_joint_positions):
            joint_x, joint_y = joint_position
            cv2.circle(img, (joint_x, joint_y), 7, blue, -1)


        # During calibration period
        if (time.time() - calibration_start_time < 3):
            if r_hip_x != 0:
                calibration_joint_values_x.append(r_hip_x)
                calibration_joint_values_y.append(r_hip_y)
        # Not calibration period
        else:
            calibrated_hip_x = np.mean(calibration_joint_values_x)
            calibrated_hip_y = np.mean(calibration_joint_values_y)
            end_x = int(calibrated_hip_x + line_len * math.cos(angle_rad))
            end_y = int(calibrated_hip_y - line_len * math.sin(angle_deg))
            cv2.line(img, (int(calibrated_hip_x), int(calibrated_hip_y)), (end_x, end_y), red, 2)
            y_line_at_point = calibrated_hip_y + int((r_knee_x - calibrated_hip_x) * math.tan(angle_rad))

            # Count reps
            if time.time() - cooldown_start_time > 4: # only one rep per 4 seconds
                if r_knee_y < y_line_at_point:
                    reps += 1
                    cooldown_start_time = time.time()


        # Display
        resized_img = cv2.resize(img, (1300, 700))
        cv2.imshow('MediaPipe Pose', resized_img)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

        if breakflag:
            counter += 1

# plank code
if( exercise_flag == 5):
    blue = (255, 127, 0)
    red = (50, 50, 255)
    green = (127, 255, 0)
    dark_blue = (127, 20, 0)
    light_green = (127, 233, 100)
    yellow = (0, 255, 255)
    pink = (255, 0, 255)
    good_frames = 0
    good_time = 0
    bad_frames = 0
    fps = 0
    
    start_time = time.time()
    # This is your actual processing code
    while True:
        if time.time() - start_time > 180:
            break
        
        # Read frame
        success, img = cap.read()

        # rgb for skeleton
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = pose.process(imgRGB)

        if results.pose_landmarks is None:
            continue  # Skip to the next iteration if no person is detected
        # Clear previous joint positions for the new frame
        current_joint_positions = []

        # Following line is the original skeleton drawing, but we dont really need the connections drawn or the points due to later code
    
        lm = results.pose_landmarks
        lmPose = mpPose.PoseLandmark
        h, w, c = img.shape

        # Right shoulder
        r_shldr_x = int(lm.landmark[lmPose.RIGHT_SHOULDER].x * w)
        r_shldr_y = int(lm.landmark[lmPose.RIGHT_SHOULDER].y * h)

        # right hip
        r_hip_x = int(lm.landmark[lmPose.RIGHT_HIP].x * w)
        r_hip_y = int(lm.landmark[lmPose.RIGHT_HIP].y * h)
        # Right knee
        r_knee_x = int(lm.landmark[lmPose.RIGHT_KNEE].x * w)
        r_knee_y = int(lm.landmark[lmPose.RIGHT_KNEE].y * h)
        
        joints = [(r_shldr_x, r_shldr_y), (r_hip_x, r_hip_y), (r_knee_x, r_knee_y)]
        # Apply smoothing to the joint positions
        for joint, (joint_x, joint_y) in enumerate(joints):
            if len(prev_joint_positions) > 0:
                smoothed_x = int(smoothing_factor * joint_x + (1 - smoothing_factor) * prev_joint_positions[joint][0])
                smoothed_y = int(smoothing_factor * joint_y + (1 - smoothing_factor) * prev_joint_positions[joint][1])
            else:
                smoothed_x, smoothed_y = joint_x, joint_y
            # Store the current smoothed joint positions
            current_joint_positions.append((smoothed_x, smoothed_y))

        
        # set for the next frame
        prev_joint_positions = current_joint_positions

        cv2.circle(img, (r_shldr_x, r_shldr_y), 7, yellow, -1)
        cv2.circle(img, (r_hip_x, r_hip_y), 7, yellow, -1)
        cv2.circle(img, (r_knee_x, r_knee_y), 7, yellow, -1)
        
        for joint_position in (current_joint_positions):
            joint_x, joint_y = joint_position
            cv2.circle(img, (joint_x, joint_y), 7, blue, -1)


        # calculate angles
        
        knee_inclination = findAngle(r_knee_x, r_knee_y, r_hip_x, r_hip_y)
        torso_inclination = findAngle(r_hip_x, r_hip_y, r_shldr_x, r_shldr_y)
        # Put text, Posture and angle inclination.
        # Text string for display.
        angle_text_string = 'Knee : ' + str(int(knee_inclination)) + '  Torso : ' + str(int(torso_inclination))

        # calculate diff
        angle_diff = np.abs(knee_inclination - torso_inclination)
        # Determine whether good posture or bad posture.
        # The threshold angles have been set based on intuition.
        if angle_diff < 5:
            good_frames += 1
            
            cv2.putText(img, angle_text_string, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, light_green, 2)
            cv2.putText(img, str(int(knee_inclination)), (r_shldr_x + 10, r_shldr_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, light_green, 2)
            cv2.putText(img, str(int(torso_inclination)), (r_hip_x + 10, r_hip_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, light_green, 2)

            # Join landmarks.
            cv2.line(img, (r_shldr_x, r_shldr_y), (r_hip_x, r_hip_y), green, 4)
            cv2.line(img, (r_hip_x, r_hip_y), (r_knee_x, r_knee_y), green, 4)

        else:
            bad_frames += 1

            cv2.putText(img, angle_text_string, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, light_green, 2)
            cv2.putText(img, str(int(knee_inclination)), (r_shldr_x + 10, r_shldr_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, light_green, 2)
            cv2.putText(img, str(int(torso_inclination)), (r_hip_x + 10, r_hip_y), cv2.FONT_HERSHEY_SIMPLEX, 0.9, light_green, 2)

            # Join landmarks.
            cv2.line(img, (r_shldr_x, r_shldr_y), (r_hip_x, r_hip_y), green, 4)
            cv2.line(img, (r_hip_x, r_hip_y), (r_knee_x, r_knee_y), green, 4)

        # Calculate the time of remaining in a particular posture.
        if fps != 0:
            good_time = (1 / fps) * good_frames
            bad_time =  (1 / fps) * bad_frames

        # Pose time.
        if good_time > 0:
            time_string_good = 'Proper Plank time : ' + str(round(good_time, 1)) + 's'
            cv2.putText(img, time_string_good, (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.9, green, 2)
            time_string_bad = 'Bad Plank Time : ' + str(round(bad_time, 1)) + 's'
            cv2.putText(img, time_string_bad, (10, h - 50), cv2.FONT_HERSHEY_SIMPLEX,  0.9, red, 2)

        # Display
        resized_img = cv2.resize(img, (1300, 700))
        cv2.imshow('MediaPipe Pose', resized_img)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime


cap.release()
cv2.destroyAllWindows()