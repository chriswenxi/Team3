from email.mime import base
import paho.mqtt.client as mqtt
import time

## Define Variables

calibration_duration = 50  # 10 messages per second * 5 seconds
calibration_data = []
baseline_data = []
# Error count variables
errorCount = 0
OpenCV_err_Flag = False
error_time = time.time()
true_error_time = time.time()
# Flags for starting up processes
rpi_Process = False
stopFlag = False
pushup_Pause = False

## Define Functions

def on_connect(client, userdata, flags, rc):
    global flag_connected
    flag_connected = 1
    client_subscriptions(client)
    print("Connected to MQTT server")

def on_disconnect(client, userdata, rc):
    global flag_connected
    flag_connected = 0
    print("Disconnected from MQTT server")

def parse_imu_data(data_str):
    # Parse IMU data from string to a suitable data structure
    # Replace this with your actual parsing logic based on the IMU data format
    # Example: Splitting a comma-separated string into a list of values
    return [float(val) for val in data_str.split(',')]

def client_subscriptions(client):
    client.subscribe("esp32/sensor1")
    client.subscribe("TurnerOpenCV")
    client.subscribe("Control")

def is_movement_detected_bicep(current_data, baseline_data):
    global OpenCV_err_Flag
    # Compare current data with baseline data to detect movement
    average_differences = [
        sum(abs(curr - baseline) for curr, baseline in zip(current_data, baseline_data)) / len(current_data)
    ]
    threshold = 5 
    if any(avg_diff > threshold for avg_diff in average_differences):
        if time.time() - error_time < 1:
            OpenCV_err_Flag = False
            return True
        else:
            return False

# Created a seperate one of these in case I need to change error timing for detection
def is_movement_detected_pushup(current_data, baseline_data):
    global pushup_Pause
    # Compare current data with baseline data to detect movement
    average_differences = [
        sum(abs(curr - baseline) for curr, baseline in zip(current_data, baseline_data)) / len(current_data)
    ]
    threshold = 40 
    if any(avg_diff > threshold for avg_diff in average_differences):
        if (time.time() - error_time < 1.5) and (time.time() - error_time > 0.5):
            pushup_Pause = False
            return True
        else:
            return False
## Callback Functions

# Callback to handle the esp32 data for the bicepcurl specifically
def callback_esp32_bicepCurl(client, userdata, msg):
    global calibration_data, baseline_data, errorCount, true_error_time
    if not rpi_Process:
        # If processing flag is False, return early without processing
        return

    print('ESP sensor1 data: ', msg.payload.decode('utf-8'))

    if len(calibration_data) < calibration_duration:
        # During calibration period, collect data to establish baseline
        calibration_data.append(parse_imu_data(msg.payload.decode('utf-8')))
    else:
        # After calibration, perform movement detection
        current_data = parse_imu_data(msg.payload.decode('utf-8'))
        if not baseline_data:
            # Set the initial baseline as the average of the calibration values
            baseline_data = [sum(val) / len(val) for val in zip(*calibration_data)]
        if is_movement_detected_bicep(current_data, baseline_data):
            if time.time() - true_error_time > 2:
                print("True error detected!")
                errorCount += 1
                true_error_time = time.time()
                # Update baseline when movement is detected
                baseline_data = current_data

def callback_OpenCV_bicepCurl(client, userdata, msg):
    global error_time, OpenCV_err_Flag
    converted_msg = str(msg.payload.decode('utf-8'))
    print('OpenCV message: ', converted_msg)

    if (converted_msg == "Error from X position") or (converted_msg == "Error from Y position"):
        error_time = time.time()
        OpenCV_err_Flag = True

def callback_esp32_Pushup(client, userdata, msg):
    # TO DO: add implementation
    # look for movement of IMU
    # desired is no movement
    global baseline_data, errorCount, true_error_time
    current_data = parse_imu_data(msg.payload.decode('utf-8'))
    print('ESP sensor1 data: ', msg.payload.decode('utf-8'))
    # No calibration needed for pushup IMU so just set the baseline data to current for the moment
    if len(baseline_data) == 0:
        baseline_data = current_data
    # Check for movement
    if pushup_Pause == True:
        if is_movement_detected_pushup(current_data, baseline_data):
            # If there is movement, we are supposed to be in pushup pause, and there hasnt been another error
            # in past 2 seconds then add another error
            print("Movement detected")
            baseline_data = current_data
            if time.time() - true_error_time > 2:
                print("True error detected!")
                errorCount += 1
                true_error_time = time.time()
                # Update baseline when movement is detected

def callback_OpenCV_Pushup(client, userdata, msg):
    # TO DO: add implementation
    # When we recieve the notification that the bottom of the pushup is reached
    # Ask esp32 callback to look for no movement for one second
    global pushup_Pause, error_time
    converted_msg = str(msg.payload.decode('utf-8'))
    print('OpenCV message: ', converted_msg)
    if converted_msg == "pushup pause begin":
        error_time = time.time()
        pushup_Pause = True

# This is the main function that determines the start/stop to sync data
# As well as determining which exercise the program is currently handling
def control_callback(client, userdata, msg):
    global rpi_Process, stopFlag, errorCount
    converted_msg = str(msg.payload.decode('utf-8'))

    if converted_msg == "1":
        client.message_callback_add('esp32/sensor1', callback_esp32_bicepCurl)
        client.message_callback_add('TurnerOpenCV', callback_OpenCV_bicepCurl)
    if converted_msg == "2":
        client.message_callback_add('esp32/sensor1', callback_esp32_Pushup)
        client.message_callback_add('TurnerOpenCV', callback_OpenCV_Pushup)
    if converted_msg == "Start":
        rpi_Process = True
    if converted_msg == "Workout Complete!":
        print("-------------------------------------------------------------\n")
        print("-------------------------------------------------------------\n")
        print("Workout Complete!\nTrue errors after 10 reps: ", errorCount)
        print("-------------------------------------------------------------\n")
        print("-------------------------------------------------------------\n")
        stopFlag = True
        client.disconnect()


## Basic main functionality
# define MQTT client & add the control callback
client = mqtt.Client("rpi_client_main")  # This should be a unique name
flag_connected = 0
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.message_callback_add('Control', control_callback)
client.connect('127.0.0.1', 1883)

# Start a new thread
client.loop_start()
client_subscriptions(client)
print("......client setup complete............")

while not stopFlag:
    time.sleep(4)
    if flag_connected != 1:
        print("trying to connect MQTT server..")
