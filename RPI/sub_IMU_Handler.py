import paho.mqtt.client as mqtt
import time

calibration_duration = 50  # 10 messages per second * 5 seconds
calibration_data = []
baseline_data = []

# Error count variables
errorCount = 0
OpenCV_err_Flag = False
error_time = time.time()
true_error_time = time.time()

rpi_Process = False
stopFlag = False

def on_connect(client, userdata, flags, rc):
    global flag_connected
    flag_connected = 1
    client_subscriptions(client)
    print("Connected to MQTT server")

def on_disconnect(client, userdata, rc):
    global flag_connected
    flag_connected = 0
    print("Disconnected from MQTT server")

def callback_esp32_sensor1(client, userdata, msg):
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
        if is_movement_detected(current_data, baseline_data):
            if time.time() - true_error_time > 2:
                print("True error detected!")
                errorCount += 1
                true_error_time = time.time()
                # Update baseline when movement is detected
                baseline_data = current_data

def parse_imu_data(data_str):
    # Parse IMU data from string to a suitable data structure
    # Replace this with your actual parsing logic based on the IMU data format
    # Example: Splitting a comma-separated string into a list of values
    return [float(val) for val in data_str.split(',')]

def is_movement_detected(current_data, baseline_data):
    global OpenCV_err_Flag
    # Compare current data with baseline data to detect movement
    # Replace this with your actual movement detection logic
    # Example: Calculate the absolute difference between each corresponding value
    average_differences = [
        sum(abs(curr - baseline) for curr, baseline in zip(current_data, baseline_data)) / len(current_data)
    ]
    threshold = 10  # Adjust this threshold based on your specific use case
    if any(avg_diff > threshold for avg_diff in average_differences):
        if time.time() - error_time < 1:
            OpenCV_err_Flag = False
            return True
        else:
            return False

def Opencv(client, userdata, msg):
    global errorCount, error_time, OpenCV_err_Flag, rpi_Process, stopFlag
    converted_msg = str(msg.payload.decode('utf-8'))
    print('OpenCV message: ', converted_msg)
    if converted_msg == "Start":
        rpi_Process = True
    if converted_msg == "Workout Complete!":
        print("-------------------------------------------------------------\n")
        print("-------------------------------------------------------------\n")
        print("Workout Complete!\nTrue errors after 10 reps: ", errorCount)
        print("-------------------------------------------------------------\n")
        print("-------------------------------------------------------------\n")
        stopFlag = True
    if (converted_msg == "Error from X position") or (converted_msg == "Error from Y position"):
        error_time = time.time()
        OpenCV_err_Flag = True



def callback_rpi_broadcast(client, userdata, msg):
    print('RPi Broadcast message:  ', str(msg.payload.decode('utf-8')))

def client_subscriptions(client):
    client.subscribe("esp32/#")
    client.subscribe("TurnerOpenCV")
    client.subscribe("rpi/broadcast")

client = mqtt.Client("rpi_client_main")  # This should be a unique name
flag_connected = 0

client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.message_callback_add('esp32/sensor1', callback_esp32_sensor1)
client.message_callback_add('TurnerOpenCV', Opencv)
client.message_callback_add('rpi/broadcast', callback_rpi_broadcast)
client.connect('127.0.0.1', 1883)
# Start a new thread
client.loop_start()
client_subscriptions(client)
print("......client setup complete............")

while not stopFlag:
    time.sleep(4)
    if flag_connected != 1:
        print("trying to connect MQTT server..")
