import paho.mqtt.client as mqtt

mqtt_broker = "test.mosquitto.org"  # IP address or hostname of your MQTT broker (ESP32)
mqtt_topic = "esp32/IMU"

def on_message(client, userdata, message):
    print(f"Received message: {message.payload.decode()}")

client = mqtt.Client()
client.on_message = on_message
client.connect(mqtt_broker, 1883, 10)
client.subscribe(mqtt_topic, qos=0)

while True:
    client.loop_start()

