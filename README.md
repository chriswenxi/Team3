# Team3

Guide:

**ESP32-IMU**

This folder holds the entire necessary code base to have an esp32 successfully reading off of the berryIMU, as well as the implementation for the MQTT service.

The arduino-BerryIMU.ino is what actually gets uploaded to the esp32 through the arduino IDE. This code was edited heavily to allow for functionality on the ESP32, as it was originally deisgned for something heavier duty like an arduino uno board.

If you are using our PCB, you must change the imu.cpp file. Please comment out the core of the detectIMU() function. The only thing you should leave in this function are the below two lines.

`BerryIMUversion = 3;`

`delay(1000);`

**Laptop**

This folder holds the code running from your laptop. As of right now this is only the skeleton code.

**RPI**

This folder holds things necessary for your RPI. 

The sub_IMU_handler.py is the main script for this folder. The client_sub_csv.py is a simple script that allows you to send the recieved data into a CSV file conviently to analyze some data if necessary.

The data folder holds a few csvs for the type of data we recieve from the IMU.

The mosquitto.conf file is a configuration file for the MQTT broker on the RPI, this can just be moved into /etc/mosquitto/ and replace the standard config file.

Some other instructions for having the mosquitto broker running is listed below.

To have MQTT broker working on rpi, you must install the correct packages into your environment

`sudo apt install mosquitto`

`sudo apt install mosquitto-clients`

For interacting with the mosquitto broker you just installed, use

`sudo systemctl status mosquitto.service`

`sudo systemctl start mosquitto.service`

`sudo systemctl stop mosquitto.service`

`sudo systemctl restart mosquitto.service `

You may need to create a config file for your mosquitto service
There is an example config file within the rpi folder
