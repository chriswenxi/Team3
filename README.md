# Team3

To have MQTT working on rpi, you must install the correct packages into your environment

'sudo apt install mosquitto'

'sudo apt install mosquitto-clients'

For interacting with the mosquitto broker you just installed, use

'sudo systemctl status mosquitto.service'

'sudo systemctl start mosquitto.service'

'sudo systemctl stop mosquitto.service'

'sudo systemctl restart mosquitto.service' 

You may need to create a config file for your mosquitto service
There is an example config file within the rpi folder
